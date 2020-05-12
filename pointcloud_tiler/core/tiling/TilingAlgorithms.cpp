#include "tiling/TilingAlgorithms.h"
#include "debug/ProgressReporter.h"

#include "containers/DestructuringIterator.h"
#include "terminal/stdout_helper.h"
#include "threading/Parallel.h"

#include <mutex>

/**
 * The maximum depth of an octree that is representable with a single MortonIndex
 */
constexpr static uint32_t MAX_OCTREE_LEVELS = 21;
/**
 * The minimum number of points in a node needed in order for that node to be processed
 * asynchronously
 */
constexpr static size_t MIN_POINTS_FOR_ASYNC_PROCESSING = 100'000;

#pragma region helper_functions
/**
 * Reads the cached points for the given node from disk and returns them as IndexedPoints
 */
static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
read_pnts_from_disk(const std::string& node_name,
                    const AABB& octree_bounds,
                    const AABB& node_bounds,
                    PointsCache& points_cache,
                    PointsPersistence& persistence)
{
  PointBuffer tmp_points;
  persistence.retrieve_points(node_name, tmp_points);
  if (!tmp_points.count())
    return {};

  auto& points = points_cache.emplace_points(std::move(tmp_points));

  std::vector<IndexedPoint<MortonIndex64Levels>> indexed_points;
  indexed_points.reserve(points.count());
  try {
    index_points<MortonIndex64Levels>(
      std::begin(points),
      std::end(points),
      std::back_inserter(indexed_points),
      octree_bounds,
      OutlierPointsBehaviour::ClampToBounds); // Clamp to fix floating point
                                              // errors
  } catch (const std::runtime_error& ex) {
    throw std::runtime_error{
      (boost::format("Indexing points of node %1% failed:\n%2%") % node_name % ex.what()).str()
    };
  }

  return indexed_points;
}
#pragma endregion

#pragma region processing_functions

/**
 * Takes a sorted range of IndexedPoints and splits it up into up to eight ranges, one for each
 * child node. This method then returns the appropriate NodeTilingData for tiling each of the child
 * nodes
 */
static std::vector<NodeTilingData>
split_range_into_child_nodes(std::vector<IndexedPoint64>::iterator points_begin,
                             std::vector<IndexedPoint64>::iterator points_end,
                             octree::NodeStructure const& node,
                             octree::NodeStructure const& root_node)
{
  const auto child_level = node.level + 1;
  const auto child_ranges = partition_points_into_child_octants(
    points_begin, points_end, static_cast<uint32_t>(child_level));

  std::vector<NodeTilingData> child_tiling_data;
  child_tiling_data.reserve(8);

  for (uint8_t octant = 0; octant < 8; ++octant) {
    const auto& child_range = child_ranges[octant];
    if (child_range.first == child_range.second)
      continue;

    auto child_node = node;
    child_node.morton_index.set_octant_at_level(child_level, octant);
    child_node.bounds = get_octant_bounds(octant, node.bounds);
    child_node.level = child_level;
    child_node.max_spacing /= 2;
    child_node.name = concat(node.name, static_cast<char>('0' + octant));

    // Copy the indices into a new vector and pass it on to the child processing
    child_tiling_data.emplace_back(
      octree::NodeData{ child_range.first, child_range.second }, child_node, root_node);
  }

  return child_tiling_data;
}

#pragma endregion

#pragma region TilingAlgorithmV1

TilingAlgorithmV1::TilingAlgorithmV1(SamplingStrategy& sampling_strategy,
                                     ProgressReporter* progress_reporter,
                                     PointsPersistence& persistence,
                                     TilerMetaParameters meta_parameters,
                                     size_t concurrency)
  : _sampling_strategy(sampling_strategy)
  , _progress_reporter(progress_reporter)
  , _persistence(persistence)
  , _meta_parameters(meta_parameters)
  , _concurrency(concurrency)
{}

void
TilingAlgorithmV1::build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf)
{
  _root_node_points.clear();
  _root_node_points.resize(points.count());
  _points_cache.clear();

  auto indexing_tasks = parallel::transform(
    std::begin(points),
    std::end(points),
    std::begin(_root_node_points),
    [this, &bounds](PointBuffer::PointReference point) {
      return index_point<MAX_OCTREE_LEVELS>(point, bounds, OutlierPointsBehaviour::ClampToBounds);
    },
    tf,
    _concurrency,
    "calc_morton_indices");

  auto sort_task =
    tf.emplace([this]() { std::sort(_root_node_points.begin(), _root_node_points.end()); })
      .name("sort");

  octree::NodeStructure root_node;
  root_node.bounds = bounds;
  root_node.level = -1;
  root_node.max_depth = _meta_parameters.max_depth;
  root_node.max_spacing = _meta_parameters.spacing_at_root;
  root_node.morton_index = {};
  root_node.name = "r";

  auto process_task =
    tf.emplace([this, root_node](tf::Subflow& subflow) mutable {
        do_tiling_for_node(std::move(_root_node_points), root_node, root_node, subflow);
        _root_node_points = {};
      })
      .name(concat(root_node.name, " [", _root_node_points.size(), "]"));

  indexing_tasks.second.precede(sort_task);
  sort_task.precede(process_task);
}

/**
 * Tile the given node as a terminal node, i.e. take up to 'max_points_per_node' points
 * and persist them without any sampling
 */
void
TilingAlgorithmV1::tile_terminal_node(octree::NodeData const& all_points,
                                      octree::NodeStructure const& node,
                                      size_t previously_taken_points_count)
{
  const auto points_to_take = std::min(all_points.size(), _meta_parameters.max_points_per_node);
  if (points_to_take < all_points.size()) {
    const auto dropped_points_count = all_points.size() - points_to_take;
    util::write_log(
      (boost::format("Dropped %1% points at node %2%") % dropped_points_count % node.name).str());

    if (_progress_reporter)
      _progress_reporter->increment_progress(progress::INDEXING, dropped_points_count);
  }

  _persistence.persist_points(
    member_iterator(std::begin(all_points), &IndexedPoint64::point_reference),
    member_iterator(std::begin(all_points) + points_to_take, &IndexedPoint64::point_reference),
    node.bounds,
    node.name);

  if (_progress_reporter)
    _progress_reporter->increment_progress(progress::INDEXING,
                                           points_to_take - previously_taken_points_count);
}

/**
 * Tile the given node as an interior node, i.e. by using the given SamplingStrategy
 */
std::vector<NodeTilingData>
TilingAlgorithmV1::tile_internal_node(octree::NodeData& all_points,
                                      octree::NodeStructure const& node,
                                      octree::NodeStructure const& root_node,
                                      size_t previously_taken_points_count)
{
  const auto partition_point = filter_points_for_octree_node(std::begin(all_points),
                                                             std::end(all_points),
                                                             node.morton_index,
                                                             node.level,
                                                             root_node.bounds,
                                                             root_node.max_spacing,
                                                             _sampling_strategy);

  const auto points_taken =
    static_cast<size_t>(std::distance(std::begin(all_points), partition_point));

  _persistence.persist_points(
    member_iterator(std::begin(all_points), &IndexedPoint64::point_reference),
    member_iterator(partition_point, &IndexedPoint64::point_reference),
    node.bounds,
    node.name);

  if (_progress_reporter) {
    // To correctly increment progress, we have to know how many points were cached when we last hit
    // this node. In the 'worst' case, we take all the same points as last time, so that would mean
    // we made no progress on this node. Hence this calculation here:
    const auto newly_taken_points = points_taken - previously_taken_points_count;
    _progress_reporter->increment_progress(progress::INDEXING, newly_taken_points);
  }

  return split_range_into_child_nodes(partition_point, std::end(all_points), node, root_node);
}

std::vector<NodeTilingData>
TilingAlgorithmV1::tile_node(octree::NodeData&& node_data,
                             const octree::NodeStructure& node_structure,
                             const octree::NodeStructure& root_node_structure,
                             tf::Subflow& subflow)
{
  const auto current_node_name = concat(
    root_node_structure.name, to_string(node_structure.morton_index, node_structure.level + 1));

  auto cached_points = read_pnts_from_disk(current_node_name,
                                           root_node_structure.bounds,
                                           node_structure.bounds,
                                           _points_cache,
                                           _persistence);

  const auto cached_points_count = cached_points.size();

  const auto node_level_to_sample_from =
    octree::get_node_level_to_sample_from(node_structure.level, root_node_structure);

  // Check whether this node is an interior node, terminal node, or a node that is so deep that it
  // is a new root node
  if (node_level_to_sample_from >= static_cast<int32_t>(node_structure.max_depth)) {
    const auto all_points_for_this_node =
      octree::merge_node_data_unsorted(std::move(node_data), std::move(cached_points));
    tile_terminal_node(all_points_for_this_node, node_structure, cached_points_count);
    return {};
  }

  if (node_level_to_sample_from >= static_cast<int32_t>(MAX_OCTREE_LEVELS - 1)) {
    // If we are so deep that we exceed the capacity of the MortonIndex, we
    // have to index our points again with the current node as new root node. We
    // also have to carry the information that we have a new root over to the
    // children so that the paths of the nodes are correct.

    // Fun fact: We don't have to adjust the loaded indices because if we ever
    // get to a node this deep again, the indices have been calculated with the
    // new root the last time also, so everything is as it should be
    auto all_points_for_this_node =
      octree::merge_node_data_unsorted(std::move(node_data), std::move(cached_points));

    // Set this node as the new root node
    auto new_root_node = node_structure;
    new_root_node.max_depth = node_structure.max_depth - node_structure.level;

    // Compute new indices based upon this node as root node
    for (auto& indexed_point : all_points_for_this_node) {
      indexed_point.morton_index = calculate_morton_index<MAX_OCTREE_LEVELS>(
        indexed_point.point_reference.position(), new_root_node.bounds);
    }

    // Make sure everything is sorted again
    std::sort(all_points_for_this_node.begin(), all_points_for_this_node.end());

    return tile_internal_node(
      all_points_for_this_node, node_structure, new_root_node, cached_points_count);
  }

  auto all_points_for_this_node =
    octree::merge_node_data_sorted(std::move(node_data), std::move(cached_points));
  return tile_internal_node(
    all_points_for_this_node, node_structure, root_node_structure, cached_points_count);
}

/**
 * Perform tiling for the given node. This method does point selection for the node and
 * creates the execution graph for processing the child nodes of this node
 */
void
TilingAlgorithmV1::do_tiling_for_node(octree::NodeData&& node_data,
                                      const octree::NodeStructure& node_structure,
                                      const octree::NodeStructure& root_node_structure,
                                      tf::Subflow& subflow)
{
  auto child_nodes = tile_node(std::move(node_data), node_structure, root_node_structure, subflow);

  if (child_nodes.empty())
    return;

  // Create the execution graph for the child nodes. Only nodes with more than
  // MIN_POINTS_FOR_ASYNC_PROCESSING points are processed as asynchronous tasks, the other nodes are
  // processed synchronously right here. We sort the nodes descending by point count so that the
  // async tasks are created first, which means that they can start processing while this method
  // processes the synchronous nodes
  std::sort(std::begin(child_nodes),
            std::end(child_nodes),
            [](const NodeTilingData& l, const NodeTilingData& r) {
              return r.points.size() < l.points.size();
            });

  const auto iter_to_first_sync_node = std::partition_point(
    std::begin(child_nodes), std::end(child_nodes), [](const NodeTilingData& node) {
      return node.points.size() >= MIN_POINTS_FOR_ASYNC_PROCESSING;
    });

  // Create async tasks for tiling child nodes that have many points
  std::for_each(
    std::begin(child_nodes), iter_to_first_sync_node, [this, &subflow](NodeTilingData& child_node) {
      const auto child_node_name = child_node.node.name;
      const auto child_points_count = child_node.points.size();
      const auto child_task_name =
        (boost::format("%1% [%2%]") % child_node_name % child_points_count).str();
      subflow
        .emplace([this, _child_node = std::move(child_node)](tf::Subflow& sub_subflow) mutable {
          do_tiling_for_node(
            std::move(_child_node.points), _child_node.node, _child_node.root_node, sub_subflow);
        })
        .name(child_task_name);
    });

  // Do tiling for child nodes that have few points
  std::for_each(
    iter_to_first_sync_node, std::end(child_nodes), [this, &subflow](NodeTilingData& child_node) {
      do_tiling_for_node(
        std::move(child_node.points), child_node.node, child_node.root_node, subflow);
    });
}

#pragma endregion

TilingAlgorithmV2::TilingAlgorithmV2(SamplingStrategy& sampling_strategy,
                                     ProgressReporter* progress_reporter,
                                     PointsPersistence& persistence,
                                     TilerMetaParameters meta_parameters,
                                     size_t concurrency)
  : _sampling_strategy(sampling_strategy)
  , _progress_reporter(progress_reporter)
  , _persistence(persistence)
  , _meta_parameters(meta_parameters)
  , _concurrency(concurrency)
{}

void
TilingAlgorithmV2::build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf)
{}