#include "tiling/TilingAlgorithms.h"
#include "debug/ProgressReporter.h"

#include "containers/DestructuringIterator.h"
#include "terminal/stdout_helper.h"
#include "threading/Parallel.h"

#include <debug/Journal.h>

#include <mutex>
#include <set>

/**
 * The maximum depth of an octree that is representable with a single
 * MortonIndex
 */
constexpr static uint32_t MAX_OCTREE_LEVELS = 21;
/**
 * The minimum number of points in a node needed in order for that node to be
 * processed asynchronously
 */
constexpr static size_t MIN_POINTS_FOR_ASYNC_PROCESSING = 100'000;

#pragma region helper_functions
/**
 * Reads the cached points for the given node from disk and returns them as
 * IndexedPoints
 */
static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
read_pnts_from_disk(const octree::NodeStructure& node,
                    const AABB& octree_bounds,
                    PointsCache& points_cache,
                    PointsPersistence& persistence)
{
  PointBuffer tmp_points;
  persistence.retrieve_points(node.name, tmp_points);
  if (!tmp_points.count())
    return {};

  auto& points = points_cache.emplace_points(std::move(tmp_points));

  /**
   * In a previous version of the code, the MortonIndices were recomputed based on the bounds of the
   * whole octree. Since we have FP inaccuracies, some of our points might be outside of the current
   * node (node_bounds). Naively computing the MortonIndices might be wrong. So we clamped to the
   * bounds of the node to make sure that our points are always inside the node. But even in this
   * case the MortonIndex calculation might result in a wrong MortonIndex, because points on the
   * edge between two nodes can be in one node or the other, depending on the FP to int conversion
   * inside calculate_morton_index
   *
   * What we do instead is use the given MortonIndex of the node as a baseline, and only compute the
   * lower levels based on that index.
   */

  std::vector<IndexedPoint<MortonIndex64Levels>> indexed_points;
  indexed_points.reserve(points.count());

  std::transform(
    std::begin(points),
    std::end(points),
    std::back_inserter(indexed_points),
    [&](const auto& point_ref) -> IndexedPoint64 {
      auto idx = node.morton_index;
      auto morton_index_starting_from_this_node =
        calculate_morton_index<MAX_OCTREE_LEVELS>(point_ref.position(), node.bounds);

      const auto start_level = static_cast<uint32_t>(node.level + 1);
      for (auto level = start_level; level < MAX_OCTREE_LEVELS; ++level) {
        idx.set_octant_at_level(
          level, morton_index_starting_from_this_node.get_octant_at_level(level - start_level));
      }

      return { point_ref, idx };
    });

  // If the Persistence is lossy, we have to sort, as FP inaccuracies might disturb the order
  // of points
  if (!persistence.is_lossless()) {
    std::sort(std::begin(indexed_points), std::end(indexed_points));
  }

  return indexed_points;
}

/**
 * Takes a sorted range of IndexedPoints and splits it up into up to eight
 * ranges, one for each child node. This method then returns the appropriate
 * NodeTilingData for tiling each of the child nodes
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
    if (child_range.size() == 0)
      continue;

    auto child_node = node;
    child_node.morton_index.set_octant_at_level(child_level, octant);
    child_node.bounds = get_octant_bounds(octant, node.bounds);
    child_node.level = child_level;
    child_node.max_spacing /= 2;
    child_node.name = concat(node.name, static_cast<char>('0' + octant));

    // for (auto& point : child_range) {
    //   if (!child_node.bounds.isInside(point.point_reference.position())) {
    //     throw std::runtime_error{
    //       (boost::format(
    //          "Point %1% with MortonIndex %2% not correctly sorted into "
    //          "child octant %3% (%4%) of node %5%") %
    //        point.point_reference.position() % to_string(point.morton_index) %
    //        static_cast<int>(octant) % child_node.bounds % node.name)
    //         .str()
    //     };
    //   }
    // }

    // Copy the indices into a new vector and pass it on to the child processing
    child_tiling_data.emplace_back(
      octree::NodeData{ std::begin(child_range), std::end(child_range) }, child_node, root_node);
  }

  return child_tiling_data;
}

template<typename Tree, typename GetTotalPoints>
static void
dump_tree_to_graphviz(const Tree& tree, const std::string& path, GetTotalPoints get_total_points)
{
  const auto tree_graphviz = tree.to_graphviz([get_total_points](const auto& node) {
    const auto total_points = get_total_points(node);

    std::stringstream ss;
    ss << OctreeNodeIndex64::to_string(node.index()) << " - " << total_points;
    return ss.str();
  });

  {
    std::ofstream fs{ path };
    fs << tree_graphviz;
    fs.close();
  }
}

#pragma endregion

#pragma region TilingAlgorithmBase

TilingAlgorithmBase::TilingAlgorithmBase(SamplingStrategy& sampling_strategy,
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

TilingAlgorithmBase::~TilingAlgorithmBase() {}

/**
 * Tile the given node as a terminal node, i.e. take up to 'max_points_per_node'
 * points and persist them without any sampling
 */
void
TilingAlgorithmBase::tile_terminal_node(octree::NodeData const& all_points,
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
 * Tile the given node as an interior node, i.e. by using the given
 * SamplingStrategy
 */
std::vector<NodeTilingData>
TilingAlgorithmBase::tile_internal_node(octree::NodeData& all_points,
                                        octree::NodeStructure const& node,
                                        octree::NodeStructure const& root_node,
                                        size_t previously_taken_points_count)
{
  const auto partition_point =
    filter_points_for_octree_node(std::begin(all_points),
                                  std::end(all_points),
                                  node.morton_index,
                                  node.level,
                                  root_node.bounds,
                                  root_node.max_spacing,
                                  SamplingBehaviour::TakeAllWhenCountBelowMaxPoints,
                                  _sampling_strategy);

  const auto points_taken =
    static_cast<size_t>(std::distance(std::begin(all_points), partition_point));

  if (node.level >= 15) {
    const auto taken_percentage = points_taken / static_cast<double>(all_points.size());
    if (taken_percentage < 0.01) {
      util::write_log(concat("Discovered potentially broken node ", node.name));
      // Dump points to text file for debugging
      std::ofstream fs{ concat("./broken_", node.name + ".txt") };
      if (!fs.is_open())
        throw std::runtime_error{ "could not open dump file" };

      fs << "Bounds:       " << node.bounds << "\n";
      fs << "Points taken: " << points_taken << "\n";
      fs << "Total points: " << all_points.size() << "\n";
      fs << "\n";

      for (auto iter = std::begin(all_points); iter != std::end(all_points); ++iter) {
        const std::string tick_mark = (iter < partition_point) ? "[x]" : "[ ]";
        fs << tick_mark << " ";

        const auto& position = iter->point_reference.position();
        const auto& morton_idx = iter->morton_index;

        fs << position << " [" << to_string(morton_idx) << "]\n";
      }
    }
  }

  _persistence.persist_points(
    member_iterator(std::begin(all_points), &IndexedPoint64::point_reference),
    member_iterator(partition_point, &IndexedPoint64::point_reference),
    node.bounds,
    node.name);

  if (_progress_reporter) {
    // To correctly increment progress, we have to know how many points were
    // cached when we last hit this node. In the 'worst' case, we take all the
    // same points as last time, so that would mean we made no progress on this
    // node. Hence this calculation here:
    const auto newly_taken_points = points_taken - previously_taken_points_count;
    _progress_reporter->increment_progress(progress::INDEXING, newly_taken_points);
  }

  return split_range_into_child_nodes(partition_point, std::end(all_points), node, root_node);
}

std::vector<NodeTilingData>
TilingAlgorithmBase::tile_node(octree::NodeData&& node_data,
                               const octree::NodeStructure& node_structure,
                               const octree::NodeStructure& root_node_structure,
                               tf::Subflow& subflow)
{
  const auto current_node_name = concat(
    root_node_structure.name, to_string(node_structure.morton_index, node_structure.level + 1));

  auto cached_points =
    read_pnts_from_disk(node_structure, root_node_structure.bounds, _points_cache, _persistence);

  // const auto node_morton_index_str = node_structure.name.substr(1);
  // for (auto& point : cached_points) {
  //   if (!node_structure.bounds.isInside(point.point_reference.position())) {
  //     throw std::runtime_error{
  //       (boost::format("Cached point %1% is out of bounds @ node %2%") %
  //        point.point_reference.position() % node_structure.bounds)
  //         .str()
  //     };
  //   }
  //   const auto point_morton_index_str = to_string(point.morton_index);
  //   if (point_morton_index_str.rfind(node_morton_index_str, 0) != 0) {
  //     throw std::runtime_error{
  //       (boost::format("Cached point %1% has MortonIndex %2% but it does not
  //       "
  //                      "match the MortonIndex of the node: %3%") %
  //        point.point_reference.position() % point_morton_index_str %
  //        node_morton_index_str)
  //         .str()
  //     };
  //   }
  // }
  // for (auto& point : node_data) {
  //   if (!node_structure.bounds.isInside(point.point_reference.position())) {
  //     throw std::runtime_error{
  //       (boost::format("Current point %1% is out of bounds @ node %2%") %
  //        point.point_reference.position() % node_structure.bounds)
  //         .str()
  //     };
  //   }
  //   const auto point_morton_index_str = to_string(point.morton_index);
  //   if (point_morton_index_str.rfind(node_morton_index_str, 0) != 0) {
  //     throw std::runtime_error{
  //       (boost::format("Current point %1% has MortonIndex %2% but it does not
  //       "
  //                      "match the MortonIndex of the node: %3%") %
  //        point.point_reference.position() % point_morton_index_str %
  //        node_morton_index_str)
  //         .str()
  //     };
  //   }
  // }

  const auto cached_points_count = cached_points.size();

  const auto node_level_to_sample_from =
    octree::get_node_level_to_sample_from(node_structure.level, root_node_structure);

  // Check whether this node is an interior node, terminal node, or a node that
  // is so deep that it is a new root node
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
 * Perform tiling for the given node. This method does point selection for the
 * node and creates the execution graph for processing the child nodes of this
 * node
 */
void
TilingAlgorithmBase::do_tiling_for_node(octree::NodeData&& node_data,
                                        const octree::NodeStructure& node_structure,
                                        const octree::NodeStructure& root_node_structure,
                                        tf::Subflow& subflow)
{
  auto child_nodes = tile_node(std::move(node_data), node_structure, root_node_structure, subflow);

  if (child_nodes.empty())
    return;

  // Create the execution graph for the child nodes. Only nodes with more than
  // MIN_POINTS_FOR_ASYNC_PROCESSING points are processed as asynchronous tasks,
  // the other nodes are processed synchronously right here. We sort the nodes
  // descending by point count so that the async tasks are created first, which
  // means that they can start processing while this method processes the
  // synchronous nodes
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

#pragma region TilingAlgorithmV1

TilingAlgorithmV1::TilingAlgorithmV1(SamplingStrategy& sampling_strategy,
                                     ProgressReporter* progress_reporter,
                                     PointsPersistence& persistence,
                                     TilerMetaParameters meta_parameters,
                                     size_t concurrency)
  : TilingAlgorithmBase(sampling_strategy,
                        progress_reporter,
                        persistence,
                        meta_parameters,
                        concurrency)
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

#pragma endregion

#pragma region TilingAlgorithmV2

TilingAlgorithmV2::TilingAlgorithmV2(SamplingStrategy& sampling_strategy,
                                     ProgressReporter* progress_reporter,
                                     PointsPersistence& persistence,
                                     TilerMetaParameters meta_parameters,
                                     size_t concurrency)
  : TilingAlgorithmBase(sampling_strategy,
                        progress_reporter,
                        persistence,
                        meta_parameters,
                        concurrency)
{}

void
TilingAlgorithmV2::build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf)
{
  /**
   * #### Revised algorithm for better concurrency ####
   *
   * Instead of starting from the root node (which has to be processed by a
   * single thread), select a number of deeper nodes that can all be processed
   * in parallel. This will skip some nodes, their content has to be
   * reconstructed from the deeper nodes at the end, either by subsampling or
   * averaging
   */

  _root_node_points.clear();
  _root_node_points.resize(points.count());
  _points_cache.clear();
  _indexed_points_ranges.clear();
  _indexed_points_ranges.resize(_concurrency);

  const auto chunk_size = _root_node_points.size() / _concurrency;

  auto scatter_task = parallel::scatter(
    std::begin(points),
    std::end(points),
    [this, chunk_size, bounds](PointsIter points_begin, PointsIter points_end, size_t task_index) {
      // A bit of logic to access the slots of pre-allocated memory for this
      // task
      const auto point_data_offset = (task_index * chunk_size);
      const auto indexed_points_begin = std::begin(_root_node_points) + point_data_offset;
      const auto indexed_points_end =
        indexed_points_begin + std::distance(points_begin, points_end);
      auto& task_output = _indexed_points_ranges[task_index];

      index_and_sort_points(
        { points_begin, points_end }, { indexed_points_begin, indexed_points_end }, bounds);
      task_output = split_indexed_points_into_subranges(
        { indexed_points_begin, indexed_points_end }, _concurrency);
    },
    tf,
    _concurrency);

  auto transpose_task = tf.emplace([this, bounds](tf::Subflow& subflow) {
    auto ranges_per_node = merge_selected_start_nodes(_indexed_points_ranges, _concurrency);

    // ranges_per_node is an Octree where some of the nodes contain the starting
    // data for tiling

    std::unordered_set<OctreeNodeIndex64> left_out_nodes;
    std::vector<tf::Task> new_tiling_tasks;

    for (auto node : ranges_per_node.traverse_level_order()) {
      if (node->empty())
        continue;

      auto parent = node.index().parent();
      while (true) {
        // Insert parent node and its parents etc. to left_out_nodes
        left_out_nodes.insert(parent);
        if (parent.levels() == 0)
          break;
        parent = parent.parent();
      }

      new_tiling_tasks.push_back(subflow.emplace(
        [this, bounds, index = node.index(), _data = std::move(*node)](tf::Subflow& subsubflow) {
          auto process_data = prepare_range_for_tiling(_data, index, bounds);

          do_tiling_for_node(
            std::move(process_data.points), process_data.node, process_data.root_node, subsubflow);
        }));
    }

    const auto reconstruct_task =
      subflow.emplace([this, bounds, _left_out_nodes = std::move(left_out_nodes)]() {
        reconstruct_left_out_nodes(_left_out_nodes, bounds);
      });

    for (auto tiling_task : new_tiling_tasks) {
      tiling_task.precede(reconstruct_task);
    }
  });

  for (auto& scatter_subtask : scatter_task.scattered_tasks) {
    scatter_subtask.precede(transpose_task);
  }
}

void
TilingAlgorithmV2::index_and_sort_points(util::Range<PointsIter> points,
                                         util::Range<IndexedPointsIter> indexed_points,
                                         const AABB& bounds) const
{
  assert(points.size() == indexed_points.size());

  points.transform(std::begin(indexed_points),
                   [&bounds](PointBuffer::PointReference point_ref) -> IndexedPoint64 {
                     return index_point<MAX_OCTREE_LEVELS>(
                       point_ref, bounds, OutlierPointsBehaviour::ClampToBounds);
                   });

  indexed_points.sort();
}

/**
 * This method takes a range of IndexedPoints that span the root node of the
 * dataset and break it up into approximately* 'min_number_of_ranges' connected
 * ranges, where each of the new ranges spans exactly one node in the octree.
 * Ranges are split by size, with the largest range being split first. When
 * split, the range is replaced by ALL ranges for its immediate child nodes (up
 * to 8 ranges).
 *
 * To understand this, here is a simplified case in 2D, with node indices
 * written as 'ABC' where A is the nodes index at level 0, B at level 1 etc.
 *
 * Input range: [000, 002, 003, 011, 013, 030, 102, 103, 120, 300, 310, 313]
 * Desired ranges: 4
 *
 * Output: [[000, 002, 003], [011, 013], [030], [102, 103, 120], [300, 310,
 * 313]]
 *
 * This result is achieved in two steps:
 *
 * 1) Split up the root range. Yields:
 * [[000, 002, 003, 011, 013, 030], [102, 103, 120], [300, 310, 313]]
 *  |----------------------------|  |-------------|  |-------------|
 *      all points for node 0          ...node 1        ...node 3
 *
 * 2) Current range count is 3, which is less than desired ranges. Split up the
 * largest range, which is the range for node 0 (contains 6 points). This yields
 * the final result:
 * [[000, 002, 003], [011, 013], [030], [102, 103, 120], [300, 310, 313]]
 *  |-------------|  |--------|  |---|  |-------------|  |-------------|
 *      node 00        node 01  node 03     node 1            node 3
 */
Octree<util::Range<TilingAlgorithmV2::IndexedPointsIter>>
TilingAlgorithmV2::split_indexed_points_into_subranges(
  util::Range<IndexedPointsIter> indexed_points,
  size_t min_number_of_ranges) const
{
  Octree<util::Range<IndexedPointsIter>> point_ranges{ { {}, indexed_points } };

  if (indexed_points.size() <= min_number_of_ranges) {
    return point_ranges;
  }

  // Recursively split the node with the largest range into up to 8 child nodes

  auto get_node_with_most_points = [](Octree<util::Range<IndexedPointsIter>>& octree) {
    const auto iter_to_max_range =
      std::max_element(std::begin(octree.traverse_level_order()),
                       std::end(octree.traverse_level_order()),
                       [](const auto& left_node, const auto& right_node) {
                         return left_node->size() < right_node->size();
                       });
    if (iter_to_max_range == std::end(octree.traverse_level_order()))
      return iter_to_max_range;

    // To ensure that this whole method always terminates, we have to make
    // SURE that we only return a range from 'get_largest_range' if this
    // range can be split up into its child nodes. This can be done with a
    // simple check for the Morton index difference between the first and
    // last points in the range
    const auto& first_point = iter_to_max_range->first();
    const auto& last_point = iter_to_max_range->last();
    if (first_point.morton_index == last_point.morton_index)
      return std::end(octree.traverse_level_order());

    return iter_to_max_range;
  };

  size_t num_non_empty_nodes = 1;

  // // As long as we have less than 'min_number_of_ranges', keep splitting up
  // the largest range
  while (num_non_empty_nodes < min_number_of_ranges) {
    auto iter_to_max_range = get_node_with_most_points(point_ranges);
    if (iter_to_max_range == std::end(point_ranges.traverse_level_order()))
      break;

    // Split node up into child ranges
    auto node_with_max_range = *iter_to_max_range;
    const auto child_level = node_with_max_range.index().levels();
    const auto child_ranges = partition_points_into_child_octants(
      node_with_max_range->begin(), node_with_max_range->end(), child_level);

    // Insert each child range into the octree
    for (uint8_t octant = 0; octant < 8; ++octant) {
      const auto& child_range = child_ranges[octant];
      if (child_range.size() == 0)
        continue;

      point_ranges.insert(node_with_max_range.index().child(octant), child_range);

      ++num_non_empty_nodes;
    }

    // Clear the range of this node
    point_ranges.insert(node_with_max_range.index(), {});
    --num_non_empty_nodes;
  }

  size_t points_in_octree = 0;
  for (auto node : point_ranges.traverse_level_order()) {
    points_in_octree += node->size();
  }

  assert(points_in_octree == indexed_points.size());

  return point_ranges;
}

Octree<std::vector<util::Range<TilingAlgorithmV2::IndexedPointsIter>>>
TilingAlgorithmV2::merge_selected_start_nodes(
  const std::vector<Octree<util::Range<IndexedPointsIter>>>& selected_nodes,
  size_t min_number_of_ranges)
{
  using Octree_t = Octree<std::vector<util::Range<TilingAlgorithmV2::IndexedPointsIter>>>;
  // Merge all trees into one tree

  auto merged_tree = std::accumulate(
    std::begin(selected_nodes),
    std::end(selected_nodes),
    Octree_t{},
    [](const auto& accum, const auto& val) {
      return Octree_t::transform_merge<util::Range<IndexedPointsIter>>(
        accum,
        val,
        [](util::Range<IndexedPointsIter> range) -> std::vector<util::Range<IndexedPointsIter>> {
          if (range.size() == 0)
            return {};
          return { range };
        },
        [](const std::vector<util::Range<IndexedPointsIter>>& l,
           const std::vector<util::Range<IndexedPointsIter>>& r) {
          std::vector<util::Range<IndexedPointsIter>> merged_ranges;
          for (auto& range : l) {
            if (range.size() == 0)
              continue;
            merged_ranges.push_back(range);
          }
          for (auto& range : r) {
            if (range.size() == 0)
              continue;
            merged_ranges.push_back(range);
          }
          return merged_ranges;
        });
    });

  // This tree can now have nodes that are in a parent/child relationship that
  // both have a non-empty range assigned to them. To break this pattern, we
  // first split the ranges in the parent nodes and push them to the child
  // nodes. Then, we keep merging child nodes until we have an acceptable number
  // of nodes

  std::vector<Octree_t::MutableNode> interior_nodes;
  for (auto node : merged_tree.traverse_level_order()) {
    // Look for nodes that have at least one non-empty range
    if (node->empty())
      continue;
    if (std::all_of(std::begin(*node), std::end(*node), [](const auto& range) {
          return range.size() == 0;
        })) {
      continue;
    }

    // If the node is a leaf node, we skip it
    const auto children = node.children();
    if (children.empty())
      continue;

    // For each range, split the range and distribute all splitted ranges to the
    // appropriate child nodes
    for (auto& range : *node) {
      if (range.size() == 0) {
        __builtin_trap();
      }
      const auto split_ranges = partition_points_into_child_octants(
        std::begin(range), std::end(range), node.index().levels());
      for (uint8_t octant = 0; octant < 8; ++octant) {
        const auto& subrange = split_ranges[octant];
        if (subrange.size() == 0) {
          continue;
        }

        const auto child_node_index = node.index().child(octant);
        merged_tree.at(child_node_index)->push_back(subrange);
      }
    }

    // Clear this node (but keep it in the tree)
    merged_tree.at(node.index())->clear();
  }

  // Keep merging leaf-nodes until we have ~min_number_of_ranges leaf nodes
  std::unordered_map<OctreeNodeIndex64, Octree_t::MutableNode> penultimate_nodes;
  const auto is_penultimate_node = [](const auto& node) {
    const auto children = node.children();
    return !node.is_leaf() && std::all_of(std::begin(children),
                                          std::end(children),
                                          [](const auto& node) { return node.is_leaf(); });
  };

  // Look for all nodes that have only leaf nodes as children
  for (auto node : merged_tree.traverse_level_order()) {
    if (!is_penultimate_node(node))
      continue;
    penultimate_nodes[node.index()] = node;
  }

  auto num_start_nodes = static_cast<size_t>(
    std::count_if(std::begin(merged_tree.traverse_level_order()),
                  std::end(merged_tree.traverse_level_order()),
                  [](const auto& node) { return node.is_leaf() && (!node->empty()); }));

  // Counts the total number of points in all direct child nodes of 'node'
  const auto num_points_in_children = [](const Octree_t::MutableNode& node) {
    const auto children = node.children();
    return std::accumulate(
      std::begin(children), std::end(children), size_t{ 0 }, [](size_t accum, const auto& node) {
        return accum + std::accumulate(
                         std::begin(*node),
                         std::end(*node),
                         size_t{ 0 },
                         [](size_t accum, const auto& range) { return accum + range.size(); });
      });
  };

  const auto merge_leaves =
    [is_penultimate_node](
      const Octree_t::MutableNode& leaf_parent,
      std::unordered_map<OctreeNodeIndex64, Octree_t::MutableNode>& penultimate_nodes) -> size_t {
    size_t merged_nodes = 0;
    std::vector<util::Range<TilingAlgorithmV2::IndexedPointsIter>> merged_ranges;
    for (auto leaf : leaf_parent.children()) {
      if (leaf->empty())
        continue;

      merged_ranges.insert(std::end(merged_ranges), std::begin(*leaf), std::end(*leaf));
      ++merged_nodes;
    }

    leaf_parent.erase();                     // Clears all child nodes that we just merged
    *leaf_parent = std::move(merged_ranges); // Puts all the merged ranges into this node

    // 'leaf_parent' is no longer a penultimate node, but its parent might be
    // now!
    penultimate_nodes.erase(leaf_parent.index());

    if (leaf_parent.index() == OctreeNodeIndex64{})
      return merged_nodes;
    const auto parent = leaf_parent.parent();
    if (!is_penultimate_node(parent))
      return merged_nodes;

    penultimate_nodes[parent.index()] = parent;
    --merged_nodes;
    return merged_nodes;
  };

  // Finds the best node to merge, which is the node with the least number of
  // points in its direct children
  const auto find_best_node_to_merge = [num_points_in_children](const auto& penultimate_nodes) {
    return std::min_element(std::begin(penultimate_nodes),
                            std::end(penultimate_nodes),
                            [num_points_in_children](const auto& l, const auto& r) {
                              return num_points_in_children(l.second) <
                                     num_points_in_children(r.second);
                            });
  };

  // uint32_t steps = 0;

  while (num_start_nodes > min_number_of_ranges) {
    const auto node_to_merge_iter = find_best_node_to_merge(penultimate_nodes);
    if (node_to_merge_iter == std::end(penultimate_nodes))
      break;

    const auto children_of_node_to_merge = node_to_merge_iter->second.children();
    auto num_nodes_to_merge = std::count_if(std::begin(children_of_node_to_merge),
                                            std::end(children_of_node_to_merge),
                                            [](const auto& node) { return !node->empty(); });

    // Don't merge anymore if we drop below the minimum number of ranges
    if ((num_start_nodes - num_nodes_to_merge) < min_number_of_ranges) {
      break;
    }

    // Merge the leaves
    num_start_nodes -= merge_leaves(node_to_merge_iter->second, penultimate_nodes);
  }

  return merged_tree;
}

NodeTilingData
TilingAlgorithmV2::prepare_range_for_tiling(
  const std::vector<util::Range<IndexedPointsIter>>& start_node_data,
  OctreeNodeIndex64 node_index,
  const AABB& bounds)
{
  const auto start_node_point_count =
    std::accumulate(std::begin(start_node_data),
                    std::end(start_node_data),
                    size_t{ 0 },
                    [](size_t accum, const auto& range) { return accum + range.size(); });

  octree::NodeData merged_data{ start_node_point_count };

  merge_ranges(util::range(start_node_data),
               util::range(merged_data),
               [](const IndexedPoint64& l, const IndexedPoint64& r) {
                 return l.morton_index.get() < r.morton_index.get();
               });

  octree::NodeStructure root_node;
  root_node.bounds = bounds;
  root_node.level = -1;
  root_node.max_depth = _meta_parameters.max_depth;
  root_node.max_spacing = _meta_parameters.spacing_at_root;
  root_node.morton_index = {};
  root_node.name = "r";

  octree::NodeStructure this_node;
  this_node.bounds = get_bounds_from_node_index(node_index, bounds);
  this_node.level = static_cast<int32_t>(node_index.levels()) - 1;
  this_node.max_depth = root_node.max_depth;
  this_node.max_spacing = root_node.max_spacing / std::pow(2, node_index.levels());
  this_node.morton_index = node_index.to_static_morton_index();
  this_node.name = std::string{ "r" } + OctreeNodeIndex64::to_string(node_index);

  return { std::move(merged_data), this_node, root_node };
}

void
TilingAlgorithmV2::reconstruct_single_node(const OctreeNodeIndex64& node_index,
                                           const AABB& root_bounds)
{

  // TODO If this node already exists (from a previous iteration), we will lose
  // points!
  // const auto this_node_name = concat("r",
  // OctreeNodeIndex64::to_string(node_index));

  // 1) Read data of direct child nodes
  PointBuffer data;
  for (uint8_t octant = 0; octant < 8; ++octant) {
    const auto child_index = node_index.child(octant);
    const auto node_name = concat("r", OctreeNodeIndex64::to_string(child_index));

    PointBuffer tmp;
    _persistence.retrieve_points(node_name, tmp);

    if (tmp.empty())
      continue;

    data.append_buffer(tmp);
  }

  // 2) Calculate morton indices for child data
  std::vector<IndexedPoint64> indexed_points;
  indexed_points.reserve(data.count());
  index_points<MAX_OCTREE_LEVELS>(std::begin(data),
                                  std::end(data),
                                  std::back_inserter(indexed_points),
                                  root_bounds,
                                  OutlierPointsBehaviour::ClampToBounds);

  // 3) Data is sorted, so we can sample directly
  const auto morton_index_for_node = node_index.to_static_morton_index();
  const auto selected_points_end = sample_points(_sampling_strategy,
                                                 std::begin(indexed_points),
                                                 std::end(indexed_points),
                                                 morton_index_for_node,
                                                 static_cast<int32_t>(node_index.levels()) - 1,
                                                 root_bounds,
                                                 _meta_parameters.spacing_at_root,
                                                 SamplingBehaviour::AlwaysAdhereToMinSpacing);

  // 3) Write to disk
  const auto node_bounds = get_bounds_from_node_index(node_index, root_bounds);
  const auto node_name = concat("r", OctreeNodeIndex64::to_string(node_index));

  // TOOD For 3D Tiles, reconstructed nodes should have their children be
  // 'REPLACE' instead of 'ADD'

  _persistence.persist_points(
    member_iterator(std::begin(indexed_points), &IndexedPoint64::point_reference),
    member_iterator(selected_points_end, &IndexedPoint64::point_reference),
    node_bounds,
    node_name);
}

void
TilingAlgorithmV2::reconstruct_left_out_nodes(
  const std::unordered_set<OctreeNodeIndex64>& left_out_nodes,
  const AABB& root_bounds)
{
  std::list<OctreeNodeIndex64> nodes_by_level{ std::begin(left_out_nodes),
                                               std::end(left_out_nodes) };
  nodes_by_level.sort(
    [](const OctreeNodeIndex64& l, const OctreeNodeIndex64& r) { return l.levels() > r.levels(); });

  for (auto& node_index : nodes_by_level) {
    reconstruct_single_node(node_index, root_bounds);
  }
}

#pragma endregion

#pragma region TilingAlgorithmV3
TilingAlgorithmV3::TilingAlgorithmV3(SamplingStrategy& sampling_strategy,
                                     ProgressReporter* progress_reporter,
                                     PointsPersistence& persistence,
                                     TilerMetaParameters meta_parameters,
                                     const fs::path& output_dir,
                                     size_t concurrency)
  : TilingAlgorithmBase(sampling_strategy,
                        progress_reporter,
                        persistence,
                        meta_parameters,
                        concurrency)
  , _output_dir(output_dir)
{}

void
TilingAlgorithmV3::build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf)
{
  /**
   * #### Revised algorithm for better concurrency ####
   *
   * Instead of starting from the root node (which has to be processed by a
   * single thread), select a number of deeper nodes that can all be processed
   * in parallel. This will skip some nodes, their content has to be
   * reconstructed from the deeper nodes at the end, either by subsampling or
   * averaging
   */

  _root_node_points.clear();
  _root_node_points.resize(points.count());
  _points_cache.clear();
  _indexed_points_ranges.clear();
  _indexed_points_ranges.resize(_concurrency);

  if (!_level_of_start_nodes.has_value()) {
    build_execution_graph_for_first_iteration(points, bounds, tf);
  } else {
    build_execution_graph_for_later_iterations(points, bounds, tf);
  }
}

void
TilingAlgorithmV3::finalize(const AABB& bounds)
{
  if (!_level_of_start_nodes.has_value()) {
    // build_execution_graph_for_first_iteration was never run, i.e. we never
    // processed any points
    return;
  }
  reconstruct_left_out_nodes(bounds);
}

void
TilingAlgorithmV3::build_execution_graph_for_first_iteration(PointBuffer& points,
                                                             const AABB& bounds,
                                                             tf::Taskflow& tf)
{
  // After indexing the points, we sort them all together, estimate the start
  // node level and then generate the start nodes

  const auto chunk_size = _root_node_points.size() / _concurrency;

  auto index_task = parallel::scatter(
    std::begin(points),
    std::end(points),
    [this, chunk_size, bounds](PointsIter points_begin, PointsIter points_end, size_t task_index) {
      // A bit of logic to access the slots of pre-allocated memory for this
      // task
      const auto point_data_offset = (task_index * chunk_size);
      const auto indexed_points_begin = std::begin(_root_node_points) + point_data_offset;

      util::Range<PointBuffer::PointIterator> points{ points_begin, points_end };

      points.transform(indexed_points_begin,
                       [&bounds](PointBuffer::PointReference point_ref) -> IndexedPoint64 {
                         return index_point<MAX_OCTREE_LEVELS>(
                           point_ref, bounds, OutlierPointsBehaviour::ClampToBounds);
                       });
    },
    tf,
    _concurrency,
    "index_points");

  auto sort_estimate_get_start_node =
    tf.emplace([this, bounds](tf::Subflow& subflow) {
        util::Range<IndexedPointsIter> indexed_points{ std::begin(_root_node_points),
                                                       std::end(_root_node_points) };
        indexed_points.sort();

        _level_of_start_nodes = estimate_start_node_level_in_octree(indexed_points, _concurrency);

        util::write_log(concat("Level of start nodes: ", *_level_of_start_nodes, "\n"));
        if (debug::Journal::instance().is_enabled()) {
          debug::Journal::instance().add_entry(
            concat("Level of start nodes: ", *_level_of_start_nodes, "\n"));
        }

        auto start_nodes =
          split_indexed_points_into_subranges(indexed_points, *_level_of_start_nodes);

        if (debug::Journal::instance().is_enabled()) {
          debug::Journal::instance().add_entry(
            concat("start nodes 0:\n", start_nodes.to_graphviz([](const auto& node) {
              std::stringstream ss;
              ss << OctreeNodeIndex64::to_string(node.index()) << " - " << node->size();
              return ss.str();
            })));
        }

        for (auto node : start_nodes.traverse_level_order()) {
          if (node->size() == 0)
            continue;

          const auto child_task_name = (boost::format("r%1% [%2%]") %
                                        OctreeNodeIndex64::to_string(node.index()) % node->size())
                                         .str();

          subflow
            .emplace([this, bounds, index = node.index(), _data = std::move(*node)](
                       tf::Subflow& subsubflow) {
              octree::NodeStructure root_node;
              root_node.bounds = bounds;
              root_node.level = -1;
              root_node.max_depth = _meta_parameters.max_depth;
              root_node.max_spacing = _meta_parameters.spacing_at_root;
              root_node.morton_index = {};
              root_node.name = "r";

              octree::NodeStructure this_node;
              this_node.bounds = get_bounds_from_node_index(index, bounds);
              this_node.level = static_cast<int32_t>(index.levels()) - 1;
              this_node.max_depth = root_node.max_depth;
              this_node.max_spacing = root_node.max_spacing / std::pow(2, index.levels());
              this_node.morton_index = index.to_static_morton_index();
              this_node.name = std::string{ "r" } + OctreeNodeIndex64::to_string(index);

              do_tiling_for_node(
                { std::begin(_data), std::end(_data) }, this_node, root_node, subsubflow);
            })
            .name(child_task_name);
        }
      })
      .name("sort_and_get_start_nodes");

  for (auto& scatter_subtask : index_task.scattered_tasks) {
    scatter_subtask.precede(sort_estimate_get_start_node);
  }
}

void
TilingAlgorithmV3::build_execution_graph_for_later_iterations(PointBuffer& points,
                                                              const AABB& bounds,
                                                              tf::Taskflow& tf)
{

  static uint32_t s_counter = 0;

  const auto chunk_size = _root_node_points.size() / _concurrency;

  auto scatter_task = parallel::scatter(
    std::begin(points),
    std::end(points),
    [this, chunk_size, bounds](PointsIter points_begin, PointsIter points_end, size_t task_index) {
      // A bit of logic to access the slots of pre-allocated memory for this
      // task
      const auto point_data_offset = (task_index * chunk_size);
      const auto indexed_points_begin = std::begin(_root_node_points) + point_data_offset;
      const auto indexed_points_end =
        indexed_points_begin + std::distance(points_begin, points_end);
      auto& task_output = _indexed_points_ranges[task_index];

      index_and_sort_points(
        { points_begin, points_end }, { indexed_points_begin, indexed_points_end }, bounds);
      task_output = split_indexed_points_into_subranges(
        { indexed_points_begin, indexed_points_end }, *_level_of_start_nodes);
    },
    tf,
    _concurrency,
    "index_then_sort_then_split_ranges");

  auto transpose_task =
    tf.emplace([this, bounds](tf::Subflow& subflow) {
        auto ranges_per_node = merge_selected_start_nodes(_indexed_points_ranges);

        if (debug::Journal::instance().is_enabled()) {
          debug::Journal::instance().add_entry(concat(
            "start nodes ", ++s_counter, ":\n", ranges_per_node.to_graphviz([](const auto& node) {
              std::stringstream ss;
              ss << OctreeNodeIndex64::to_string(node.index()) << " - "
                 << std::accumulate(
                      std::begin(*node),
                      std::end(*node),
                      size_t{ 0 },
                      [](auto accum, const auto& range) { return accum + range.size(); });
              return ss.str();
            })));
        }

        // ranges_per_node is an Octree where some of the nodes contain the
        // starting data for tiling

        for (auto node : ranges_per_node.traverse_level_order()) {
          if (node->empty())
            continue;

          const auto num_points = std::accumulate(
            std::begin(*node), std::end(*node), size_t{ 0 }, [](auto accum, const auto& range) {
              return accum + range.size();
            });
          const auto child_task_name =
            (boost::format("r%1% [%2%]") % OctreeNodeIndex64::to_string(node.index()) % num_points)
              .str();

          subflow
            .emplace([this, bounds, index = node.index(), _data = std::move(*node)](
                       tf::Subflow& subsubflow) {
              auto process_data = prepare_range_for_tiling(_data, index, bounds);

              do_tiling_for_node(std::move(process_data.points),
                                 process_data.node,
                                 process_data.root_node,
                                 subsubflow);
            })
            .name(child_task_name);
        }
      })
      .name("merge_ranges_for_start_nodes");

  for (auto& scatter_subtask : scatter_task.scattered_tasks) {
    scatter_subtask.precede(transpose_task);
  }
}

void
TilingAlgorithmV3::index_and_sort_points(util::Range<PointsIter> points,
                                         util::Range<IndexedPointsIter> indexed_points,
                                         const AABB& bounds) const
{
  assert(points.size() == indexed_points.size());

  points.transform(std::begin(indexed_points),
                   [&bounds](PointBuffer::PointReference point_ref) -> IndexedPoint64 {
                     return index_point<MAX_OCTREE_LEVELS>(
                       point_ref, bounds, OutlierPointsBehaviour::ClampToBounds);
                   });

  indexed_points.sort();
}

size_t
TilingAlgorithmV3::estimate_start_node_level_in_octree(
  util::Range<IndexedPointsIter> indexed_points,
  size_t concurrency) const
{
  // Split at level 1, then level 2, level 3 etc.
  // For each split level, calculate a score based on the number of ranges that
  // resulted and the distribution of points in these ranges. This should weight
  // an even distribution of points higher to prevent cases where there are
  // 'concurrency' ranges, but all but one range have only a handfull of points

  std::vector<util::Range<IndexedPointsIter>> current_splits{ indexed_points };

  const auto is_large_range = [](const util::Range<IndexedPointsIter>& range) -> bool {
    return range.size() >= 100'000;
  };

  const auto score_ranges = [concurrency, is_large_range](
                              const std::vector<util::Range<IndexedPointsIter>>& ranges) -> float {
    if (ranges.size() <= concurrency / 2)
      return 0.f;

    const auto num_large_ranges =
      std::count_if(std::begin(ranges), std::end(ranges), is_large_range);

    return static_cast<float>(num_large_ranges) / static_cast<float>(concurrency);
  };

  const auto split_at_level = [](uint32_t level,
                                 const std::vector<util::Range<IndexedPointsIter>>& ranges)
    -> std::vector<util::Range<IndexedPointsIter>> {
    std::vector<util::Range<IndexedPointsIter>> new_ranges;
    for (const auto& range : ranges) {
      const auto split_ranges =
        partition_points_into_child_octants(std::begin(range), std::end(range), level);

      std::copy_if(std::begin(split_ranges),
                   std::end(split_ranges),
                   std::back_inserter(new_ranges),
                   [](const auto& range) { return range.size() > 0; });
    }
    return new_ranges;
  };

  constexpr uint32_t MIN_LEVEL = 3;
  constexpr uint32_t MAX_LEVEL = 6;
  constexpr float MIN_SCORE = 1.f;

  for (uint32_t level = 0; level < MAX_LEVEL; ++level) {
    current_splits = split_at_level(level, current_splits);

    const auto current_score = score_ranges(current_splits);
    if (current_score >= MIN_SCORE) {
      return std::max(level + 1, MIN_LEVEL);
    }
  }

  return MAX_LEVEL;
}

Octree<util::Range<TilingAlgorithmV3::IndexedPointsIter>>
TilingAlgorithmV3::split_indexed_points_into_subranges(
  util::Range<IndexedPointsIter> indexed_points,
  size_t level_of_subranges_in_octree) const
{
  using IndexedPointOctree = Octree<util::Range<IndexedPointsIter>>;
  IndexedPointOctree point_ranges{ { {}, indexed_points } };

  std::queue<IndexedPointOctree::MutableNode> next_nodes;
  next_nodes.push(point_ranges.at({}));

  while (!next_nodes.empty()) {
    // Take node and split it into child ranges. Every child node that is <
    // level_of_subranges_in_octree is inserted into the queue again

    const auto node = next_nodes.front();
    next_nodes.pop();

    const auto node_index = node.index();
    const auto child_ranges =
      partition_points_into_child_octants(std::begin(*node), std::end(*node), node_index.levels());
    for (uint8_t octant = 0; octant < 8; ++octant) {
      const auto& child_range = child_ranges[octant];
      if (child_range.size() == 0) {
        continue;
      }

      const auto child_index = node_index.child(octant);
      point_ranges.insert(child_index, child_range);

      if (child_index.levels() == level_of_subranges_in_octree)
        continue;

      next_nodes.push(point_ranges.at(child_index));
    }

    // Clear range of this node
    *node = {};
  }

  return point_ranges;
}

Octree<std::vector<util::Range<TilingAlgorithmV3::IndexedPointsIter>>>
TilingAlgorithmV3::merge_selected_start_nodes(
  const std::vector<Octree<util::Range<IndexedPointsIter>>>& selected_nodes)
{
  // Since all Octrees have nodes with non-empty ranges at the same level, this
  // is done by merging all trees with each other without any additional checks
  using Octree_t = Octree<std::vector<util::Range<IndexedPointsIter>>>;

  return std::accumulate(
    std::begin(selected_nodes),
    std::end(selected_nodes),
    Octree_t{},
    [](const auto& accum, const auto& val) {
      return Octree_t::transform_merge<util::Range<IndexedPointsIter>>(
        accum,
        val,
        [](util::Range<IndexedPointsIter> range) -> std::vector<util::Range<IndexedPointsIter>> {
          if (range.size() == 0)
            return {};
          return { range };
        },
        [](const std::vector<util::Range<IndexedPointsIter>>& l,
           const std::vector<util::Range<IndexedPointsIter>>& r) {
          std::vector<util::Range<IndexedPointsIter>> merged_ranges;
          for (auto& range : l) {
            if (range.size() == 0)
              continue;
            merged_ranges.push_back(range);
          }
          for (auto& range : r) {
            if (range.size() == 0)
              continue;
            merged_ranges.push_back(range);
          }
          return merged_ranges;
        });
    });
}

NodeTilingData
TilingAlgorithmV3::prepare_range_for_tiling(
  const std::vector<util::Range<IndexedPointsIter>>& start_node_data,
  OctreeNodeIndex64 node_index,
  const AABB& bounds)
{
  const auto start_node_point_count =
    std::accumulate(std::begin(start_node_data),
                    std::end(start_node_data),
                    size_t{ 0 },
                    [](size_t accum, const auto& range) { return accum + range.size(); });

  octree::NodeData merged_data{ start_node_point_count };

  merge_ranges(util::range(start_node_data),
               util::range(merged_data),
               [](const IndexedPoint64& l, const IndexedPoint64& r) {
                 return l.morton_index.get() < r.morton_index.get();
               });

  octree::NodeStructure root_node;
  root_node.bounds = bounds;
  root_node.level = -1;
  root_node.max_depth = _meta_parameters.max_depth;
  root_node.max_spacing = _meta_parameters.spacing_at_root;
  root_node.morton_index = {};
  root_node.name = "r";

  octree::NodeStructure this_node;
  this_node.bounds = get_bounds_from_node_index(node_index, bounds);
  this_node.level = static_cast<int32_t>(node_index.levels()) - 1;
  this_node.max_depth = root_node.max_depth;
  this_node.max_spacing = root_node.max_spacing / std::pow(2, node_index.levels());
  this_node.morton_index = node_index.to_static_morton_index();
  this_node.name = std::string{ "r" } + OctreeNodeIndex64::to_string(node_index);

  return { std::move(merged_data), this_node, root_node };
}

void
TilingAlgorithmV3::reconstruct_single_node(const OctreeNodeIndex64& node, const AABB& root_bounds)
{
  PointBuffer data;
  for (uint8_t octant = 0; octant < 8; ++octant) {
    const auto child_index = node.child(octant);
    const auto node_name = concat("r", OctreeNodeIndex64::to_string(child_index));

    PointBuffer tmp;
    _persistence.retrieve_points(node_name, tmp);

    if (tmp.empty())
      continue;

    data.append_buffer(tmp);
  }

  // 2) Calculate morton indices for child data
  std::vector<IndexedPoint64> indexed_points;
  indexed_points.reserve(data.count());
  index_points<MAX_OCTREE_LEVELS>(std::begin(data),
                                  std::end(data),
                                  std::back_inserter(indexed_points),
                                  root_bounds,
                                  OutlierPointsBehaviour::ClampToBounds);

  // 3) Data is sorted, so we can sample directly
  const auto morton_index_for_node = node.to_static_morton_index();
  const auto selected_points_end = sample_points(_sampling_strategy,
                                                 std::begin(indexed_points),
                                                 std::end(indexed_points),
                                                 morton_index_for_node,
                                                 static_cast<int32_t>(node.levels()) - 1,
                                                 root_bounds,
                                                 _meta_parameters.spacing_at_root,
                                                 SamplingBehaviour::AlwaysAdhereToMinSpacing);

  // 4) Write to disk
  const auto node_bounds = get_bounds_from_node_index(node, root_bounds);
  const auto node_name = concat("r", OctreeNodeIndex64::to_string(node));

  _persistence.persist_points(
    member_iterator(std::begin(indexed_points), &IndexedPoint64::point_reference),
    member_iterator(selected_points_end, &IndexedPoint64::point_reference),
    node_bounds,
    node_name);
}

void
TilingAlgorithmV3::reconstruct_left_out_nodes(const AABB& root_bounds)
{
  if (*_level_of_start_nodes == 0) {
    return;
  }

  const auto node_exists = [this](const OctreeNodeIndex64& node_index) {
    const auto node_name = concat("r", OctreeNodeIndex64::to_string(node_index));
    return _persistence.node_exists(node_name);
  };

  // Collect all nodes that we left out and have to reconstruct. These are the
  // direct and indirect parent nodes of the existing nodes at
  // '_level_of_start_nodes'
  std::unordered_set<OctreeNodeIndex64> nodes_to_reconstruct;

  const auto max_possible_nodes = static_cast<size_t>(std::pow(8, *_level_of_start_nodes));
  for (size_t idx = 0; idx < max_possible_nodes; ++idx) {
    const auto node_index =
      OctreeNodeIndex64::unchecked_from_index_and_levels(idx, *_level_of_start_nodes);
    if (!node_exists(node_index))
      continue;

    auto cur_node = node_index;
    while (cur_node.levels() > 0) {
      cur_node = cur_node.parent();
      nodes_to_reconstruct.insert(cur_node);
    }
  }

  // It is important that we reconstruct nodes from deeper levels to more
  // shallow levels, as each node samples from its direct children, so the
  // children have to be reconstructed before the parents are
  std::vector<OctreeNodeIndex64> sorted_nodes_to_reconstruct{ std::begin(nodes_to_reconstruct),
                                                              std::end(nodes_to_reconstruct) };
  std::sort(std::begin(sorted_nodes_to_reconstruct),
            std::end(sorted_nodes_to_reconstruct),
            [](const auto& l, const auto& r) { return l.levels() > r.levels(); });

  if (debug::Journal::instance().is_enabled()) {
    std::stringstream ss;
    ss << "Reconstructed nodes: [ ";
    for (auto& node : sorted_nodes_to_reconstruct) {
      ss << "\"" << OctreeNodeIndex64::to_string(node) << "\" ";
    }
    ss << "]";
    debug::Journal::instance().add_entry(ss.str());
  }

  const auto t_start = std::chrono::high_resolution_clock::now();

  for (auto& node : sorted_nodes_to_reconstruct) {
    reconstruct_single_node(node, root_bounds);
  }

  const auto t_end = std::chrono::high_resolution_clock::now();
  if (debug::Journal::instance().is_enabled()) {
    const auto delta_t = t_end - t_start;
    const auto delta_t_seconds = static_cast<double>(delta_t.count()) / 1e9;
    debug::Journal::instance().add_entry(
      (boost::format("Reconstructing nodes: %1% s") % delta_t_seconds).str());
  }
}
#pragma endregion