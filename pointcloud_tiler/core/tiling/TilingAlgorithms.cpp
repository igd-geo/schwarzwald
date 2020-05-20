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

/**
 * Tile the given node as a terminal node, i.e. take up to 'max_points_per_node' points
 * and persist them without any sampling
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
 * Tile the given node as an interior node, i.e. by using the given SamplingStrategy
 */
std::vector<NodeTilingData>
TilingAlgorithmBase::tile_internal_node(octree::NodeData& all_points,
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
TilingAlgorithmBase::tile_node(octree::NodeData&& node_data,
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
TilingAlgorithmBase::do_tiling_for_node(octree::NodeData&& node_data,
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
   * Instead of starting from the root node (which has to be processed by a single thread), select
   * a number of deeper nodes that can all be processed in parallel. This will skip some nodes,
   * their content has to be reconstructed from the deeper nodes at the end, either by subsampling
   * or averaging
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
      // A bit of logic to access the slots of pre-allocated memory for this task
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

    // TODO get_start_nodes_from(ranges_per_node) and then iterate over this
    // --> this is some traversal of the tree, together with a filter operation

    // for (size_t idx = 0; idx < _indexed_points_ranges.size(); ++idx) {
    //   subflow.emplace(
    //     [this, bounds, _data = std::move(ranges_per_node[idx])](tf::Subflow& subsubflow) {
    //       auto process_data = prepare_range_for_tiling(_data, bounds);

    //       do_tiling_for_node(
    //         std::move(process_data.points), process_data.root_node, process_data.node,
    //         subsubflow);
    //     });
    // }
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
 * This method takes a range of IndexedPoints that span the root node of the dataset and
 * break it up into approximately* 'min_number_of_ranges' connected ranges, where each of
 * the new ranges spans exactly one node in the octree. Ranges are split by size, with the
 * largest range being split first. When split, the range is replaced by ALL ranges for its
 * immediate child nodes (up to 8 ranges).
 *
 * To understand this, here is a simplified case in 2D, with node indices written as 'ABC'
 * where A is the nodes index at level 0, B at level 1 etc.
 *
 * Input range: [000, 002, 003, 011, 013, 030, 102, 103, 120, 300, 310, 313]
 * Desired ranges: 4
 *
 * Output: [[000, 002, 003], [011, 013], [030], [102, 103, 120], [300, 310, 313]]
 *
 * This result is achieved in two steps:
 *
 * 1) Split up the root range. Yields:
 * [[000, 002, 003, 011, 013, 030], [102, 103, 120], [300, 310, 313]]
 *  |----------------------------|  |-------------|  |-------------|
 *      all points for node 0          ...node 1        ...node 3
 *
 * 2) Current range count is 3, which is less than desired ranges. Split up the largest range,
 *    which is the range for node 0 (contains 6 points). This yields the final result:
 * [[000, 002, 003], [011, 013], [030], [102, 103, 120], [300, 310, 313]]
 *  |-------------|  |--------|  |---|  |-------------|  |-------------|
 *      node 00        node 01  node 03     node 1            node 3
 */
Octree<TilingAlgorithmV2::IndexedPointsForNode>
TilingAlgorithmV2::split_indexed_points_into_subranges(
  util::Range<IndexedPointsIter> indexed_points,
  size_t min_number_of_ranges) const
{
  Octree<IndexedPointsForNode> point_ranges{ { {},
                                               IndexedPointsForNode(true, { indexed_points }) } };

  if (indexed_points.size() <= min_number_of_ranges) {
    return point_ranges;
  }

  throw std::runtime_error{ "not implemented" };

  // Recursively split the node with the largest range into up to 8 child nodes

  // auto get_largest_range =
  //   [](Octree<IndexedPointsForNode>& ranges) -> std::vector<IndexedPointsRangeForNode>::iterator
  //   { const auto iter_to_max_range =
  //     std::max_element(std::begin(ranges),
  //                      std::end(ranges),
  //                      [](const IndexedPointsRangeForNode& l, const IndexedPointsRangeForNode& r)
  //                      {
  //                        return l.range.size() < r.range.size();
  //                      });
  //   if (iter_to_max_range == std::end(ranges))
  //     return iter_to_max_range;

  //   // To ensure that this whole method always terminates, we have to make SURE that we only
  //   return
  //   // a range from 'get_largest_range' if this range can be split up into its child nodes. This
  //   can
  //   // be done with a simple check for the Morton index difference between the first and last
  //   points
  //   // in the range
  //   const auto& first_point = iter_to_max_range->range.first();
  //   const auto& last_point = iter_to_max_range->range.last();
  //   if (first_point.morton_index == last_point.morton_index)
  //     return std::end(ranges);

  //   return iter_to_max_range;
  // };

  // // As long as we have less than 'min_number_of_ranges', keep splitting up the largest range
  // while (ranges_by_node.size() < min_number_of_ranges) {
  //   auto iter_to_max_range = get_largest_range(ranges_by_node);
  //   if (iter_to_max_range == std::end(ranges_by_node))
  //     break;

  //   auto max_range = *iter_to_max_range;
  //   const auto child_level = max_range.node_index.depth();
  //   const auto child_ranges = partition_points_into_child_octants(
  //     max_range.range.begin(), max_range.range.end(), static_cast<uint32_t>(child_level));

  //   std::vector<IndexedPointsRangeForNode> child_ranges_with_levels;
  //   for (uint8_t octant = 0; octant < 8; ++octant) {
  //     const auto& child_range = child_ranges[octant];
  //     if (child_range.first == child_range.second)
  //       continue;

  //     child_ranges_with_levels.push_back(
  //       { util::Range<IndexedPointsIter>{ child_range.first, child_range.second },
  //         max_range.node_index.child(octant) });
  //   }

  //   // Replace this range with the child ranges
  //   auto insert_iter = ranges_by_node.erase(iter_to_max_range);
  //   ranges_by_node.insert(
  //     insert_iter, std::begin(child_ranges_with_levels), std::end(child_ranges_with_levels));
  // }

  // return ranges_by_node;
}

// std::vector<TilingAlgorithmV2::FindStartNodesResult>
// TilingAlgorithmV2::transpose_ranges(
//   std::vector<std::vector<IndexedPointsRangeForNode>>&& ranges) const
// {
//   // 'ranges' will contain a fairly small number of ranges and unique nodes (~10ish), so
//   // this is implemented using linear search
//   std::vector<FindStartNodesResult> transposed_ranges;

//   auto get_or_create_start_nodes_result =
//     [](const DynamicMortonIndex& morton_index,
//        std::vector<FindStartNodesResult>& transposed_ranges) -> FindStartNodesResult& {
//     auto iter = std::find_if(
//       std::begin(transposed_ranges),
//       std::end(transposed_ranges),
//       [&morton_index](const FindStartNodesResult& res) { return res.node_index == morton_index;
//       });
//     if (iter != std::end(transposed_ranges)) {
//       return *iter;
//     }

//     transposed_ranges.push_back({ morton_index, {} });
//     return transposed_ranges.back();
//   };

//   for (auto& range : ranges) {
//     for (auto& subrange : range) {
//       auto& transposed_range =
//         get_or_create_start_nodes_result(subrange.node_index, transposed_ranges);
//       transposed_range.indexed_points_ranges.push_back(subrange.range);
//     }
//   }

//   return transposed_ranges;
// }

// template<typename V>
// static void
// add_node_to_tree(const DynamicMortonIndex& node_index,
//                  V* node_data,
//                  std::unordered_map<DynamicMortonIndex, V*>& tree)
// {
//   auto iter = tree.find(node_index);
//   if (iter != std::end(tree))
//     return;

//   tree[node_index] = node_data;

//   if (node_index.depth() == 0)
//     return;

//   add_node_to_tree<V>(node_index.parent(), nullptr, tree);
// }

// std::vector<TilingAlgorithmV2::FindStartNodesResult>
// TilingAlgorithmV2::consolidate_ranges(std::vector<FindStartNodesResult>&& ranges,
//                                       size_t min_number_of_ranges) const
// {
//   // Build an tree index from the ranges so that the computation is easier
//   std::unordered_map<DynamicMortonIndex, FindStartNodesResult*> tree;

//   for (auto& result : ranges) {
//     add_node_to_tree(result.node_index, &result, tree);
//   }

//   // Now find all interior nodes in the tree, ideally starting with the deepest interior nodes.
//   We
//   // can then recursively collapse / expand these nodes until we have no interior nodes anymore
//   and
//   // an acceptable number of ranges
//   throw std::runtime_error{ "not implemented" };
// }

Octree<TilingAlgorithmV2::IndexedPointsForNode>
TilingAlgorithmV2::merge_selected_start_nodes(
  const std::vector<Octree<IndexedPointsForNode>>& selected_nodes,
  size_t min_number_of_ranges)
{
  throw std::runtime_error{ "not implemented" };
}

NodeTilingData
TilingAlgorithmV2::prepare_range_for_tiling(const IndexedPointsForNode& start_node_data,
                                            OctreeNodeIndex64 node_index,
                                            const AABB& bounds)
{
  throw std::runtime_error{ "not implemented" };
  // const auto start_node_point_count =
  //   std::accumulate(std::begin(start_node_data.indexed_points_ranges),
  //                   std::end(start_node_data.indexed_points_ranges),
  //                   size_t{ 0 },
  //                   [](size_t accum, const auto& range) { return accum + range.size(); });

  // octree::NodeData merged_data{ start_node_point_count };

  // merge_ranges(util::range(start_node_data.indexed_points_ranges),
  //              util::range(merged_data),
  //              [](const IndexedPoint64& l, const IndexedPoint64& r) {
  //                return l.morton_index.get() < r.morton_index.get();
  //              });

  // octree::NodeStructure root_node;
  // root_node.bounds = bounds;
  // root_node.level = -1;
  // root_node.max_depth = _meta_parameters.max_depth;
  // root_node.max_spacing = _meta_parameters.spacing_at_root;
  // root_node.morton_index = {};
  // root_node.name = "r";

  // octree::NodeStructure this_node;
  // this_node.bounds = get_bounds_from_morton_index(start_node_data.node_index, bounds);
  // this_node.level = static_cast<int32_t>(start_node_data.node_index.depth()) - 1;
  // this_node.max_depth = root_node.max_depth;
  // this_node.max_spacing = root_node.max_spacing / std::pow(2,
  // start_node_data.node_index.depth()); this_node.morton_index =
  // start_node_data.node_index.to_static_morton_index<MAX_OCTREE_LEVELS>(); this_node.name =
  // to_string(start_node_data.node_index, MortonIndexNamingConvention::Potree);

  // return { std::move(merged_data), root_node, this_node };
}

#pragma endregion