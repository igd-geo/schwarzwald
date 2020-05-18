#pragma once

#include "datastructures/PointBuffer.h"
#include "io/PointsPersistence.h"
#include "process/Tiler.h"
#include "tiling/Node.h"
#include "tiling/Sampling.h"

#include <containers/Range.h>

#include <memory>
#include <taskflow/taskflow.hpp>
#include <vector>

struct ProgressReporter;

/**
 * Helper structure that stores PointBuffer objects in a thread-safe manner. This is used to cache
 * the points loaded from disk at each node.
 */
struct PointsCache
{
  PointsCache() {}
  PointsCache(const PointsCache&) = delete;
  PointsCache(PointsCache&&) = delete;
  PointsCache& operator=(const PointsCache&) = delete;
  PointsCache& operator=(PointsCache&&) = delete;

  PointBuffer& emplace_points(PointBuffer&& points)
  {
    std::lock_guard guard{ _lock };
    _cache.push_back(std::make_unique<PointBuffer>(std::move(points)));
    return *_cache.back();
  }

  void clear()
  {
    std::lock_guard guard{ _lock };
    _cache.clear();
  }

private:
  std::vector<std::unique_ptr<PointBuffer>> _cache;
  std::mutex _lock;
};

/**
 * Helper structure that encapsulates data for tiling a single node
 */
struct NodeTilingData
{
  NodeTilingData() {}
  NodeTilingData(octree::NodeData points,
                 octree::NodeStructure node,
                 octree::NodeStructure root_node)
    : points(std::move(points))
    , node(node)
    , root_node(root_node)
  {}

  octree::NodeData points;
  octree::NodeStructure node;
  octree::NodeStructure root_node;
};

/**
 * Base class for different tiling algorithms
 */
struct TilingAlgorithmBase
{
  TilingAlgorithmBase(SamplingStrategy& sampling_strategy,
                      ProgressReporter* progress_reporter,
                      PointsPersistence& persistence,
                      TilerMetaParameters meta_parameters,
                      size_t concurrency);
  /**
   * Build an execution graph for tiling the given PointBuffer
   */
  virtual void build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf) = 0;

protected:
  std::vector<NodeTilingData> tile_node(octree::NodeData&& node_data,
                                        const octree::NodeStructure& node_structure,
                                        const octree::NodeStructure& root_node_structure,
                                        tf::Subflow& subflow);
  void tile_terminal_node(octree::NodeData const& all_points,
                          octree::NodeStructure const& node,
                          size_t previously_taken_points);
  std::vector<NodeTilingData> tile_internal_node(octree::NodeData& all_points,
                                                 octree::NodeStructure const& node,
                                                 octree::NodeStructure const& root_node,
                                                 size_t previously_taken_points);
  void do_tiling_for_node(octree::NodeData&& node_data,
                          const octree::NodeStructure& node_structure,
                          const octree::NodeStructure& root_node_structure,
                          tf::Subflow& subflow);

  SamplingStrategy& _sampling_strategy;
  ProgressReporter* _progress_reporter;
  PointsPersistence& _persistence;
  TilerMetaParameters _meta_parameters;
  size_t _concurrency;

  octree::NodeData _root_node_points;
  PointsCache _points_cache;
};

/**
 * Version 1 of the tiling algorithm, as presented in the first version of my paper. It uses:
 *
 * -  Parallel indexing
 * -  Sequential sorting
 * -  Processing from the root node
 */
struct TilingAlgorithmV1 : TilingAlgorithmBase
{
  TilingAlgorithmV1(SamplingStrategy& sampling_strategy,
                    ProgressReporter* progress_reporter,
                    PointsPersistence& persistence,
                    TilerMetaParameters meta_parameters,
                    size_t concurrency);

  void build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf) override;
};

/**
 * Optimized version of the tiling algorithm. It uses:
 *
 * -  Parallel indexing
 * -  Skipping root node and selecting 'desired_parallelism' nodes for processing
 * -  Parallel sorting for each selected node
 *
 * In general, a lot of parallel map/reduce operations
 */
struct TilingAlgorithmV2 : TilingAlgorithmBase
{
  TilingAlgorithmV2(SamplingStrategy& sampling_strategy,
                    ProgressReporter* progress_reporter,
                    PointsPersistence& persistence,
                    TilerMetaParameters meta_parameters,
                    size_t concurrency);

  void build_execution_graph(PointBuffer& points, const AABB& bounds, tf::Taskflow& tf) override;

private:
  using IndexedPoints = std::vector<IndexedPoint64>;
  using IndexedPointsIter = typename IndexedPoints::iterator;
  using PointsIter = typename PointBuffer::PointIterator;

  /**
   * A range of indexed points for a specific octree node
   */
  struct IndexedPointsRangeForNode
  {
    util::Range<IndexedPointsIter> range;
    DynamicMortonIndex node_index;
  };

  /**
   * All the ranges of indexed points that belong to the given octree node. The ranges
   * are sorted but possibly disjoint, i.e. two ranges are not guaranteed to be adjacent
   * to each other in memory
   */
  struct FindStartNodesResult
  {
    DynamicMortonIndex node_index;
    std::vector<util::Range<IndexedPointsIter>> indexed_points_ranges;
  };

  /**
   * Takes a range of points from a PointBuffer, calculates the Morton indices for the points
   * and sorts them based on the Morton indices
   */
  void index_and_sort_points(util::Range<PointsIter> points,
                             util::Range<IndexedPointsIter> indexed_points,
                             const AABB& bounds) const;

  std::vector<IndexedPointsRangeForNode> split_indexed_points_into_subranges(
    util::Range<IndexedPointsIter> indexed_points,
    size_t min_number_of_ranges) const;

  /**
   * Takes the results of N invocations of 'split_indexed_points_into_subranges', where each
   * invocation resulted in approximately M subranges, where M is the number of nodes that the
   * TilingAlgorithmV2 starts the processing with. Each subrange spans points that belong to a
   * specific octree node. This method then takes all these subranges and transposes them into a
   * range of ranges where the inner ranges contain all subranges that belong to a single node.
   *
   * This is hard to read and easy to visualize, so here is a diagram:
   * Input:
   *  [
   *   [RangeForNodeA, RangeForNodeB, RangeForNodeD, RangeForNodeE],
   *   [RangeForNodeB, RangeForNodeC, RangeForNodeE, RangeForNodeF],
   *   [RangeForNodeA, RangeForNodeD, RangeForNodeE, RangeForNodeF],
   *   ...
   *  ]
   *
   * Output:
   *  [
   *   [Range1ForNodeA, Range2ForNodeA],
   *   [Range1ForNodeB, Range2ForNodeB],
   *   [Range1ForNodeC],
   *   [Range1ForNodeD, Range2ForNodeD],
   *   [Range1ForNodeE, Range2ForNodeE, Range3ForNodeE],
   *   [Range1ForNodeF, Range2ForNodeF]
   *  ]
   *
   */
  std::vector<FindStartNodesResult> transpose_ranges(
    std::vector<std::vector<IndexedPointsRangeForNode>>&& ranges) const;

  /**
   * The result of 'split_indexed_points_into_subranges' can be different for each invocation (as it
   * is called with a different set of points). Through this, the situation can arise that in one
   * invocation, a node 'A' is selected, but in another invocation, all the children of 'A' are
   * selected instead. When we then call 'transpose_ranges' to merge all these ranges, we will end
   * up with multiple root nodes where one node might be a child of the other node. This must not
   * happen (parent-child relationship prevents parallel processing), so we have to ensure that we
   * only take either 'A' or the children of 'A' as root nodes, never both.
   *
   * This is somewhat nasty, as of course the parent-child relationship can be indirect. Ultimately,
   * 'ranges' represents a k-way tree (with k <= 8) and we have to eliminate all the 'inner' nodes
   * of this tree. The tree representation might not be complete ('ranges' contains just a bunch of
   * nodes, no relationships, nodes may be missing because they were never selected).
   */
  std::vector<FindStartNodesResult> consolidate_ranges(
    std::vector<FindStartNodesResult>&& ranges, size_t min_number_of_ranges) const;

  /**
   * Takes a range of sorted IndexedPoint ranges for a single node, merges the ranges into a single
   * sorted range and returns the necessary data for tiling this node
   */
  NodeTilingData prepare_range_for_tiling(const FindStartNodesResult& start_node_data,
                                          const AABB& bounds);

  std::vector<std::vector<IndexedPointsRangeForNode>> _indexed_points_ranges;
};