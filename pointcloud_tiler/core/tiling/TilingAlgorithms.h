#pragma once

#include "datastructures/Octree.h"
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
  // struct IndexedPointsForNode
  // {
  //   IndexedPointsForNode()
  //     : is_start_node(false)
  //   {}
  //   IndexedPointsForNode(bool is_start_node, std::vector<util::Range<IndexedPointsIter>> ranges)
  //     : is_start_node(is_start_node)
  //     , ranges(std::move(ranges))
  //   {}

  //   bool is_start_node;
  //   std::vector<util::Range<IndexedPointsIter>> ranges;
  // };

  /**
   * Takes a range of points from a PointBuffer, calculates the Morton indices for the points
   * and sorts them based on the Morton indices
   */
  void index_and_sort_points(util::Range<PointsIter> points,
                             util::Range<IndexedPointsIter> indexed_points,
                             const AABB& bounds) const;

  Octree<util::Range<IndexedPointsIter>> split_indexed_points_into_subranges(
    util::Range<IndexedPointsIter> indexed_points,
    size_t min_number_of_ranges) const;

  /**
   * Merge the results of multiple 'split_indexed_points_into_subranges' invocations
   */
  Octree<std::vector<util::Range<IndexedPointsIter>>> merge_selected_start_nodes(
    const std::vector<Octree<util::Range<IndexedPointsIter>>>& selected_nodes,
    size_t min_number_of_ranges);

  /**
   * Takes a range of sorted IndexedPoint ranges for a single node, merges the ranges into a single
   * sorted range and returns the necessary data for tiling this node
   */
  NodeTilingData prepare_range_for_tiling(
    const std::vector<util::Range<IndexedPointsIter>>& start_node_data,
    OctreeNodeIndex64 node_index,
    const AABB& bounds);

  /**
   * Reconstruct the nodes that we left out initially
   */
  void reconstruct_left_out_nodes(const std::unordered_set<OctreeNodeIndex64>& left_out_nodes);

  std::vector<Octree<util::Range<IndexedPointsIter>>> _indexed_points_ranges;
};