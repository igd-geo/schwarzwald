#pragma once

#include "OctreeAlgorithms.h"
#include "datastructures/DynamicMortonIndex.h"
#include "datastructures/MortonIndex.h"
#include "math/AABB.h"

#include <unordered_map>

namespace octree {

struct NodeStructure
{
  std::string name;
  MortonIndex64 morton_index;
  AABB bounds;
  int32_t level;
  float max_spacing;
  uint32_t max_depth;
};

using NodeData = std::vector<IndexedPoint64>;

using HierarchyOctree = std::unordered_map<DynamicMortonIndex, NodeStructure>;

/**
 * Merge NodeData for two octree nodes, assuming that the data is sorted
 */
NodeData
merge_node_data_sorted(NodeData&& first_node, NodeData&& second_node);
/**
 * Merge NodeData for two octree nodes in unsorted fashion
 */
NodeData
merge_node_data_unsorted(NodeData&& first_node, NodeData&& second_node);

/**
 * Returns the first node level (relative to root) with a side-length that is
 * less than two times the given spacing
 */
int32_t
first_node_level_obeying_spacing(float target_spacing, NodeStructure const& root_node);
/**
 * Returns the level of the node to sample from
 */
int32_t
get_node_level_to_sample_from(int32_t source_node_level, NodeStructure const& root_node);
} // namespace octree