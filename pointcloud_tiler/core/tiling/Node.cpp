#include "tiling/Node.h"

octree::NodeData
octree::merge_node_data_sorted(NodeData&& first_node, NodeData&& second_node)
{
  if (first_node.empty())
    return std::move(second_node);
  if (second_node.empty())
    return std::move(first_node);

  NodeData merged;
  merged.reserve(first_node.size() + second_node.size());
  std::merge(first_node.begin(),
             first_node.end(),
             second_node.begin(),
             second_node.end(),
             std::back_inserter(merged),
             [](const auto& idx_l, const auto& idx_r) {
               return idx_l.morton_index.get() < idx_r.morton_index.get();
             });
  return merged;
}

octree::NodeData
octree::merge_node_data_unsorted(NodeData&& first_node, NodeData&& second_node)
{
  if (first_node.empty())
    return std::move(second_node);
  if (second_node.empty())
    return std::move(first_node);

  first_node.insert(first_node.end(), second_node.begin(), second_node.end());
  return std::move(first_node);
}

int32_t
octree::first_node_level_obeying_spacing(float target_spacing, NodeStructure const& root_node)
{
  // Returns the last level (starting from root) at which the node size is >=
  // target spacing.
  return std::max(-1,
                  (int)std::floor(std::log2f(root_node.bounds.extent().x / target_spacing)) -
                    1); // the root node (whole octree) is level '-1', so
                        // level 0 has a sidelength of half the max octree,
                        // hence we have to subtract one here
}

int32_t
octree::get_node_level_to_sample_from(int32_t source_node_level, NodeStructure const& root_node)
{
  // Target spacing is calculated from root spacing by halfing
  // at each level Since the root node is level -1, spacing at level 0 is half
  // spacing_at_root
  const auto spacing_at_target_node = root_node.max_spacing / std::pow(2, source_node_level + 1);

  return first_node_level_obeying_spacing(spacing_at_target_node, root_node);
}