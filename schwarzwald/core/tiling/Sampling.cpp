#include "tiling/Sampling.h"
#include "tiling/Node.h"

#include <types/type_util.h>

RandomSortedGridSampling::RandomSortedGridSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

GridCenterSampling::GridCenterSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

PoissonDiskSampling::PoissonDiskSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

AdaptivePoissonDiskSampling::AdaptivePoissonDiskSampling(
  size_t max_points_per_node,
  std::function<float(int32_t)> density_per_level)
  : _max_points_per_node(max_points_per_node)
  , _density_per_level(density_per_level)
{}

JitteredSampling::JitteredSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

int32_t
required_morton_index_depth(const SamplingStrategy& sampling_strategy,
                            int32_t node_level,
                            const octree::NodeStructure& root_node)
{
  return std::visit(
    overloaded{
      [node_level, &root_node](const RandomSortedGridSampling&) -> int32_t {
        return octree::get_node_level_to_sample_from(node_level, root_node);
      },
      [node_level, &root_node](const GridCenterSampling&) -> int32_t {
        return octree::get_node_level_to_sample_from(node_level, root_node);
      },
      [node_level](const PoissonDiskSampling&) -> int32_t {
        return node_level;
      },
      [node_level](const AdaptivePoissonDiskSampling&) -> int32_t {
        return node_level;
      },
      [node_level, &root_node](const JitteredSampling&) -> int32_t {
        const auto spacing_at_this_node =
          root_node.max_spacing / std::pow(2, node_level + 1);
        const auto perfect_cell_count =
          (root_node.bounds.extent().x / std::pow(2, node_level + 1)) /
          spacing_at_this_node;
        const auto actual_cell_count =
          get_prev_power_of_two(static_cast<uint32_t>(perfect_cell_count));

        const uint32_t levels =
          static_cast<uint32_t>(std::log2(actual_cell_count));
        return static_cast<uint32_t>(node_level + levels);
      } },
    sampling_strategy);
}