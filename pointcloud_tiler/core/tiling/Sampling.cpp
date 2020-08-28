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
      [node_level](const PoissonDiskSampling&) -> int32_t { return node_level; },
      [node_level](const AdaptivePoissonDiskSampling&) -> int32_t { return node_level; } },
    sampling_strategy);
}