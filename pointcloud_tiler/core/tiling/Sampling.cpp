#include "tiling/Sampling.h"

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