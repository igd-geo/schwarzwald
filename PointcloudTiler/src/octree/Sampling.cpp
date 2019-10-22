#include "octree/Sampling.h"

RandomSortedGridSampling::RandomSortedGridSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

GridCenterSampling::GridCenterSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

PoissonDiskSampling::PoissonDiskSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}

FastPoissonDiskSampling::FastPoissonDiskSampling(size_t max_points_per_node)
  : _max_points_per_node(max_points_per_node)
{}