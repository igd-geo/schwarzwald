#include "octree/OctreeAlgorithms.h"

Potree::AABB
get_octant_bounds(uint8_t octant, const Potree::AABB& parent_bounds)
{
  const auto min_z =
    (octant & 1) ? (parent_bounds.min.z + parent_bounds.size.z / 2) : (parent_bounds.min.z);
  const auto min_y =
    ((octant >> 1) & 1) ? (parent_bounds.min.y + parent_bounds.size.y / 2) : (parent_bounds.min.y);
  const auto min_x =
    ((octant >> 2) & 1) ? (parent_bounds.min.x + parent_bounds.size.x / 2) : (parent_bounds.min.x);

  const auto max_x = min_x + parent_bounds.size.x / 2;
  const auto max_y = min_y + parent_bounds.size.y / 2;
  const auto max_z = min_z + parent_bounds.size.z / 2;
  return { { min_x, min_y, min_z }, { max_x, max_y, max_z } };
}

uint8_t
get_octant(const Potree::Vector3<double>& position, const Potree::AABB& bounds)
{
  auto nx = (uint8_t)(2 * (position.x - bounds.min.x) / bounds.size.x);
  auto ny = (uint8_t)(2 * (position.y - bounds.min.y) / bounds.size.y);
  auto nz = (uint8_t)(2 * (position.z - bounds.min.z) / bounds.size.z);

  auto ix = min(nx, (uint8_t)1);
  auto iy = min(ny, (uint8_t)1);
  auto iz = min(nz, (uint8_t)1);

  return (iz) | (iy << 1) | (ix << 2);
}