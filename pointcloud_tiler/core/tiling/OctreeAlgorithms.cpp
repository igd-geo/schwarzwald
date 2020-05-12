#include "tiling/OctreeAlgorithms.h"

AABB
get_octant_bounds(uint8_t octant, const AABB& parent_bounds)
{
  const auto parent_extent = parent_bounds.extent();
  const auto min_z =
    (octant & 1) ? (parent_bounds.min.z + parent_extent.z / 2) : (parent_bounds.min.z);
  const auto min_y =
    ((octant >> 1) & 1) ? (parent_bounds.min.y + parent_extent.y / 2) : (parent_bounds.min.y);
  const auto min_x =
    ((octant >> 2) & 1) ? (parent_bounds.min.x + parent_extent.x / 2) : (parent_bounds.min.x);

  const auto max_x = min_x + parent_extent.x / 2;
  const auto max_y = min_y + parent_extent.y / 2;
  const auto max_z = min_z + parent_extent.z / 2;
  return { { min_x, min_y, min_z }, { max_x, max_y, max_z } };
}

AABB
get_bounds_from_node_name(const std::string& node_name, const AABB& root_bounds)
{
  auto current_bounds = root_bounds;
  for (size_t idx = 1; idx < node_name.size(); ++idx) {
    const auto octant = static_cast<uint8_t>(node_name[idx] - '0');
    current_bounds = get_octant_bounds(octant, current_bounds);
  }
  return current_bounds;
}

AABB
get_bounds_from_morton_index(const DynamicMortonIndex& morton_index, const AABB& root_bounds)
{
  auto current_bounds = root_bounds;
  for (auto octant : morton_index) {
    current_bounds = get_octant_bounds(octant, current_bounds);
  }
  return current_bounds;
}

uint8_t
get_octant(const Vector3<double>& position, const AABB& bounds)
{
  const auto bounds_extent = bounds.extent();
  auto nx = (uint8_t)(2 * (position.x - bounds.min.x) / bounds_extent.x);
  auto ny = (uint8_t)(2 * (position.y - bounds.min.y) / bounds_extent.y);
  auto nz = (uint8_t)(2 * (position.z - bounds.min.z) / bounds_extent.z);

  auto ix = std::min(nx, (uint8_t)1);
  auto iy = std::min(ny, (uint8_t)1);
  auto iz = std::min(nz, (uint8_t)1);

  return (iz) | (iy << 1) | (ix << 2);
}