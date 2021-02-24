#pragma once

#include "MortonIndex.h"
#include "OctreeNodeIndex.h"
#include "math/AABB.h"

#include <boost/container/flat_map.hpp>
#include <unordered_map>
#include <vector>

/**
 * Sparse grid implementation that uses Morton indices. It also assumes that
 * points are added in Morton order, so neighbourhood checks only have to be
 * performed for cells that have a smaller Morton index than the current cell
 */
struct MortonGrid
{
  /**
   * Creates a new MortonGrid from the given AABB and target spacing. The AABB
   * must be a cube that belongs to a specific octree node
   */
  MortonGrid(const AABB& bounds,
             double spacing,
             uint32_t level_in_octree_of_bounds);

  bool try_add(const Vector3<double>& point, MortonIndex64 point_index);

  size_t dbg_num_comparisons() const;

private:
  using Cell = std::vector<Vector3<double>>;
  bool fits_in_cell(const Cell& cell, const Vector3<double>& point) const;

  bool try_add_naive(const Vector3<double>& point, MortonIndex64 point_index);

  // boost::container::flat_map<MortonIndex64, Cell> _cells;
  std::unordered_map<OctreeNodeIndex64, Cell> _cells;
  // std::vector<std::pair<Vector3<double>, OctreeNodeIndex64>> _taken_points;
  AABB _bounds;
  double _spacing;
  double _squared_spacing;
  double _cell_size;
  uint32_t _root_cell_depth;
  uint32_t _cell_depth;

  mutable size_t _dbg_num_comparisons;
};