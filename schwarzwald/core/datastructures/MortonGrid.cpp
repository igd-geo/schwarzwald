#include "datastructures/MortonGrid.h"
#include "tiling/OctreeAlgorithms.h"

uint32_t
get_prev_power_of_two(uint32_t x)
{
  x = x | (x >> 1);
  x = x | (x >> 2);
  x = x | (x >> 4);
  x = x | (x >> 8);
  x = x | (x >> 16);
  return x - (x >> 1);
}

/**
 * Given the index of a cell within the MortonGrid, returns the index of the 3x3
 * neighbourhood of the cell (including the cell itself) within the grid. Only
 * cells within the grid are returned, so the returned indices might be less
 * than 27
 */
static std::vector<OctreeNodeIndex64>
get_neighbour_cells(const OctreeNodeIndex64& cell_index)
{
  const auto max_cell = (1ull << cell_index.levels()) - 1ull;
  const auto grid_index = cell_index.to_grid_index();

  const auto x_min = (grid_index.x == 0) ? 0 : grid_index.x - 1;
  const auto y_min = (grid_index.y == 0) ? 0 : grid_index.y - 1;
  const auto z_min = (grid_index.z == 0) ? 0 : grid_index.z - 1;
  const auto x_max = (grid_index.x == max_cell) ? max_cell : grid_index.x + 1;
  const auto y_max = (grid_index.y == max_cell) ? max_cell : grid_index.y + 1;
  const auto z_max = (grid_index.z == max_cell) ? max_cell : grid_index.z + 1;
  const auto num_neighbours =
    (x_max - x_min + 1) * (y_max - y_min + 1) * (z_max - z_min + 1);

  std::vector<OctreeNodeIndex64> neighbours;
  neighbours.reserve(num_neighbours);
  for (auto x = x_min; x <= x_max; ++x) {
    for (auto y = y_min; y <= y_max; ++y) {
      for (auto z = z_min; z <= z_max; ++z) {
        const auto neighbour_index =
          OctreeNodeIndex64::from_grid_index({ x, y, z }, cell_index.levels());
        neighbours.push_back(neighbour_index);
      }
    }
  }
  return neighbours;
}

MortonGrid::MortonGrid(const AABB& bounds,
                       double spacing,
                       uint32_t level_in_octree_of_bounds)
  : _bounds(bounds)
  , _spacing(spacing)
  , _squared_spacing(spacing * spacing)
  , _root_cell_depth(level_in_octree_of_bounds)
  , _dbg_num_comparisons(0)
{
  if (spacing > bounds.extent().x) {
    throw std::runtime_error{
      "Spacing must not be larger than the given bounds!"
    };
  }
  //   if (bounds.extent().x != bounds.extent().y ||
  //       bounds.extent().x != bounds.extent().z) {
  //     throw std::runtime_error{ "Bounds must be cubic!" };
  //   }

  // Find level relative to bounds for which spacing <= cell_side_length <=
  // 2*spacing
  const auto exact_cells_count_per_axis = bounds.extent().x / spacing;
  if (exact_cells_count_per_axis > UINT32_MAX) {
    throw std::runtime_error{ "Too many cells!" };
  }
  const auto cells_count_per_axis =
    get_prev_power_of_two(static_cast<uint32_t>(exact_cells_count_per_axis)) >>
    2;
  _cell_size = bounds.extent().x / cells_count_per_axis;
  _cell_depth = static_cast<uint32_t>(std::log2(cells_count_per_axis));
}

bool
MortonGrid::try_add(const Vector3<double>& point, MortonIndex64 point_index)
{
  // Since the points are Morton sorted, maybe we can use the implicit grid that
  // they form to just search within the already selected points by identifying
  // all cells adjacent to the checked point. Kinda like an explicit grid, but
  // the cells are formed through the sorted order. Might be worth checking out,
  // and could save memory

  // Get the part of the Morton index that corresponds to the grid cell of the
  // point within the bounding box of this MortonGrid. This excludes all the
  // high bits for the levels prior to the level of the node that spans this
  // MortonGrid, but also includes the lower bits for all nodes that are smaller
  // than _cell_size
  const auto grid_bit_mask = (1 << _cell_depth) - 1;
  const auto point_cell_index =
    OctreeNodeIndex64::unchecked_from_index_and_levels(
      point_index.truncate_to_level(_root_cell_depth + _cell_depth).get() &
        grid_bit_mask,
      _cell_depth);
  const auto neighbours = get_neighbour_cells(point_cell_index);

  for (auto& neighbour : neighbours) {
    const auto cell_iter = _cells.find(neighbour);
    if (cell_iter == std::end(_cells))
      continue;
    if (!fits_in_cell(cell_iter->second, point))
      return false;
  }

  _cells[point_cell_index].push_back(point);
  return true;

  // auto next_neighbour_iter = std::begin(neighbours);
  // auto next_point_iter = std::begin(_taken_points);
  // while (next_neighbour_iter != std::end(neighbours) &&
  //        next_point_iter != std::end(_taken_points)) {
  //   const auto& current_cell_index = *next_neighbour_iter;
  //   const auto index_diff =
  //     (next_point_iter->second.index() - current_cell_index.index());
  //   if (index_diff < 0) {
  //     // next_point_iter is in a grid cell prior to next_neighbour_iter
  //     // --> Search for next point that is in the same cell (or any cell
  //     after
  //     // that)
  //     next_point_iter = std::find_if(
  //       next_point_iter + 1,
  //       std::end(_taken_points),
  //       [current_cell_index](const auto& point_index_pair) {
  //         return point_index_pair.second.index() >=
  //         current_cell_index.index();
  //       });
  //   } else if (index_diff > 0) {
  //     // next_point_iter is in a grid cell after next_neighbour_iter
  //     ++next_neighbour_iter;
  //   } else {
  //     // next_point_iter is within the cell of next_neighbour_iter --> We
  //     have
  //     // to compare
  //     const auto& other_point = next_point_iter->first;
  //     if (other_point.squaredDistanceTo(point) < _squared_spacing)
  //       return false;

  //     ++next_point_iter;
  //   }
  // }

  // _taken_points.push_back(std::make_pair(point, point_cell_index));
  // return true;

  // point_index is the Morton index relative to the root node of the octree.
  // What the octree is is irrelevant, we just know that this grid's bounds sit
  // at '_root_cell_depth' in the octree. To find the cell index, we look at the
  // part of the MortonIndex between '_root_cell_depth' and '_root_cell_depth +
  // _cell_depth'. We don't have to trim everything before '_root_cell_depth'
  // because this will be identical for all points (since it is invalid to pass
  // points that are not within '_bounds' to this method
  //   const auto truncated_index =
  //     point_index.truncate_to_level(_root_cell_depth + _cell_depth);
  // const auto node_index = OctreeNodeIndex64::unchecked_from_index_and_levels(
  //   point_index.truncate_to_level(_root_cell_depth + _cell_depth).get(),
  //   _root_cell_depth + _cell_depth);
  // const auto grid_index = node_index.to_grid_index();

  // // Check all previous neighbours and also the cell that the point falls
  // into const auto max_cell = (1ull << (_root_cell_depth + _cell_depth)) -
  // 1ull; const auto x_min = (grid_index.x == 0) ? 0 : grid_index.x - 1; const
  // auto y_min = (grid_index.y == 0) ? 0 : grid_index.y - 1; const auto z_min =
  // (grid_index.z == 0) ? 0 : grid_index.z - 1; const auto x_max =
  // (grid_index.x == max_cell) ? max_cell : grid_index.x + 1; const auto y_max
  // = (grid_index.y == max_cell) ? max_cell : grid_index.y + 1; const auto
  // z_max = (grid_index.z == max_cell) ? max_cell : grid_index.z + 1;

  // const auto self_node_name = OctreeNodeIndex64::to_string(
  //   node_index, MortonIndexNamingConvention::Potree);

  // //   const auto local_x = grid_index.x & ((1 << _cell_depth) - 1);
  // //   const auto local_y = grid_index.y & ((1 << _cell_depth) - 1);
  // //   const auto local_z = grid_index.z & ((1 << _cell_depth) - 1);
  // //   // const auto local_max_cell = (1ull << _cell_depth) - 1ull;
  // //   const auto this_cell_bounds =
  // //     get_bounds_from_node_index(OctreeNodeIndex64::from_grid_index(
  // //                                  { local_x, local_y, local_z },
  // //                                  _cell_depth),
  // //                                _bounds);
  // //   if (!this_cell_bounds.isInside(point)) {
  // //     throw std::runtime_error{ "Bounds calculation is wrong!" };
  // //   }

  // for (auto z = z_min; z <= z_max; ++z) {
  //   for (auto y = y_min; y <= y_max; ++y) {
  //     for (auto x = x_min; x <= x_max; ++x) {
  //       const auto cell_index = OctreeNodeIndex64::from_grid_index(
  //         { x, y, z }, _root_cell_depth + _cell_depth);
  //       const auto cell_name = OctreeNodeIndex64::to_string(
  //         cell_index, MortonIndexNamingConvention::Potree);
  //       const auto cell_morton_index =
  //         cell_index.to_static_morton_index().truncate_to_level(
  //           _root_cell_depth + _cell_depth);
  //       const auto cell_iter = _cells.find(cell_morton_index);
  //       if (cell_iter == std::end(_cells))
  //         continue;

  //       const auto& neighbour_cell = cell_iter->second;
  //       if (!fits_in_cell(neighbour_cell, point))
  //         return false;
  //     }
  //   }
  // }

  // // It fits!
  // const auto this_cell_index =
  //   node_index.to_static_morton_index().truncate_to_level(_root_cell_depth +
  //                                                         _cell_depth);
  // _cells[this_cell_index].push_back(point);
  // return true;
}

bool
MortonGrid::fits_in_cell(const MortonGrid::Cell& cell,
                         const Vector3<double>& point) const
{
  return std::all_of(
    std::begin(cell), std::end(cell), [this, &point](const auto& other_point) {
      ++_dbg_num_comparisons;
      return point.squaredDistanceTo(other_point) >= _squared_spacing;
    });
}

size_t
MortonGrid::dbg_num_comparisons() const
{
  return _dbg_num_comparisons;
}

bool
MortonGrid::try_add_naive(const Vector3<double>& point,
                          MortonIndex64 point_index)
{
  // auto& cell = _cells[0];
  // for (auto& other : cell) {
  //   if (other.squaredDistanceTo(point) < _squared_spacing)
  //     return false;
  // }

  // cell.push_back(point);
  return true;
}