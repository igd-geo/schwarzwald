
#include "datastructures/GridCell.h"
#include "datastructures/SparseGrid.h"
#include "util/stuff.h"

#include <iostream>

GridCell::GridCell() {}

GridCell::GridCell(SparseGrid* grid, GridIndex& index)
{
  neighbours.reserve(26);

  for (int i = std::max(index.i - 1, 0);
       i <= std::min(grid->width - 1, index.i + 1);
       i++) {
    for (int j = std::max(index.j - 1, 0);
         j <= std::min(grid->height - 1, index.j + 1);
         j++) {
      for (int k = std::max(index.k - 1, 0);
           k <= std::min(grid->depth - 1, index.k + 1);
           k++) {
        long long key = ((long long)k << 40) | ((long long)j << 20) | i;
        SparseGrid::iterator it = grid->find(key);
        if (it != grid->end()) {
          GridCell* neighbour = it->second;
          if (neighbour != this) {
            neighbours.push_back(neighbour);
            neighbour->neighbours.push_back(this);
          }
        }
      }
    }
  }
}

void
GridCell::add(const Vector3<double>& p)
{
  points.push_back(p);
}

bool
GridCell::isDistant(const Vector3<double>& p,
                    const double& squaredSpacing,
                    size_t* num_comparisons) const
{
  for (const Vector3<double>& point : points) {
    if (num_comparisons) {
      ++(*num_comparisons);
    }
    if (p.squaredDistanceTo(point) < squaredSpacing) {
      return false;
    }
  }

  return true;
}

size_t
GridCell::content_byte_size() const
{
  return vector_byte_size(points) + vector_byte_size(neighbours);
}
