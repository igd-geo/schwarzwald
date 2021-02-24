#pragma once

#include "datastructures/GridIndex.h"
#include "math/Vector3.h"

#include <math.h>
#include <vector>

class SparseGrid;

class GridCell
{
public:
  std::vector<Vector3<double>> points;
  std::vector<GridCell*> neighbours;

  GridCell();

  GridCell(SparseGrid* grid, GridIndex& index);

  void add(const Vector3<double>& p);

  bool isDistant(const Vector3<double>& p,
                 const double& squaredSpacing,
                 size_t* num_comparisons = nullptr) const;

  size_t content_byte_size() const;
};