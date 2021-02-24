#pragma once

#include "datastructures/GridCell.h"
#include "math/AABB.h"

#include <map>
#include <math.h>
#include <unordered_map>

class SparseGrid : public std::unordered_map<long long, GridCell*>
{
public:
  int width;
  int height;
  int depth;
  AABB aabb;
  float squaredSpacing;
  unsigned int numAccepted = 0;

  SparseGrid(AABB aabb, float minGap);

  SparseGrid(const SparseGrid& other)
    : std::unordered_map<long long, GridCell*>(other)
    , width(other.width)
    , height(other.height)
    , depth(other.depth)
    , aabb(other.aabb)
    , squaredSpacing(other.squaredSpacing)
    , numAccepted(other.numAccepted)
    , _dbg_num_comparisons(other._dbg_num_comparisons)
  {}

  ~SparseGrid();

  bool isDistant(const Vector3<double>& p, GridCell* cell);

  bool isDistant(const Vector3<double>& p,
                 GridCell* cell,
                 float& squaredSpacing);

  bool willBeAccepted(const Vector3<double>& p);

  bool willBeAccepted(const Vector3<double>& p, float& squaredSpacing);

  bool add(const Vector3<double>& p);

  void addWithoutCheck(const Vector3<double>& p);

  size_t content_byte_size() const;

  inline size_t dbg_num_comparisons() const { return _dbg_num_comparisons; }

private:
  size_t _dbg_num_comparisons;
};