
#include <iostream>
#include <math.h>
#include <numeric>

#include "GridIndex.h"
#include "SparseGrid.h"

const double cellSizeFactor = 5.0;

SparseGrid::SparseGrid(AABB aabb, float spacing)
  : aabb(aabb)
  , squaredSpacing(spacing * spacing)
{
  const auto bounds_extent = aabb.extent();
  this->width = (int)(bounds_extent.x / (spacing * cellSizeFactor));
  this->height = (int)(bounds_extent.y / (spacing * cellSizeFactor));
  this->depth = (int)(bounds_extent.z / (spacing * cellSizeFactor));
}

SparseGrid::~SparseGrid()
{
  SparseGrid::iterator it;
  for (it = begin(); it != end(); it++) {
    delete it->second;
  }
}

bool
SparseGrid::isDistant(const Vector3<double>& p, GridCell* cell)
{
  if (!cell->isDistant(p, squaredSpacing)) {
    return false;
  }

  for (const auto& neighbour : cell->neighbours) {
    if (!neighbour->isDistant(p, squaredSpacing)) {
      return false;
    }
  }

  return true;
}

bool
SparseGrid::isDistant(const Vector3<double>& p, GridCell* cell, float& squaredSpacing)
{
  if (!cell->isDistant(p, squaredSpacing)) {
    return false;
  }

  for (const auto& neighbour : cell->neighbours) {
    if (!neighbour->isDistant(p, squaredSpacing)) {
      return false;
    }
  }

  return true;
}

bool
SparseGrid::willBeAccepted(const Vector3<double>& p, float& squaredSpacing)
{
  const auto bounds_extent = aabb.extent();
  int nx = (int)(width * (p.x - aabb.min.x) / bounds_extent.x);
  int ny = (int)(height * (p.y - aabb.min.y) / bounds_extent.y);
  int nz = (int)(depth * (p.z - aabb.min.z) / bounds_extent.z);

  int i = std::min(nx, width - 1);
  int j = std::min(ny, height - 1);
  int k = std::min(nz, depth - 1);

  GridIndex index(i, j, k);
  long long key = ((long long)k << 40) | ((long long)j << 20) | (long long)i;
  SparseGrid::iterator it = find(key);
  if (it == end()) {
    it = this->insert(value_type(key, new GridCell(this, index))).first;
  }

  if (isDistant(p, it->second, squaredSpacing)) {
    return true;
  } else {
    return false;
  }
}

bool
SparseGrid::willBeAccepted(const Vector3<double>& p)
{
  const auto bounds_extent = aabb.extent();
  int nx = (int)(width * (p.x - aabb.min.x) / bounds_extent.x);
  int ny = (int)(height * (p.y - aabb.min.y) / bounds_extent.y);
  int nz = (int)(depth * (p.z - aabb.min.z) / bounds_extent.z);

  int i = std::min(nx, width - 1);
  int j = std::min(ny, height - 1);
  int k = std::min(nz, depth - 1);

  GridIndex index(i, j, k);
  long long key = ((long long)k << 40) | ((long long)j << 20) | (long long)i;
  SparseGrid::iterator it = find(key);
  if (it == end()) {
    it = this->insert(value_type(key, new GridCell(this, index))).first;
  }

  if (isDistant(p, it->second)) {
    return true;
  } else {
    return false;
  }
}

bool
SparseGrid::add(const Vector3<double>& p)
{
  const auto bounds_extent = aabb.extent();
  int nx = (int)(width * (p.x - aabb.min.x) / bounds_extent.x);
  int ny = (int)(height * (p.y - aabb.min.y) / bounds_extent.y);
  int nz = (int)(depth * (p.z - aabb.min.z) / bounds_extent.z);

  int i = std::min(nx, width - 1);
  int j = std::min(ny, height - 1);
  int k = std::min(nz, depth - 1);

  GridIndex index(i, j, k);
  long long key = ((long long)k << 40) | ((long long)j << 20) | (long long)i;
  SparseGrid::iterator it = find(key);
  if (it == end()) {
    it = this->insert(value_type(key, new GridCell(this, index))).first;
    // Can't accept this point, an adjacent cell might contain a non-distant point. Pretty sure this
    // is a bug in the original Potree implementation?!
    // it->second->add(p);
    // return true;
  }

  if (isDistant(p, it->second)) {
    it->second->add(p);
    numAccepted++;
    return true;
  } else {
    return false;
  }
}

void
SparseGrid::addWithoutCheck(const Vector3<double>& p)
{
  const auto bounds_extent = aabb.extent();
  int nx = (int)(width * (p.x - aabb.min.x) / bounds_extent.x);
  int ny = (int)(height * (p.y - aabb.min.y) / bounds_extent.y);
  int nz = (int)(depth * (p.z - aabb.min.z) / bounds_extent.z);

  int i = std::min(nx, width - 1);
  int j = std::min(ny, height - 1);
  int k = std::min(nz, depth - 1);

  GridIndex index(i, j, k);
  long long key = ((long long)k << 40) | ((long long)j << 20) | (long long)i;
  SparseGrid::iterator it = find(key);
  if (it == end()) {
    it = this->insert(value_type(key, new GridCell(this, index))).first;
  }

  it->second->add(p);
}

size_t
SparseGrid::content_byte_size() const
{
  // Estimate for the in-memory size of the map itself (keys, values and, since the values are
  // pointers, the dynamically allocated memory for the values)
  const auto map_byte_size = this->size() * (sizeof(long long) + sizeof(void*) + sizeof(GridCell));
  // The memory that each GridCell is referencing dynamically
  const auto cells_byte_size =
    std::accumulate(begin(), end(), size_t{ 0 }, [](auto accum, const auto& kv_pair) {
      return accum + kv_pair.second->content_byte_size();
    });
  return map_byte_size + cells_byte_size;
}