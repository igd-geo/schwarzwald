#pragma once

#include <algorithm>
#include <math.h>

#include "concepts/MemoryIntrospection.h"
#include "math/Vector3.h"

// Axis Aligned Bounding Box
struct AABB
{
  Vector3<double> min;
  Vector3<double> max;

  AABB()
    : min(std::numeric_limits<double>::max())
    , max(-std::numeric_limits<double>::max())
  {}

  AABB(Vector3<double> min, Vector3<double> max)
    : min(min)
    , max(max)
  {}

  Vector3<double> extent() const { return max - min; }

  bool isInside(const Vector3<double>& p) const
  {
    return (p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z &&
            p.z <= max.z);
  }

  void update(const Vector3<double>& point)
  {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);

    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
  }

  void update(const AABB& aabb)
  {
    update(aabb.min);
    update(aabb.max);
  }

  void makeCubic()
  {
    const auto max_extent = extent().maxValue();
    const auto half_length = max_extent / 2;
    const auto center = getCenter();
    min = { center.x - half_length, center.y - half_length, center.z - half_length };
    max = { center.x + half_length, center.y + half_length, center.z + half_length };
  }

  AABB cubic() const
  {
    auto bounds = *this;
    bounds.makeCubic();
    return bounds;
  }

  Vector3<double> getCenter() const { return min + extent() / 2; }

  AABB translate(const Vector3<double>& translation) const
  {
    return { min + translation, max + translation };
  }

  bool operator==(const AABB& other) const { return min == other.min && max == other.max; }

  friend std::ostream& operator<<(std::ostream& output, const AABB& value)
  {
    output << "min: " << value.min << " max: " << value.max;
    return output;
  }
};

namespace concepts {

template<>
inline unit::byte
size_in_memory(AABB const&)
{
  return 2 * sizeof(Vector3<double>) * boost::units::information::byte;
}
} // namespace concepts