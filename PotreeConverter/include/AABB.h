

#ifndef AABB_H
#define AABB_H

#include <algorithm>
#include <math.h>

#include "Vector3.h"

using std::endl;
using std::max;
using std::min;

namespace Potree {

// Axis Aligned Bounding Box
class AABB
{

public:
  Vector3<double> min;
  Vector3<double> max;
  Vector3<double> size;

  AABB()
  {
    min = Vector3<double>(std::numeric_limits<float>::max());
    max = Vector3<double>(-std::numeric_limits<float>::max());
    size = Vector3<double>(std::numeric_limits<float>::max());
  }

  AABB(Vector3<double> min, Vector3<double> max)
  {
    this->min = min;
    this->max = max;
    size = max - min;
  }

  bool isInside(const Vector3<double>& p) const
  {
    if (min.x <= p.x && p.x <= max.x) {
      if (min.y <= p.y && p.y <= max.y) {
        if (min.z <= p.z && p.z <= max.z) {
          return true;
        }
      }
    }

    return false;
  }

  void update(const Vector3<double>& point)
  {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);

    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);

    size = max - min;
  }

  void update(const AABB& aabb)
  {
    update(aabb.min);
    update(aabb.max);
  }

  void makeCubic()
  {
    max = min + size.maxValue();
    size = max - min;
  }

  // not tested
  Vector3<double> getCenter() const
  {
    Vector3<double> center;
    center = min + size / 2;
    return center;
  }

  bool operator==(const AABB& other) const
  {
    return min == other.min && max == other.max && size == other.size;
  }

  friend ostream& operator<<(ostream& output, const AABB& value)
  {
    output << "min: " << value.min << endl;
    output << "max: " << value.max << endl;
    output << "size: " << value.size << endl;
    return output;
  }
};
}

#endif