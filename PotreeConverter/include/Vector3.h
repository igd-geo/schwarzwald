
#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include <iostream>

using std::ostream;
#ifndef _MSC_VER
using std::max;
#endif

namespace Potree {

template <class T>
class Vector3 {
 public:
  T x = 0;
  T y = 0;
  T z = 0;

  Vector3() = default;

  Vector3(T x, T y, T z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  Vector3(T value) {
    this->x = value;
    this->y = value;
    this->z = value;
  }

  Vector3(const Vector3<T>& other) : x(other.x), y(other.y), z(other.z) {}

  ~Vector3() = default;

  /// <summary>
  /// Cast the given vector from type T into type U using static_cast
  /// </summary>
  template <typename U>
  static Vector3<U> cast(const Vector3<T>& source) {
    return {static_cast<U>(source.x), static_cast<U>(source.y),
            static_cast<U>(source.z)};
  }

  T length() { return sqrt(x * x + y * y + z * z); }

  T squaredLength() { return x * x + y * y + z * z; }

  T distanceTo(Vector3<T> p) const { return ((*this) - p).length(); }

  T squaredDistanceTo(const Vector3<T>& p) const {
    return ((*this) - p).squaredLength();
  }

  T maxValue() { return max(x, max(y, z)); }

  /// <summary>
  /// Constructs a new vector containing the minimum values in X, Y and Z from
  /// the given two vectors
  /// </summary>
  static Vector3<T> minByAxis(const Vector3<T>& l, const Vector3<T>& r) {
    return {std::min(l.x, r.x), std::min(l.y, r.y), std::min(l.z, r.z)};
  }

  /// <summary>
  /// Constructs a new vector containing the maximum values in X, Y and Z from
  /// the given two vectors
  /// </summary>
  static Vector3<T> maxByAxis(const Vector3<T>& l, const Vector3<T>& r) {
    return {std::max(l.x, r.x), std::max(l.y, r.y), std::max(l.z, r.z)};
  }

  Vector3<T> operator-(const Vector3<T>& right) const {
    return Vector3<T>(x - right.x, y - right.y, z - right.z);
  }

  Vector3<T>& operator-=(const Vector3<T>& right) {
    x -= right.x;
    y -= right.y;
    z -= right.z;
    return *this;
  }

  Vector3<T> operator+(const Vector3<T>& right) const {
    return Vector3<T>(x + right.x, y + right.y, z + right.z);
  }

  Vector3<T>& operator+=(const Vector3<T>& right) {
    x += right.x;
    y += right.y;
    z += right.z;
    return *this;
  }

  Vector3<T> operator+(const T right) const {
    return Vector3<T>(x + right, y + right, z + right);
  }

  Vector3<T> operator/(const T& a) const {
    return Vector3<T>(x / a, y / a, z / a);
  }

  friend ostream& operator<<(ostream& output, const Vector3<T>& value) {
    output << "[" << value.x << ", " << value.y << ", " << value.z << "]";
    return output;
  }
};

}  // namespace Potree

#endif
