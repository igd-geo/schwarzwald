#pragma once

#include "Vector3.h"

#include <gsl/gsl>
#include <optional>
#include <vector>

namespace Potree {

// TODO Make attributes more dynamic (map<AttributeType, GenericAttribute*> or
// something like that)

/// <summary>
/// Buffer structure that stores point attributes (position, color etc.) for
/// multiple points at once in a structure-of-array fashion. Compared to storing
/// all points as Point structures, this has better performance for data access
/// </summary>
struct PointBuffer
{
  struct PointConstReference;

  /// <summary>
  /// Mutable indirect reference to a single point inside the PointBuffer
  /// </summary>
  struct PointReference
  {
    friend struct PointBuffer;
    friend struct PointConstReference;

    PointReference(const PointReference&) = default;
    PointReference& operator=(const PointReference&) = default;

    Vector3<double>& position() const;
    Vector3<uint8_t>* rgbColor() const;
    Vector3<float>* normal() const;
    uint16_t* intensity() const;
    uint8_t* classification() const;

  private:
    PointReference(PointBuffer* pointBuffer, size_t index);

    PointBuffer* _pointBuffer;
    size_t _index;
  };

  /// <summary>
  /// Constant indirect reference to a single point inside the PointBuffer
  /// </summary>
  struct PointConstReference
  {
    friend struct PointBuffer;

    PointConstReference(const PointReference& point_reference);

    PointConstReference(const PointConstReference&) = default;
    PointConstReference& operator=(const PointConstReference&) = default;

    const Vector3<double>& position() const;
    const Vector3<uint8_t>* rgbColor() const;
    const Vector3<float>* normal() const;
    const uint16_t* intensity() const;
    const uint8_t* classification() const;

  private:
    PointConstReference(PointBuffer const* pointBuffer, size_t index);

    PointBuffer const* _pointBuffer;
    size_t _index;
  };

  /// <summary>
  /// Creates an empty point buffer
  /// </summary>
  PointBuffer();

  /// <summary>
  /// Creates a new PointBuffer storing count points. For all passed attribute
  /// vectors, their count has to be equal to the specified count, otherwise a
  /// invalid_argument error is thrown
  /// </summary>
  PointBuffer(size_t count,
              std::vector<Vector3<double>> positions,
              std::vector<Vector3<uint8_t>> rgbColors = {},
              std::vector<Vector3<float>> normals = {},
              std::vector<uint16_t> intensities = {},
              std::vector<uint8_t> classifications = {});

  PointBuffer(const PointBuffer&) = default;
  PointBuffer(PointBuffer&&) = default;

  PointBuffer& operator=(const PointBuffer&) = default;
  PointBuffer& operator=(PointBuffer&&) = default;

  /// <summary>
  /// Push a single point into this PointBuffer. If the point has attributes
  /// that are not defined in this PointBuffer they are ignored. Attributes from
  /// this PointBuffer that are not defined in the given point are filled with
  /// default values
  /// </summary>
  void push_point(PointConstReference point);

  /// <summary>
  /// Push a range of point attributes into this PointBuffer. Throws an
  /// invalid_argument exception if any two spans passed into this function have
  /// different, non-zero sizes
  /// </summary>
  void push_points(gsl::span<Vector3<double>> positions,
                   gsl::span<Vector3<uint8_t>> rgbColors = {},
                   gsl::span<Vector3<float>> normals = {},
                   gsl::span<uint16_t> intensities = {},
                   gsl::span<uint8_t> classifications = {});

  /// <summary>
  /// Returns a constant indirect reference to the point with the given index
  /// </summary>
  PointConstReference get_point(size_t point_index) const;

  /// <summary>
  /// Returns a mutable indirect reference to the point with the given index
  /// </summary>
  PointReference get_point(size_t point_index);

  /// <summary>
  /// Appends the contents of the given PointBuffer to this PointBuffer. This
  /// will copy all attributes that exist from the given buffer into this
  /// buffer. Attributes that exist in this buffer but not in the other buffer
  /// are filled with default values
  /// </summary>
  void append_buffer(const PointBuffer& other);

  size_t count() const { return _count; }
  bool empty() const { return _count == 0; }
  void clear();

  std::vector<Vector3<double>>& positions() { return _positions; }
  std::vector<Vector3<uint8_t>>& rgbColors() { return _rgbColors; }
  std::vector<Vector3<float>>& normals() { return _normals; }
  std::vector<uint16_t>& intensities() { return _intensities; }
  std::vector<uint8_t>& classifications() { return _classifications; }

  const std::vector<Vector3<double>>& positions() const { return _positions; }
  const std::vector<Vector3<uint8_t>>& rgbColors() const { return _rgbColors; }
  const std::vector<Vector3<float>>& normals() const { return _normals; }
  const std::vector<uint16_t>& intensities() const { return _intensities; }
  const std::vector<uint8_t>& classifications() const { return _classifications; }

  bool hasColors() const;
  bool hasNormals() const;
  bool hasIntensities() const;
  bool hasClassifications() const;

  void verify() const;

  /// <summary>
  /// Returns the raw size in bytes of the contents (positions, normals etc.) of this PointBuffer.
  /// This does NOT include the in-memory size of a PointBuffer structure itself, but rather the
  /// allocated memory of all the vectors of the PointBuffer
  /// </summary>
  size_t content_byte_size() const;

  struct PointIterator
  {
    PointIterator(PointBuffer& pointBuffer, size_t idx);

    PointReference operator*() const;
    PointIterator& operator++();
    PointIterator operator+(size_t idx) const;

    bool operator==(const PointIterator& other) const;
    bool operator!=(const PointIterator& other) const;

  private:
    PointBuffer* _pointBuffer;
    size_t _index;
  };

  struct PointConstIterator
  {
    PointConstIterator(PointBuffer const& pointBuffer, size_t idx);

    PointConstReference operator*() const;
    PointConstIterator& operator++();
    PointConstIterator operator+(size_t idx) const;

    bool operator==(const PointConstIterator& other) const;
    bool operator!=(const PointConstIterator& other) const;

  private:
    PointBuffer const* _pointBuffer;
    size_t _index;
  };

  PointIterator begin();
  PointIterator end();

  PointConstIterator begin() const;
  PointConstIterator end() const;

private:
  size_t _count;
  std::vector<Vector3<double>> _positions;
  std::vector<Vector3<uint8_t>> _rgbColors;
  std::vector<Vector3<float>> _normals;
  std::vector<uint16_t> _intensities;
  std::vector<uint8_t> _classifications;
};

} // namespace Potree
