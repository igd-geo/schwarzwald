#pragma once

#include "concepts/MemoryIntrospection.h"
#include "math/Vector3.h"

#include <gsl/gsl>
#include <optional>
#include <vector>

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

    PointReference();
    PointReference(const PointReference&) = default;
    PointReference& operator=(const PointReference&) = default;

    Vector3<double>& position() const;
    Vector3<uint8_t>* rgbColor() const;
    Vector3<float>* normal() const;
    uint16_t* intensity() const;
    uint8_t* classification() const;
    uint8_t* edge_of_flight_line() const;
    double* gps_time() const;
    uint8_t* number_of_returns() const;
    uint8_t* return_number() const;
    uint16_t* point_source_id() const;
    uint8_t* scan_direction_flag() const;
    int8_t* scan_angle_rank() const;
    uint8_t* user_data() const;

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

    PointConstReference();
    PointConstReference(const PointReference& point_reference);
    PointConstReference(const PointConstReference&) = default;
    PointConstReference& operator=(const PointConstReference&) = default;

    const Vector3<double>& position() const;
    const Vector3<uint8_t>* rgbColor() const;
    const Vector3<float>* normal() const;
    const uint16_t* intensity() const;
    const uint8_t* classification() const;
    const uint8_t* edge_of_flight_line() const;
    const double* gps_time() const;
    const uint8_t* number_of_returns() const;
    const uint8_t* return_number() const;
    const uint16_t* point_source_id() const;
    const uint8_t* scan_direction_flag() const;
    const int8_t* scan_angle_rank() const;
    const uint8_t* user_data() const;

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
  /// Creates a PointBuffer from the given range of PointReferences
  /// </summary>
  explicit PointBuffer(gsl::span<PointReference> points);
  /// <summary>
  /// Creates a PointBuffer from the given range of PointConstReferences
  /// </summary>
  explicit PointBuffer(gsl::span<PointConstReference> points);

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
              std::vector<uint8_t> classifications = {},
              std::vector<uint8_t> edge_of_flight_lines = {},
              std::vector<double> gps_times = {},
              std::vector<uint8_t> number_of_returns = {},
              std::vector<uint8_t> return_numbers = {},
              std::vector<uint16_t> point_source_ids = {},
              std::vector<uint8_t> scan_direction_flags = {},
              std::vector<int8_t> scan_angle_ranks = {},
              std::vector<uint8_t> user_data = {});

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

  auto& edge_of_flight_lines() { return _edge_of_flight_lines; }
  auto& gps_times() { return _gps_times; }
  auto& number_of_returns() { return _number_of_returns; }
  auto& return_numbers() { return _return_numbers; }
  auto& point_source_ids() { return _point_source_ids; }
  auto& scan_direction_flags() { return _scan_direction_flags; }
  auto& scan_angle_ranks() { return _scan_angle_ranks; }
  auto& user_data() { return _user_data; }

  const std::vector<Vector3<double>>& positions() const { return _positions; }
  const std::vector<Vector3<uint8_t>>& rgbColors() const { return _rgbColors; }
  const std::vector<Vector3<float>>& normals() const { return _normals; }
  const std::vector<uint16_t>& intensities() const { return _intensities; }
  const std::vector<uint8_t>& classifications() const
  {
    return _classifications;
  }

  const auto& edge_of_flight_lines() const { return _edge_of_flight_lines; }
  const auto& gps_times() const { return _gps_times; }
  const auto& number_of_returns() const { return _number_of_returns; }
  const auto& return_numbers() const { return _return_numbers; }
  const auto& point_source_ids() const { return _point_source_ids; }
  const auto& scan_direction_flags() const { return _scan_direction_flags; }
  const auto& scan_angle_ranks() const { return _scan_angle_ranks; }
  const auto& user_data() const { return _user_data; }

  bool hasColors() const;
  bool hasNormals() const;
  bool hasIntensities() const;
  bool hasClassifications() const;
  bool has_edge_of_flight_lines() const;
  bool has_gps_times() const;
  bool has_number_of_returns() const;
  bool has_return_numbers() const;
  bool has_point_source_ids() const;
  bool has_scan_direction_flags() const;
  bool has_scan_angle_ranks() const;
  bool has_user_data() const;

  void verify() const;

  /// <summary>
  /// Returns the raw size in bytes of the contents (positions, normals etc.) of
  /// this PointBuffer. This does NOT include the in-memory size of a
  /// PointBuffer structure itself, but rather the allocated memory of all the
  /// vectors of the PointBuffer
  /// </summary>
  size_t content_byte_size() const;

  struct PointIterator
  {
    PointIterator(PointBuffer& pointBuffer, size_t idx);

    PointReference operator*() const;
    PointIterator& operator++();
    PointIterator operator++(int);
    PointIterator& operator--();
    PointIterator operator--(int);
    PointIterator operator+(std::ptrdiff_t count) const;
    PointIterator operator-(std::ptrdiff_t count) const;
    PointIterator& operator+=(std::ptrdiff_t count);
    PointIterator& operator-=(std::ptrdiff_t count);
    friend std::ptrdiff_t operator-(const PointIterator& l,
                                    const PointIterator& r);
    PointReference operator[](std::ptrdiff_t idx) const;

    bool operator==(const PointIterator& other) const;
    bool operator!=(const PointIterator& other) const;
    friend bool operator<(const PointIterator& l, const PointIterator& r);
    friend bool operator<=(const PointIterator& l, const PointIterator& r);
    friend bool operator>(const PointIterator& l, const PointIterator& r);
    friend bool operator>=(const PointIterator& l, const PointIterator& r);

  private:
    PointBuffer* _pointBuffer;
    size_t _index;
  };

  struct PointConstIterator
  {
    PointConstIterator(PointBuffer const& pointBuffer, size_t idx);

    PointConstReference operator*() const;
    PointConstIterator& operator++();
    PointConstIterator operator++(int);
    PointConstIterator& operator--();
    PointConstIterator operator--(int);
    PointConstIterator operator+(std::ptrdiff_t count) const;
    PointConstIterator operator-(std::ptrdiff_t count) const;
    PointConstIterator& operator+=(std::ptrdiff_t count);
    PointConstIterator& operator-=(std::ptrdiff_t count);
    friend std::ptrdiff_t operator-(const PointConstIterator& l,
                                    const PointConstIterator& r);
    PointConstReference operator[](std::ptrdiff_t idx) const;

    bool operator==(const PointConstIterator& other) const;
    bool operator!=(const PointConstIterator& other) const;
    friend bool operator<(const PointConstIterator& l,
                          const PointConstIterator& r);
    friend bool operator<=(const PointConstIterator& l,
                           const PointConstIterator& r);
    friend bool operator>(const PointConstIterator& l,
                          const PointConstIterator& r);
    friend bool operator>=(const PointConstIterator& l,
                           const PointConstIterator& r);

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

  std::vector<uint8_t> _edge_of_flight_lines;
  std::vector<double> _gps_times;
  std::vector<uint8_t> _number_of_returns;
  std::vector<uint8_t> _return_numbers;
  std::vector<uint16_t> _point_source_ids;
  std::vector<uint8_t> _scan_direction_flags;
  std::vector<int8_t> _scan_angle_ranks;
  std::vector<uint8_t> _user_data;
};

namespace std {
template<>
struct iterator_traits<::PointBuffer::PointIterator>
{
  using difference_type = std::ptrdiff_t;
  using value_type = ::PointBuffer::PointReference;
  using iterator_category = std::random_access_iterator_tag;
};

template<>
struct iterator_traits<::PointBuffer::PointConstIterator>
{
  using difference_type = std::ptrdiff_t;
  using value_type = ::PointBuffer::PointConstReference;
  using iterator_category = std::random_access_iterator_tag;
};
} // namespace std

namespace concepts {

namespace detail {
template<>
constexpr bool has_constant_size_impl<::PointBuffer> = false;
}

template<>
inline unit::byte
size_in_memory(PointBuffer const& point_buffer)
{
  return (sizeof(size_t) * boost::units::information::byte) +
         size_in_memory(point_buffer.positions()) +
         size_in_memory(point_buffer.rgbColors()) +
         size_in_memory(point_buffer.normals()) +
         size_in_memory(point_buffer.intensities()) +
         size_in_memory(point_buffer.classifications());
}
} // namespace concepts