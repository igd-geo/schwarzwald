#include "datastructures/PointBuffer.h"

#include "util/stuff.h"

PointBuffer::PointBuffer()
  : _count(0)
{}

PointBuffer::PointBuffer(size_t count,
                         std::vector<Vector3<double>> positions,
                         std::vector<Vector3<uint8_t>> rgbColors,
                         std::vector<Vector3<float>> normals,
                         std::vector<uint16_t> intensities,
                         std::vector<uint8_t> classifications,
                         std::vector<uint8_t> edge_of_flight_lines,
                         std::vector<double> gps_times,
                         std::vector<uint8_t> number_of_returns,
                         std::vector<uint8_t> return_numbers,
                         std::vector<uint16_t> point_source_ids,
                         std::vector<uint8_t> scan_direction_flags,
                         std::vector<int8_t> scan_angle_ranks,
                         std::vector<uint8_t> user_data)
  : _count(count)
  , _positions(std::move(positions))
  , _rgbColors(std::move(rgbColors))
  , _normals(std::move(normals))
  , _intensities(std::move(intensities))
  , _classifications(std::move(classifications))
  , _edge_of_flight_lines(std::move(edge_of_flight_lines))
  , _gps_times(std::move(gps_times))
  , _number_of_returns(std::move(number_of_returns))
  , _return_numbers(std::move(return_numbers))
  , _point_source_ids(std::move(point_source_ids))
  , _scan_direction_flags(std::move(scan_direction_flags))
  , _scan_angle_ranks(std::move(scan_angle_ranks))
  , _user_data(std::move(user_data))
{
  if (_positions.size() != count) {
    throw std::invalid_argument{ "positions.size() does not equal count!" };
  }
  if (_rgbColors.size() && _rgbColors.size() != count) {
    throw std::invalid_argument{ "rgbColors.size() does not equal count!" };
  }
  if (_normals.size() && _normals.size() != count) {
    throw std::invalid_argument{ "normals.size() does not equal count!" };
  }
  if (_intensities.size() && _intensities.size() != count) {
    throw std::invalid_argument{ "intensities.size() does not equal count!" };
  }
  if (_classifications.size() && _classifications.size() != count) {
    throw std::invalid_argument{ "positions.size() does not equal count!" };
  }
  if (_edge_of_flight_lines.size() && _edge_of_flight_lines.size() != count) {
    throw std::invalid_argument{ "edge_of_flight_lines.size() does not equal count!" };
  }
  if (_gps_times.size() && _gps_times.size() != count) {
    throw std::invalid_argument{ "gps_times.size() does not equal count!" };
  }
  if (_number_of_returns.size() && _number_of_returns.size() != count) {
    throw std::invalid_argument{ "number_of_returns.size() does not equal count!" };
  }
  if (_return_numbers.size() && _return_numbers.size() != count) {
    throw std::invalid_argument{ "return_numbers.size() does not equal count!" };
  }
  if (_point_source_ids.size() && _point_source_ids.size() != count) {
    throw std::invalid_argument{ "point_source_ids.size() does not equal count!" };
  }
  if (_scan_direction_flags.size() && _scan_direction_flags.size() != count) {
    throw std::invalid_argument{ "scan_direction_flags.size() does not equal count!" };
  }
  if (_scan_angle_ranks.size() && _scan_angle_ranks.size() != count) {
    throw std::invalid_argument{ "scan_angle_ranks.size() does not equal count!" };
  }
  if (_user_data.size() && _user_data.size() != count) {
    throw std::invalid_argument{ "user_data.size() does not equal count!" };
  }
}

PointBuffer::PointBuffer(gsl::span<PointReference> points)
  : _count(points.size())
{
  if (!_count)
    return;

  const auto& first_point = points.at(0);

  _positions.reserve(_count);
  if (first_point.rgbColor() != nullptr) {
    _rgbColors.reserve(_count);
  }
  if (first_point.normal() != nullptr) {
    _normals.reserve(_count);
  }
  if (first_point.intensity() != nullptr) {
    _intensities.reserve(_count);
  }
  if (first_point.classification() != nullptr) {
    _classifications.reserve(_count);
  }
  if (first_point.edge_of_flight_line() != nullptr) {
    _edge_of_flight_lines.reserve(_count);
  }
  if (first_point.gps_time() != nullptr) {
    _gps_times.reserve(_count);
  }
  if (first_point.number_of_returns() != nullptr) {
    _number_of_returns.reserve(_count);
  }
  if (first_point.point_source_id() != nullptr) {
    _point_source_ids.reserve(_count);
  }
  if (first_point.return_number() != nullptr) {
    _return_numbers.reserve(_count);
  }
  if (first_point.scan_angle_rank() != nullptr) {
    _scan_angle_ranks.reserve(_count);
  }
  if (first_point.scan_direction_flag() != nullptr) {
    _scan_direction_flags.reserve(_count);
  }
  if (first_point.user_data() != nullptr) {
    _user_data.reserve(_count);
  }

  for (auto& point : points) {
    _positions.push_back(point.position());
    if (point.rgbColor()) {
      _rgbColors.push_back(*point.rgbColor());
    }
    if (point.normal()) {
      _normals.push_back(*point.normal());
    }
    if (point.intensity()) {
      _intensities.push_back(*point.intensity());
    }
    if (point.classification()) {
      _classifications.push_back(*point.classification());
    }
    if (point.edge_of_flight_line()) {
      _edge_of_flight_lines.push_back(*point.edge_of_flight_line());
    }
    if (point.gps_time()) {
      _gps_times.push_back(*point.gps_time());
    }
    if (point.number_of_returns()) {
      _number_of_returns.push_back(*point.number_of_returns());
    }
    if (point.point_source_id()) {
      _point_source_ids.push_back(*point.point_source_id());
    }
    if (point.return_number()) {
      _return_numbers.push_back(*point.return_number());
    }
    if (point.scan_angle_rank()) {
      _scan_angle_ranks.push_back(*point.scan_angle_rank());
    }
    if (point.scan_direction_flag()) {
      _scan_direction_flags.push_back(*point.scan_direction_flag());
    }
    if (point.user_data()) {
      _user_data.push_back(*point.user_data());
    }
  }
}

PointBuffer::PointBuffer(gsl::span<PointConstReference> points)
  : _count(points.size())
{
  if (!_count)
    return;

  const auto& first_point = points.at(0);

  _positions.reserve(_count);
  if (first_point.rgbColor() != nullptr) {
    _rgbColors.reserve(_count);
  }
  if (first_point.normal() != nullptr) {
    _normals.reserve(_count);
  }
  if (first_point.intensity() != nullptr) {
    _intensities.reserve(_count);
  }
  if (first_point.classification() != nullptr) {
    _classifications.reserve(_count);
  }
  if (first_point.edge_of_flight_line() != nullptr) {
    _edge_of_flight_lines.reserve(_count);
  }
  if (first_point.gps_time() != nullptr) {
    _gps_times.reserve(_count);
  }
  if (first_point.number_of_returns() != nullptr) {
    _number_of_returns.reserve(_count);
  }
  if (first_point.point_source_id() != nullptr) {
    _point_source_ids.reserve(_count);
  }
  if (first_point.return_number() != nullptr) {
    _return_numbers.reserve(_count);
  }
  if (first_point.scan_angle_rank() != nullptr) {
    _scan_angle_ranks.reserve(_count);
  }
  if (first_point.scan_direction_flag() != nullptr) {
    _scan_direction_flags.reserve(_count);
  }
  if (first_point.user_data() != nullptr) {
    _user_data.reserve(_count);
  }

  for (auto& point : points) {
    _positions.push_back(point.position());
    if (point.rgbColor()) {
      _rgbColors.push_back(*point.rgbColor());
    }
    if (point.normal()) {
      _normals.push_back(*point.normal());
    }
    if (point.intensity()) {
      _intensities.push_back(*point.intensity());
    }
    if (point.classification()) {
      _classifications.push_back(*point.classification());
    }
    if (point.edge_of_flight_line()) {
      _edge_of_flight_lines.push_back(*point.edge_of_flight_line());
    }
    if (point.gps_time()) {
      _gps_times.push_back(*point.gps_time());
    }
    if (point.number_of_returns()) {
      _number_of_returns.push_back(*point.number_of_returns());
    }
    if (point.point_source_id()) {
      _point_source_ids.push_back(*point.point_source_id());
    }
    if (point.return_number()) {
      _return_numbers.push_back(*point.return_number());
    }
    if (point.scan_angle_rank()) {
      _scan_angle_ranks.push_back(*point.scan_angle_rank());
    }
    if (point.scan_direction_flag()) {
      _scan_direction_flags.push_back(*point.scan_direction_flag());
    }
    if (point.user_data()) {
      _user_data.push_back(*point.user_data());
    }
  }
}

void
PointBuffer::push_point(PointConstReference point)
{
  _positions.push_back(point.position());
  if (point.rgbColor()) {
    _rgbColors.push_back(*point.rgbColor());
  }
  if (point.normal()) {
    _normals.push_back(*point.normal());
  }
  if (point.intensity()) {
    _intensities.push_back(*point.intensity());
  }
  if (point.classification()) {
    _classifications.push_back(*point.classification());
  }
  ++_count;
}

PointBuffer::PointConstReference
PointBuffer::get_point(size_t point_index) const
{
  return { this, point_index };
}

PointBuffer::PointReference
PointBuffer::get_point(size_t point_index)
{
  return { this, point_index };
}

void
PointBuffer::append_buffer(const PointBuffer& other)
{
  _positions.insert(_positions.end(), other.positions().begin(), other.positions().end());

  const auto appendAttributes = [](auto& sourceAttributeContainer,
                                   const auto& targetAttributeContainer,
                                   auto sourceHasAttribute,
                                   auto targetHasAttribute,
                                   auto sourcePointSize,
                                   auto targetPointSize) {
    if (targetHasAttribute) {
      if (!sourceHasAttribute) {
        sourceAttributeContainer.resize(sourcePointSize);
      }

      sourceAttributeContainer.insert(sourceAttributeContainer.end(),
                                      targetAttributeContainer.begin(),
                                      targetAttributeContainer.end());
    } else if (sourceHasAttribute) {
      sourceAttributeContainer.resize(sourceAttributeContainer.size() + targetPointSize);
    }
  };

  appendAttributes(
    _rgbColors, other.rgbColors(), hasColors(), other.hasColors(), count(), other.count());
  appendAttributes(
    _normals, other.normals(), hasNormals(), other.hasNormals(), count(), other.count());
  appendAttributes(_intensities,
                   other.intensities(),
                   hasIntensities(),
                   other.hasIntensities(),
                   count(),
                   other.count());
  appendAttributes(_classifications,
                   other.classifications(),
                   hasClassifications(),
                   other.hasClassifications(),
                   count(),
                   other.count());
  appendAttributes(_edge_of_flight_lines,
                   other.edge_of_flight_lines(),
                   has_edge_of_flight_lines(),
                   other.has_edge_of_flight_lines(),
                   count(),
                   other.count());
  appendAttributes(
    _gps_times, other.gps_times(), has_gps_times(), other.has_gps_times(), count(), other.count());
  appendAttributes(_number_of_returns,
                   other.number_of_returns(),
                   has_number_of_returns(),
                   other.has_number_of_returns(),
                   count(),
                   other.count());
  appendAttributes(_return_numbers,
                   other.return_numbers(),
                   has_return_numbers(),
                   other.has_return_numbers(),
                   count(),
                   other.count());
  appendAttributes(_point_source_ids,
                   other.point_source_ids(),
                   has_point_source_ids(),
                   other.has_point_source_ids(),
                   count(),
                   other.count());
  appendAttributes(_scan_direction_flags,
                   other.scan_direction_flags(),
                   has_scan_direction_flags(),
                   other.has_scan_direction_flags(),
                   count(),
                   other.count());
  appendAttributes(_scan_angle_ranks,
                   other.scan_angle_ranks(),
                   has_scan_angle_ranks(),
                   other.has_scan_angle_ranks(),
                   count(),
                   other.count());
  appendAttributes(
    _user_data, other.user_data(), has_user_data(), other.has_user_data(), count(), other.count());

  _count += other.count();
}

void
PointBuffer::clear()
{
  _count = 0;
  _positions.clear();
  _positions.shrink_to_fit();
  _rgbColors.clear();
  _rgbColors.shrink_to_fit();
  _normals.clear();
  _normals.shrink_to_fit();
  _intensities.clear();
  _intensities.shrink_to_fit();
  _classifications.clear();
  _classifications.shrink_to_fit();
}

bool
PointBuffer::hasColors() const
{
  return !_rgbColors.empty();
}

bool
PointBuffer::hasNormals() const
{
  return !_normals.empty();
}

bool
PointBuffer::hasIntensities() const
{
  return !_intensities.empty();
}

bool
PointBuffer::hasClassifications() const
{
  return !_classifications.empty();
}

bool
PointBuffer::has_edge_of_flight_lines() const
{
  return !_edge_of_flight_lines.empty();
}
bool
PointBuffer::has_gps_times() const
{
  return !_gps_times.empty();
}
bool
PointBuffer::has_number_of_returns() const
{
  return !_number_of_returns.empty();
}
bool
PointBuffer::has_return_numbers() const
{
  return !_return_numbers.empty();
}
bool
PointBuffer::has_point_source_ids() const
{
  return !_point_source_ids.empty();
}
bool
PointBuffer::has_scan_direction_flags() const
{
  return !_scan_direction_flags.empty();
}
bool
PointBuffer::has_scan_angle_ranks() const
{
  return !_scan_angle_ranks.empty();
}
bool
PointBuffer::has_user_data() const
{
  return !_user_data.empty();
}

void
PointBuffer::verify() const
{
  if (_positions.size() != _count) {
    throw std::invalid_argument{ "positions.size() does not equal count!" };
  }
  if (_rgbColors.size() && _rgbColors.size() != _count) {
    throw std::invalid_argument{ "rgbColors.size() does not equal count!" };
  }
  if (_normals.size() && _normals.size() != _count) {
    throw std::invalid_argument{ "normals.size() does not equal count!" };
  }
  if (_intensities.size() && _intensities.size() != _count) {
    throw std::invalid_argument{ "intensities.size() does not equal count!" };
  }
  if (_classifications.size() && _classifications.size() != _count) {
    throw std::invalid_argument{ "positions.size() does not equal count!" };
  }
  if (_edge_of_flight_lines.size() && _edge_of_flight_lines.size() != _count) {
    throw std::invalid_argument{ "edge_of_flight_lines.size() does not equal count!" };
  }
  if (_gps_times.size() && _gps_times.size() != _count) {
    throw std::invalid_argument{ "gps_times.size() does not equal count!" };
  }
  if (_number_of_returns.size() && _number_of_returns.size() != _count) {
    throw std::invalid_argument{ "number_of_returns.size() does not equal count!" };
  }
  if (_return_numbers.size() && _return_numbers.size() != _count) {
    throw std::invalid_argument{ "return_numbers.size() does not equal count!" };
  }
  if (_point_source_ids.size() && _point_source_ids.size() != _count) {
    throw std::invalid_argument{ "point_source_ids.size() does not equal count!" };
  }
  if (_scan_direction_flags.size() && _scan_direction_flags.size() != _count) {
    throw std::invalid_argument{ "scan_direction_flags.size() does not equal count!" };
  }
  if (_scan_angle_ranks.size() && _scan_angle_ranks.size() != _count) {
    throw std::invalid_argument{ "scan_angle_ranks.size() does not equal count!" };
  }
  if (_user_data.size() && _user_data.size() != _count) {
    throw std::invalid_argument{ "user_data.size() does not equal count!" };
  }
}

size_t
PointBuffer::content_byte_size() const
{
  return vector_byte_size(_positions) + vector_byte_size(_rgbColors) + vector_byte_size(_normals) +
         vector_byte_size(_intensities) + vector_byte_size(_classifications) +
         vector_byte_size(_edge_of_flight_lines) + vector_byte_size(_gps_times) +
         vector_byte_size(_number_of_returns) + vector_byte_size(_return_numbers) +
         vector_byte_size(_point_source_ids) + vector_byte_size(_scan_angle_ranks) +
         vector_byte_size(_scan_direction_flags) + vector_byte_size(_user_data);
}

PointBuffer::PointIterator
PointBuffer::begin()
{
  return PointBuffer::PointIterator{ *this, 0 };
}

PointBuffer::PointIterator
PointBuffer::end()
{
  return PointBuffer::PointIterator{ *this, count() };
}

PointBuffer::PointConstIterator
PointBuffer::begin() const
{
  return PointBuffer::PointConstIterator{ *this, 0 };
}
PointBuffer::PointConstIterator
PointBuffer::end() const
{
  return PointBuffer::PointConstIterator{ *this, count() };
}

#pragma region PointConstReference
const Vector3<double>&
PointBuffer::PointConstReference::position() const
{
  return _pointBuffer->positions()[_index];
}

const Vector3<uint8_t>*
PointBuffer::PointConstReference::rgbColor() const
{
  if (!_pointBuffer->hasColors())
    return nullptr;
  return _pointBuffer->rgbColors().data() + _index;
}

const Vector3<float>*
PointBuffer::PointConstReference::normal() const
{
  if (!_pointBuffer->hasNormals())
    return nullptr;
  return _pointBuffer->normals().data() + _index;
}

const uint16_t*
PointBuffer::PointConstReference::intensity() const
{
  if (!_pointBuffer->hasIntensities())
    return nullptr;
  return _pointBuffer->intensities().data() + _index;
}

const uint8_t*
PointBuffer::PointConstReference::classification() const
{
  if (!_pointBuffer->hasClassifications())
    return nullptr;
  return _pointBuffer->classifications().data() + _index;
}

const uint8_t*
PointBuffer::PointConstReference::edge_of_flight_line() const
{
  if (!_pointBuffer->has_edge_of_flight_lines())
    return nullptr;
  return _pointBuffer->edge_of_flight_lines().data() + _index;
}
const double*
PointBuffer::PointConstReference::gps_time() const
{
  if (!_pointBuffer->has_gps_times())
    return nullptr;
  return _pointBuffer->gps_times().data() + _index;
}
const uint8_t*
PointBuffer::PointConstReference::number_of_returns() const
{
  if (!_pointBuffer->has_number_of_returns())
    return nullptr;
  return _pointBuffer->number_of_returns().data() + _index;
}

const uint8_t*
PointBuffer::PointConstReference::return_number() const
{
  if (!_pointBuffer->has_return_numbers())
    return nullptr;
  return _pointBuffer->return_numbers().data() + _index;
}

const uint16_t*
PointBuffer::PointConstReference::point_source_id() const
{
  if (!_pointBuffer->has_point_source_ids())
    return nullptr;
  return _pointBuffer->point_source_ids().data() + _index;
}
const uint8_t*
PointBuffer::PointConstReference::scan_direction_flag() const
{
  if (!_pointBuffer->has_scan_direction_flags())
    return nullptr;
  return _pointBuffer->scan_direction_flags().data() + _index;
}
const int8_t*
PointBuffer::PointConstReference::scan_angle_rank() const
{
  if (!_pointBuffer->has_scan_angle_ranks())
    return nullptr;
  return _pointBuffer->scan_angle_ranks().data() + _index;
}
const uint8_t*
PointBuffer::PointConstReference::user_data() const
{
  if (!_pointBuffer->has_user_data())
    return nullptr;
  return _pointBuffer->user_data().data() + _index;
}

PointBuffer::PointConstReference::PointConstReference()
  : _pointBuffer(nullptr)
  , _index(0)
{}

PointBuffer::PointConstReference::PointConstReference(
  const PointBuffer::PointReference& point_reference)
  : _pointBuffer(point_reference._pointBuffer)
  , _index(point_reference._index)
{}

PointBuffer::PointConstReference::PointConstReference(PointBuffer const* pointBuffer, size_t index)
  : _pointBuffer(pointBuffer)
  , _index(index)
{}
#pragma endregion

#pragma region PointReference
Vector3<double>&
PointBuffer::PointReference::position() const
{
  return _pointBuffer->positions()[_index];
}

Vector3<uint8_t>*
PointBuffer::PointReference::rgbColor() const
{
  if (!_pointBuffer->hasColors())
    return nullptr;
  return _pointBuffer->rgbColors().data() + _index;
}

Vector3<float>*
PointBuffer::PointReference::normal() const
{
  if (!_pointBuffer->hasNormals())
    return nullptr;
  return _pointBuffer->normals().data() + _index;
}

uint16_t*
PointBuffer::PointReference::intensity() const
{
  if (!_pointBuffer->hasIntensities())
    return nullptr;
  return _pointBuffer->intensities().data() + _index;
}

uint8_t*
PointBuffer::PointReference::classification() const
{
  if (!_pointBuffer->hasClassifications())
    return nullptr;
  return _pointBuffer->classifications().data() + _index;
}

uint8_t*
PointBuffer::PointReference::edge_of_flight_line() const
{
  if (!_pointBuffer->has_edge_of_flight_lines())
    return nullptr;
  return _pointBuffer->edge_of_flight_lines().data() + _index;
}
double*
PointBuffer::PointReference::gps_time() const
{
  if (!_pointBuffer->has_gps_times())
    return nullptr;
  return _pointBuffer->gps_times().data() + _index;
}
uint8_t*
PointBuffer::PointReference::number_of_returns() const
{
  if (!_pointBuffer->has_number_of_returns())
    return nullptr;
  return _pointBuffer->number_of_returns().data() + _index;
}

uint8_t*
PointBuffer::PointReference::return_number() const
{
  if (!_pointBuffer->has_return_numbers())
    return nullptr;
  return _pointBuffer->return_numbers().data() + _index;
}

uint16_t*
PointBuffer::PointReference::point_source_id() const
{
  if (!_pointBuffer->has_point_source_ids())
    return nullptr;
  return _pointBuffer->point_source_ids().data() + _index;
}
uint8_t*
PointBuffer::PointReference::scan_direction_flag() const
{
  if (!_pointBuffer->has_scan_direction_flags())
    return nullptr;
  return _pointBuffer->scan_direction_flags().data() + _index;
}
int8_t*
PointBuffer::PointReference::scan_angle_rank() const
{
  if (!_pointBuffer->has_scan_angle_ranks())
    return nullptr;
  return _pointBuffer->scan_angle_ranks().data() + _index;
}
uint8_t*
PointBuffer::PointReference::user_data() const
{
  if (!_pointBuffer->has_user_data())
    return nullptr;
  return _pointBuffer->user_data().data() + _index;
}

PointBuffer::PointReference::PointReference()
  : _pointBuffer(nullptr)
  , _index(0)
{}

PointBuffer::PointReference::PointReference(PointBuffer* pointBuffer, size_t index)
  : _pointBuffer(pointBuffer)
  , _index(index)
{}
#pragma endregion

#pragma region PointConstIterator
PointBuffer::PointConstIterator::PointConstIterator(const PointBuffer& pointBuffer, size_t idx)
  : _pointBuffer(&pointBuffer)
  , _index(idx)
{}

PointBuffer::PointConstReference PointBuffer::PointConstIterator::operator*() const
{
  return PointBuffer::PointConstReference{ _pointBuffer, _index };
}

PointBuffer::PointConstIterator&
PointBuffer::PointConstIterator::operator++()
{
  ++_index;
  return *this;
}

bool
PointBuffer::PointConstIterator::operator==(const PointConstIterator& other) const
{
  return _pointBuffer == other._pointBuffer && _index == other._index;
}

bool
PointBuffer::PointConstIterator::operator!=(const PointConstIterator& other) const
{
  return _pointBuffer != other._pointBuffer || _index != other._index;
}

PointBuffer::PointConstIterator
PointBuffer::PointConstIterator::operator+(std::ptrdiff_t idx) const
{
  return { *_pointBuffer, _index + idx };
}

PointBuffer::PointConstIterator
PointBuffer::PointConstIterator::operator++(int)
{
  auto iter = *this;
  ++(*this);
  return iter;
}

PointBuffer::PointConstIterator&
PointBuffer::PointConstIterator::operator--()
{
  --_index;
  return *this;
}

PointBuffer::PointConstIterator
PointBuffer::PointConstIterator::operator--(int)
{
  auto iter = *this;
  --(*this);
  return iter;
}

PointBuffer::PointConstIterator
PointBuffer::PointConstIterator::operator-(std::ptrdiff_t count) const
{
  return { *_pointBuffer, static_cast<size_t>(_index - count) };
}

PointBuffer::PointConstIterator&
PointBuffer::PointConstIterator::operator+=(std::ptrdiff_t count)
{
  _index = static_cast<size_t>(_index + count);
  return *this;
}

PointBuffer::PointConstIterator&
PointBuffer::PointConstIterator::operator-=(std::ptrdiff_t count)
{
  _index = static_cast<size_t>(_index - count);
  return *this;
}

std::ptrdiff_t
operator-(const PointBuffer::PointConstIterator& l, const PointBuffer::PointConstIterator& r)
{
  return l._index - r._index;
}

PointBuffer::PointConstReference PointBuffer::PointConstIterator::operator[](
  std::ptrdiff_t idx) const
{
  return PointConstReference{ _pointBuffer, static_cast<size_t>(_index + idx) };
}

bool
operator<(const PointBuffer::PointConstIterator& l, const PointBuffer::PointConstIterator& r)
{
  return l._index < r._index;
}
bool
operator<=(const PointBuffer::PointConstIterator& l, const PointBuffer::PointConstIterator& r)
{
  return l._index <= r._index;
}
bool
operator>(const PointBuffer::PointConstIterator& l, const PointBuffer::PointConstIterator& r)
{
  return l._index > r._index;
  ;
}
bool
operator>=(const PointBuffer::PointConstIterator& l, const PointBuffer::PointConstIterator& r)
{
  return l._index >= r._index;
}

#pragma endregion

#pragma region PointIterator
PointBuffer::PointIterator::PointIterator(PointBuffer& pointBuffer, size_t idx)
  : _pointBuffer(&pointBuffer)
  , _index(idx)
{}

PointBuffer::PointReference PointBuffer::PointIterator::operator*() const
{
  return PointBuffer::PointReference{ _pointBuffer, _index };
}

PointBuffer::PointIterator&
PointBuffer::PointIterator::operator++()
{
  ++_index;
  return *this;
}

bool
PointBuffer::PointIterator::operator==(const PointIterator& other) const
{
  return _pointBuffer == other._pointBuffer && _index == other._index;
}

bool
PointBuffer::PointIterator::operator!=(const PointIterator& other) const
{
  return _pointBuffer != other._pointBuffer || _index != other._index;
}

PointBuffer::PointIterator
PointBuffer::PointIterator::operator+(std::ptrdiff_t idx) const
{
  return { *_pointBuffer, static_cast<size_t>(_index + idx) };
}

PointBuffer::PointIterator
PointBuffer::PointIterator::operator++(int)
{
  auto iter = *this;
  ++(*this);
  return iter;
}

PointBuffer::PointIterator&
PointBuffer::PointIterator::operator--()
{
  --_index;
  return *this;
}

PointBuffer::PointIterator
PointBuffer::PointIterator::operator--(int)
{
  auto iter = *this;
  --(*this);
  return iter;
}

PointBuffer::PointIterator
PointBuffer::PointIterator::operator-(std::ptrdiff_t count) const
{
  return { *_pointBuffer, static_cast<size_t>(_index - count) };
}

PointBuffer::PointIterator&
PointBuffer::PointIterator::operator+=(std::ptrdiff_t count)
{
  _index = static_cast<size_t>(_index + count);
  return *this;
}

PointBuffer::PointIterator&
PointBuffer::PointIterator::operator-=(std::ptrdiff_t count)
{
  _index = static_cast<size_t>(_index - count);
  return *this;
}

std::ptrdiff_t
operator-(const PointBuffer::PointIterator& l, const PointBuffer::PointIterator& r)
{
  return l._index - r._index;
}

PointBuffer::PointReference PointBuffer::PointIterator::operator[](std::ptrdiff_t idx) const
{
  return PointReference{ _pointBuffer, static_cast<size_t>(_index + idx) };
}

bool
operator<(const PointBuffer::PointIterator& l, const PointBuffer::PointIterator& r)
{
  return l._index < r._index;
}
bool
operator<=(const PointBuffer::PointIterator& l, const PointBuffer::PointIterator& r)
{
  return l._index <= r._index;
}
bool
operator>(const PointBuffer::PointIterator& l, const PointBuffer::PointIterator& r)
{
  return l._index > r._index;
  ;
}
bool
operator>=(const PointBuffer::PointIterator& l, const PointBuffer::PointIterator& r)
{
  return l._index >= r._index;
}

#pragma endregion
