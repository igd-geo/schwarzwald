#include "PointBuffer.h"

#include "stuff.h"

Potree::PointBuffer::PointBuffer()
  : _count(0)
{}

Potree::PointBuffer::PointBuffer(size_t count,
                                 std::vector<Vector3<double>> positions,
                                 std::vector<Vector3<uint8_t>> rgbColors,
                                 std::vector<Vector3<float>> normals,
                                 std::vector<uint16_t> intensities,
                                 std::vector<uint8_t> classifications)
  : _count(count)
  , _positions(std::move(positions))
  , _rgbColors(std::move(rgbColors))
  , _normals(std::move(normals))
  , _intensities(std::move(intensities))
  , _classifications(std::move(classifications))
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
}

void
Potree::PointBuffer::push_point(PointConstReference point)
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

void
Potree::PointBuffer::push_points(gsl::span<Vector3<double>> newPositions,
                                 gsl::span<Vector3<uint8_t>> newRgbColors,
                                 gsl::span<Vector3<float>> newNormals,
                                 gsl::span<uint16_t> newIntensities,
                                 gsl::span<uint8_t> newClassifications)
{
  const auto newPointsSize = newPositions.size();
  if (newRgbColors.size() > 0 && newRgbColors.size() != newPointsSize) {
    throw new std::invalid_argument{ "rgbColors.size() does not equal positions.size()!" };
  }
  if (newNormals.size() > 0 && newNormals.size() != newPointsSize) {
    throw new std::invalid_argument{ "normals.size() does not equal positions.size()!" };
  }
  if (newIntensities.size() > 0 && newIntensities.size() != newPointsSize) {
    throw new std::invalid_argument{ "intensities.size() does not equal positions.size()!" };
  }
  if (newClassifications.size() > 0 && newClassifications.size() != newPointsSize) {
    throw new std::invalid_argument{ "classifications.size() does not equal positions.size()!" };
  }

  _positions.resize(_positions.size() + newPointsSize);
  std::copy(newPositions.begin(), newPositions.end(), _positions.begin() + _count);

  if (newRgbColors.size() > 0) {
    _rgbColors.resize(_rgbColors.size() + newPointsSize);
    std::copy(newRgbColors.begin(), newRgbColors.end(), _rgbColors.begin() + _count);
  }

  if (newNormals.size() > 0) {
    _normals.resize(_normals.size() + newPointsSize);
    std::copy(newNormals.begin(), newNormals.end(), _normals.begin() + _count);
  }

  if (newIntensities.size() > 0) {
    _intensities.resize(_intensities.size() + newPointsSize);
    std::copy(newIntensities.begin(), newIntensities.end(), _intensities.begin() + _count);
  }

  if (newClassifications.size() > 0) {
    _classifications.resize(_classifications.size() + newPointsSize);
    std::copy(
      newClassifications.begin(), newClassifications.end(), _classifications.begin() + _count);
  }

  _count += newPointsSize;
}

Potree::PointBuffer::PointConstReference
Potree::PointBuffer::get_point(size_t point_index) const
{
  return { this, point_index };
}

Potree::PointBuffer::PointReference
Potree::PointBuffer::get_point(size_t point_index)
{
  return { this, point_index };
}

void
Potree::PointBuffer::append_buffer(const PointBuffer& other)
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

  _count += other.count();
}

void
Potree::PointBuffer::clear()
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
Potree::PointBuffer::hasColors() const
{
  return !_rgbColors.empty();
}

bool
Potree::PointBuffer::hasNormals() const
{
  return !_normals.empty();
}

bool
Potree::PointBuffer::hasIntensities() const
{
  return !_intensities.empty();
}

bool
Potree::PointBuffer::hasClassifications() const
{
  return !_classifications.empty();
}

void
Potree::PointBuffer::verify() const
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
}

size_t
Potree::PointBuffer::content_byte_size() const
{
  return vector_byte_size(_positions) + vector_byte_size(_rgbColors) + vector_byte_size(_normals) +
         vector_byte_size(_intensities) + vector_byte_size(_classifications);
}

Potree::PointBuffer::PointIterator
Potree::PointBuffer::begin()
{
  return PointBuffer::PointIterator{ *this, 0 };
}

Potree::PointBuffer::PointIterator
Potree::PointBuffer::end()
{
  return PointBuffer::PointIterator{ *this, count() };
}

Potree::PointBuffer::PointConstIterator
Potree::PointBuffer::begin() const
{
  return PointBuffer::PointConstIterator{ *this, 0 };
}
Potree::PointBuffer::PointConstIterator
Potree::PointBuffer::end() const
{
  return PointBuffer::PointConstIterator{ *this, count() };
}

#pragma region PointConstReference
const Potree::Vector3<double>&
Potree::PointBuffer::PointConstReference::position() const
{
  return _pointBuffer->positions()[_index];
}

const Potree::Vector3<uint8_t>*
Potree::PointBuffer::PointConstReference::rgbColor() const
{
  if (!_pointBuffer->hasColors())
    return nullptr;
  return _pointBuffer->rgbColors().data() + _index;
}

const Potree::Vector3<float>*
Potree::PointBuffer::PointConstReference::normal() const
{
  if (!_pointBuffer->hasNormals())
    return nullptr;
  return _pointBuffer->normals().data() + _index;
}

const uint16_t*
Potree::PointBuffer::PointConstReference::intensity() const
{
  if (!_pointBuffer->hasIntensities())
    return nullptr;
  return _pointBuffer->intensities().data() + _index;
}

const uint8_t*
Potree::PointBuffer::PointConstReference::classification() const
{
  if (!_pointBuffer->hasClassifications())
    return nullptr;
  return _pointBuffer->classifications().data() + _index;
}

Potree::PointBuffer::PointConstReference::PointConstReference(PointBuffer const* pointBuffer,
                                                              size_t index)
  : _pointBuffer(pointBuffer)
  , _index(index)
{}

Potree::PointBuffer::PointConstReference::PointConstReference(const PointReference& point_reference)
  : _pointBuffer(point_reference._pointBuffer)
  , _index(point_reference._index)
{}
#pragma endregion

#pragma region PointReference
Potree::Vector3<double>&
Potree::PointBuffer::PointReference::position() const
{
  return _pointBuffer->positions()[_index];
}

Potree::Vector3<uint8_t>*
Potree::PointBuffer::PointReference::rgbColor() const
{
  if (!_pointBuffer->hasColors())
    return nullptr;
  return _pointBuffer->rgbColors().data() + _index;
}

Potree::Vector3<float>*
Potree::PointBuffer::PointReference::normal() const
{
  if (!_pointBuffer->hasNormals())
    return nullptr;
  return _pointBuffer->normals().data() + _index;
}

uint16_t*
Potree::PointBuffer::PointReference::intensity() const
{
  if (!_pointBuffer->hasIntensities())
    return nullptr;
  return _pointBuffer->intensities().data() + _index;
}

uint8_t*
Potree::PointBuffer::PointReference::classification() const
{
  if (!_pointBuffer->hasClassifications())
    return nullptr;
  return _pointBuffer->classifications().data() + _index;
}

Potree::PointBuffer::PointReference::PointReference(PointBuffer* pointBuffer, size_t index)
  : _pointBuffer(pointBuffer)
  , _index(index)
{}
#pragma endregion

#pragma region PointConstIterator
Potree::PointBuffer::PointConstIterator::PointConstIterator(const PointBuffer& pointBuffer,
                                                            size_t idx)
  : _pointBuffer(&pointBuffer)
  , _index(idx)
{}

Potree::PointBuffer::PointConstReference Potree::PointBuffer::PointConstIterator::operator*() const
{
  return PointBuffer::PointConstReference{ _pointBuffer, _index };
}

Potree::PointBuffer::PointConstIterator&
Potree::PointBuffer::PointConstIterator::operator++()
{
  ++_index;
  return *this;
}

bool
Potree::PointBuffer::PointConstIterator::operator==(const PointConstIterator& other) const
{
  return _pointBuffer == other._pointBuffer && _index == other._index;
}

bool
Potree::PointBuffer::PointConstIterator::operator!=(const PointConstIterator& other) const
{
  return _pointBuffer != other._pointBuffer || _index != other._index;
}

Potree::PointBuffer::PointConstIterator
Potree::PointBuffer::PointConstIterator::operator+(size_t idx) const
{
  return { *_pointBuffer, _index + idx };
}

#pragma endregion

#pragma region PointIterator
Potree::PointBuffer::PointIterator::PointIterator(PointBuffer& pointBuffer, size_t idx)
  : _pointBuffer(&pointBuffer)
  , _index(idx)
{}

Potree::PointBuffer::PointReference Potree::PointBuffer::PointIterator::operator*() const
{
  return PointBuffer::PointReference{ _pointBuffer, _index };
}

Potree::PointBuffer::PointIterator&
Potree::PointBuffer::PointIterator::operator++()
{
  ++_index;
  return *this;
}

bool
Potree::PointBuffer::PointIterator::operator==(const PointIterator& other) const
{
  return _pointBuffer == other._pointBuffer && _index == other._index;
}

bool
Potree::PointBuffer::PointIterator::operator!=(const PointIterator& other) const
{
  return _pointBuffer != other._pointBuffer || _index != other._index;
}

Potree::PointBuffer::PointIterator
Potree::PointBuffer::PointIterator::operator+(size_t idx) const
{
  return { *_pointBuffer, _index + idx };
}

#pragma endregion
