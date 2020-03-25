
#include "io/PNTSWriter.h"
#include "io/PNTSReader.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "util/Transformation.h"
#include "util/stuff.h"
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

#include <sstream>

using namespace rapidjson;

template <typename T> void writeBinary(const T &obj, std::ofstream &stream) {
  const auto mem = reinterpret_cast<const char *>(&obj);
  const auto size = sizeof(T);
  stream.write(mem, size);
}

namespace {
static attributes::PointAttributeBase::Ptr
createPointAttributeCache(const PointAttribute &attribute) {
  switch (attribute) {
  case attributes::POSITION_CARTESIAN:
    return std::make_unique<attributes::PositionAttribute>();
  case attributes::COLOR_PACKED:
  case attributes::COLOR_FROM_INTENSITY:
    return std::make_unique<attributes::RGBAttribute>();
  case attributes::INTENSITY:
    return std::make_unique<attributes::IntensityAttribute>();
  case attributes::CLASSIFICATION:
    return std::make_unique<attributes::ClassificationAttribute>();
  default:
    return nullptr;
  }
}
} // namespace

PNTSWriter::PNTSWriter(const std::string &filePath,
                       const PointAttributes &pointAttributes)
    : _filePath(filePath) {
  for (auto &desiredPointAttribute : pointAttributes) {
    auto attributeCache = createPointAttributeCache(desiredPointAttribute);
    if (!attributeCache) {
      std::cerr << "Could not create attribute cache for PointAttribute "
                << desiredPointAttribute.name
                << "! Attribute will be skipped..." << std::endl;
      continue;
    }
    _featuretable.perPointAttributes.push_back(std::move(attributeCache));
  }

  // TODO Create global attributes (how should we define them?)
}

PNTSWriter::~PNTSWriter() { close(); }

/*
Writes a Point to the Batch Table
*/
void PNTSWriter::write_points(const PointBuffer &points) {
  const auto numPoints = points.count();
  if (!numPoints) {
    return;
  }

  _featuretable.numPoints += numPoints;

  for (auto &localAttribute : _featuretable.perPointAttributes) {
    localAttribute->extractFromPoints(points);
  }
}

void PNTSWriter::write_points(gsl::span<PointBuffer::PointReference> points) {
  const auto num_points = points.size();
  if (!num_points) {
    return;
  }

  _featuretable.numPoints += num_points;

  for (auto &local_attribute : _featuretable.perPointAttributes) {
    local_attribute->extractFromPoints(points);
  }
}

void PNTSWriter::flush(const Vector3<double> &localCenter) {
  std::ofstream writer{_filePath, std::ios::out | std::ios::binary};
  if (!writer.is_open()) {
    std::cerr << "Could not write .pnts file \"" << _filePath << "\" ("
              << strerror(errno) << ")" << std::endl;
    return;
  }

  constexpr auto HEADER_SIZE = 28u;

  const auto featureTableBlob = createFeatureTableBlob(localCenter);
  const auto totalSize =
      static_cast<uint32_t>(featureTableBlob.bytes.size()) + HEADER_SIZE;

  // TODO Batch table
  const auto batchTableJSONSize = 0u;
  const auto batchTableBinarySize = 0u;

  // Write header
  writer << "pnts";
  writeBinary<uint32_t>(1, writer);
  writeBinary<uint32_t>(totalSize, writer);
  writeBinary<uint32_t>(featureTableBlob.jsonByteLength, writer);
  writeBinary<uint32_t>(featureTableBlob.binaryByteLength, writer);
  writeBinary<uint32_t>(batchTableJSONSize, writer);
  writeBinary<uint32_t>(batchTableBinarySize, writer);

  // Write body
  writer.write(reinterpret_cast<const char *>(featureTableBlob.bytes.data()),
               featureTableBlob.bytes.size());

  writer.flush();
  writer.close();
}

void PNTSWriter::close() {}

PNTSWriter::FeatureTableBlob PNTSWriter::createFeatureTableBlob(
    const Vector3<double> &localCenter) // create a featuretable
                                        // before this method
{
  Document jsonHeader;
  auto &jsonAllocator = jsonHeader.GetAllocator();

  std::vector<std::byte> binaryBody;

  // We have to write the JSON header and binary body simultaneously because of
  // the byte offsets for all the per-point attributes
  jsonHeader.SetObject();

  // We always have the POINTS_LENGTH member
  jsonHeader.AddMember("POINTS_LENGTH", _featuretable.numPoints, jsonAllocator);

  // We also set the RTC_CENTER property. This prevents jitter when rendering
  // the points
  Value localCenterArray{kArrayType};
  localCenterArray.PushBack(localCenter.x, jsonAllocator);
  localCenterArray.PushBack(localCenter.y, jsonAllocator);
  localCenterArray.PushBack(localCenter.z, jsonAllocator);
  jsonHeader.AddMember("RTC_CENTER", localCenterArray, jsonAllocator);

  struct AttributeDescription {
    gsl::span<const std::byte> binaryRange;
    uint32_t alignedOffset;
    std::string attributeName;
  };
  std::vector<AttributeDescription> attributeDescriptions;
  uint32_t currentAttributeOffset = 0;

  for (auto &perPointAttribute : _featuretable.perPointAttributes) {
    const auto attributeBytesRange = perPointAttribute->getBinaryDataRange();
    const auto attributeByteSize =
        static_cast<uint32_t>(attributeBytesRange.size());

    if (attributeBytesRange.empty()) {
      // TODO This raises a general question: If multiple source files are
      // specified that have different attributes in them, how would we extract
      // these attributes? We can either skip them in all files that have them,
      // assuming that if the attributes are not available for _ALL_ files, they
      // are not available at all, or we go the opposite way and fill the
      // missing attributes with default values... In any case, the user should
      // be notified about this!
      // std::cerr << "No values for attribute \""
      //          << perPointAttribute->getAttributeNameForJSON()
      //          << "\" were extracted from source files, the attribute will be
      //          "
      //             "skipped in \""
      //          << _filePath << "\"!" << std::endl;
      continue;
    }
    if (perPointAttribute->getNumEntries() != _featuretable.numPoints) {
      // std::stringstream ss;
      std::cerr << "Feature [" << perPointAttribute->getAttributeNameForJSON()
                << "] has wrong number of entries (is: "
                << perPointAttribute->getNumEntries()
                << "; should be: " << _featuretable.numPoints << ") on node \""
                << _filePath << "\"" << std::endl;
      continue;
      // throw std::runtime_error{ss.str()};
    }

    const auto alignmentRequirement =
        perPointAttribute->getAlignmentRequirement();
    assert(alignmentRequirement > 0);
    const auto alignedOffset =
        align(currentAttributeOffset, alignmentRequirement);

    AttributeDescription attributeDescription;
    attributeDescription.alignedOffset = alignedOffset;
    attributeDescription.binaryRange = attributeBytesRange;
    attributeDescription.attributeName =
        perPointAttribute->getAttributeNameForJSON();
    attributeDescriptions.push_back(attributeDescription);

    currentAttributeOffset = alignedOffset + attributeByteSize;
  }

  // Binary body has to be 8-byte aligned at the end
  const auto binaryBufferSize = align(currentAttributeOffset, 8u);
  binaryBody.resize(binaryBufferSize);

  for (auto &attributeDescription : attributeDescriptions) {
    // Add the JSON entry containing the byte offset
    Value key{attributeDescription.attributeName.c_str(),
              static_cast<SizeType>(attributeDescription.attributeName.size()),
              jsonAllocator};
    Value obj(kObjectType);
    obj.AddMember("byteOffset",
                  static_cast<int>(attributeDescription.alignedOffset),
                  jsonAllocator);
    jsonHeader.AddMember(key, obj, jsonAllocator);

    // Copy the binary data
    const auto startInBinaryBody =
        binaryBody.data() + attributeDescription.alignedOffset;
    std::copy(attributeDescription.binaryRange.begin(),
              attributeDescription.binaryRange.end(), startInBinaryBody);
  }

  // TODO Global attributes for the JSON header

  StringBuffer jsonBuffer;
  Writer<StringBuffer> writer(jsonBuffer);
  jsonHeader.Accept(writer);

  const auto jsonHeaderString = jsonBuffer.GetString();
  const uint64_t jsonHeaderSize = jsonBuffer.GetSize();

  // JSON header has to be 8-byte aligned
  const auto alignedJsonHeaderSize = align(jsonHeaderSize, uint64_t{8});

  FeatureTableBlob featureTableBlob;
  featureTableBlob.binaryByteLength = binaryBufferSize;
  featureTableBlob.jsonByteLength = alignedJsonHeaderSize;

  featureTableBlob.bytes.resize(alignedJsonHeaderSize + binaryBufferSize);
  // Copy JSON header and binary body into blob
  std::copy(
      reinterpret_cast<std::byte const *>(jsonHeaderString),
      reinterpret_cast<std::byte const *>(jsonHeaderString + jsonHeaderSize),
      featureTableBlob.bytes.data());
  // Write some spaces for the JSON header alignment
  std::generate(featureTableBlob.bytes.data() + jsonHeaderSize,
                featureTableBlob.bytes.data() + alignedJsonHeaderSize, []() {
                  return (std::byte)0x20; /*0x20 = space*/
                });

  std::copy(binaryBody.begin(), binaryBody.end(),
            featureTableBlob.bytes.data() + alignedJsonHeaderSize);

  return featureTableBlob;
}

void transform_pnts_file_coordinates(const std::string &file_path,
                                     Recenter recenter,
                                     const SRSTransformHelper &transform_helper,
                                     const PointAttributes &points_attributes) {

  auto pnts_file = readPNTSFile(file_path);
  if (!pnts_file) {
    std::cerr << "Could not read file " << file_path << std::endl;
    return;
  }
  if (!fs::remove(file_path)) {
    std::cerr << "[transform_pnts_file_coordinates] Could not remove file "
              << file_path << std::endl;
    return;
  }

  // Apply offset from original PNTS file
  for (auto &position : pnts_file->points.positions()) {
    position += pnts_file->rtc_center;
  }

  transform_helper.transformPositionsTo(
      TargetSRS::CesiumWorld, gsl::make_span(pnts_file->points.positions()));

  Vector3<double> local_center;
  if (recenter == Recenter::Yes) {
    local_center = setOriginToSmallestPoint(pnts_file->points.positions());
  }

  PNTSWriter writer{file_path, points_attributes};
  writer.write_points(pnts_file->points);
  writer.flush(local_center);
}

#pragma region attributes

void attributes::PositionAttribute::extractFromPoints(
    const PointBuffer &points) {
  if (!points.count())
    return;

  _positions.reserve(_positions.size() + static_cast<size_t>(points.count()));
  std::transform(points.positions().begin(), points.positions().end(),
                 std::back_inserter(_positions),
                 [](const Vector3<double> &position) -> Vector3<float> {
                   return {static_cast<float>(position.x),
                           static_cast<float>(position.y),
                           static_cast<float>(position.z)};
                 });
}

void attributes::PositionAttribute::extractFromPoints(
    gsl::span<PointBuffer::PointReference> points) {
  if (!points.size())
    return;

  _positions.reserve(_positions.size() + static_cast<size_t>(points.size()));
  std::transform(points.begin(), points.end(), std::back_inserter(_positions),
                 [](const auto &point_reference) -> Vector3<float> {
                   const auto &position = point_reference.position();
                   return {static_cast<float>(position.x),
                           static_cast<float>(position.y),
                           static_cast<float>(position.z)};
                 });
}

std::string attributes::PositionAttribute::getAttributeNameForJSON() const {
  return "POSITION";
}

gsl::span<const std::byte>
attributes::PositionAttribute::getBinaryDataRange() const {
  const auto begin = reinterpret_cast<std::byte const *>(_positions.data());
  const auto end = begin + vector_byte_size(_positions);
  return {begin, end};
}

uint32_t attributes::PositionAttribute::getAlignmentRequirement() const {
  return 4u; // Positions are stored as float values
}

void attributes::PositionQuantizedAttribute::extractFromPoints(
    const PointBuffer &points) {
  // TODO
}

void attributes::PositionQuantizedAttribute::extractFromPoints(
    gsl::span<PointBuffer::PointReference> points) {
  // TODO
}

std::string
attributes::PositionQuantizedAttribute::getAttributeNameForJSON() const {
  return "POSITION_QUANTIZED";
}

gsl::span<const std::byte>
attributes::PositionQuantizedAttribute::getBinaryDataRange() const {
  // TODO
  return {};
}

uint32_t
attributes::PositionQuantizedAttribute::getAlignmentRequirement() const {
  return uint32_t(); // TODO
}

void attributes::RGBAAttribute::extractFromPoints(const PointBuffer &points) {
  if (!points.count())
    return;
  if (!points.hasColors())
    return;

  _rgbaColors.reserve(_rgbaColors.size() + static_cast<size_t>(points.count()));
  std::transform(
      points.rgbColors().begin(), points.rgbColors().end(),
      std::back_inserter(_rgbaColors),
      [](const Vector3<uint8_t> &rgbColor) -> RGBA {
        return {rgbColor.x, rgbColor.y, rgbColor.z, static_cast<uint8_t>(255)};
      });
}

void attributes::RGBAAttribute::extractFromPoints(
    gsl::span<PointBuffer::PointReference> points) {
  if (!points.size())
    return;
  // Only checking the first point here, mixing PointReferences from different
  // PointBuffers is weird...
  if (!points[0].rgbColor())
    return;

  _rgbaColors.reserve(_rgbaColors.size() + static_cast<size_t>(points.size()));
  std::transform(
      points.begin(), points.end(), std::back_inserter(_rgbaColors),
      [](const auto &point_reference) -> RGBA {
        const auto color = point_reference.rgbColor();
        assert(color != nullptr);
        return {color->x, color->y, color->z, static_cast<uint8_t>(255)};
      });
}

std::string attributes::RGBAAttribute::getAttributeNameForJSON() const {
  return "RGBA";
}

gsl::span<const std::byte>
attributes::RGBAAttribute::getBinaryDataRange() const {
  const auto begin = reinterpret_cast<std::byte const *>(_rgbaColors.data());
  const auto end = begin + vector_byte_size(_rgbaColors);
  return {begin, end};
}

uint32_t attributes::RGBAAttribute::getAlignmentRequirement() const {
  return 1u;
}

void attributes::RGBAttribute::extractFromPoints(const PointBuffer &points) {
  if (!points.count())
    return;
  if (!points.hasColors())
    return;

  _rgbColors.reserve(_rgbColors.size() + static_cast<size_t>(points.count()));
  std::transform(points.rgbColors().begin(), points.rgbColors().end(),
                 std::back_inserter(_rgbColors),
                 [](const Vector3<uint8_t> &rgbColor) -> RGB {
                   return {rgbColor.x, rgbColor.y, rgbColor.z};
                 });
}

void attributes::RGBAttribute::extractFromPoints(
    gsl::span<PointBuffer::PointReference> points) {
  if (!points.size())
    return;
  if (!points[0].rgbColor())
    return;

  _rgbColors.reserve(_rgbColors.size() + static_cast<size_t>(points.size()));
  std::transform(points.begin(), points.end(), std::back_inserter(_rgbColors),
                 [](const auto &point_reference) -> RGB {
                   const auto color = point_reference.rgbColor();
                   assert(color != nullptr);
                   return {color->x, color->y, color->z};
                 });
}

std::string attributes::RGBAttribute::getAttributeNameForJSON() const {
  return "RGB";
}

gsl::span<const std::byte>
attributes::RGBAttribute::getBinaryDataRange() const {
  const auto begin = reinterpret_cast<std::byte const *>(_rgbColors.data());
  const auto end = begin + vector_byte_size(_rgbColors);
  return {begin, end};
}

uint32_t attributes::RGBAttribute::getAlignmentRequirement() const {
  return 1u;
}

void attributes::IntensityAttribute::extractFromPoints(
    const PointBuffer &points) {
  if (!points.count())
    return;
  if (!points.hasIntensities())
    return;

  _intensities.insert(_intensities.end(), points.intensities().begin(),
                      points.intensities().end());
}

void attributes::IntensityAttribute::extractFromPoints(
    gsl::span<PointBuffer::PointReference> points) {
  if (!points.size())
    return;
  if (!points[0].intensity())
    return;

  _intensities.reserve(_intensities.size() + points.size());
  std::transform(points.begin(), points.end(), std::back_inserter(_intensities),
                 [](const auto &point_reference) -> uint16_t {
                   const auto intensity = point_reference.intensity();
                   assert(intensity != nullptr);
                   return *intensity;
                 });
}

std::string attributes::IntensityAttribute::getAttributeNameForJSON() const {
  return "INTENSITY";
}

gsl::span<const std::byte>
attributes::IntensityAttribute::getBinaryDataRange() const {
  const auto begin = reinterpret_cast<std::byte const *>(_intensities.data());
  const auto end = begin + vector_byte_size(_intensities);
  return {begin, end};
}

uint32_t attributes::IntensityAttribute::getAlignmentRequirement() const {
  return 2u;
}

void attributes::ClassificationAttribute::extractFromPoints(
    const PointBuffer &points) {
  if (!points.count())
    return;
  if (!points.hasClassifications())
    return;

  _classifications.insert(_classifications.end(),
                          points.classifications().begin(),
                          points.classifications().end());
}

void attributes::ClassificationAttribute::extractFromPoints(
    gsl::span<PointBuffer::PointReference> points) {
  if (!points.size())
    return;
  if (!points[0].classification())
    return;

  _classifications.reserve(_classifications.size() + points.size());
  std::transform(points.begin(), points.end(),
                 std::back_inserter(_classifications),
                 [](const auto &point_reference) -> uint16_t {
                   const auto classification = point_reference.classification();
                   assert(classification != nullptr);
                   return *classification;
                 });
}

std::string
attributes::ClassificationAttribute::getAttributeNameForJSON() const {
  return "CLASSIFICATION";
}

gsl::span<const std::byte>
attributes::ClassificationAttribute::getBinaryDataRange() const {
  const auto begin =
      reinterpret_cast<std::byte const *>(_classifications.data());
  const auto end = begin + vector_byte_size(_classifications);
  return {begin, end};
}

uint32_t attributes::ClassificationAttribute::getAlignmentRequirement() const {
  return 1u;
}

#pragma endregion
