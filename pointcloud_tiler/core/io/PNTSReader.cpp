#include "io/PNTSReader.h"

#include <cstddef>
#include <fstream>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

namespace rs = rapidjson;

template<typename FeatureType>
static void
extractFeatureArray(std::vector<FeatureType>& featureVec,
                    const char* featureName,
                    const rapidjson::Document& featureTableJSONDocument,
                    const char* featureTableBinaryBegin,
                    uint32_t pointsLength)
{
  const auto featureMember = featureTableJSONDocument.FindMember(featureName);
  if (featureMember == featureTableJSONDocument.MemberEnd())
    return;

  const auto featureByteOffset = featureMember->value.FindMember("byteOffset")->value.GetUint();
  const auto featureBegin =
    reinterpret_cast<const FeatureType*>(featureTableBinaryBegin + featureByteOffset);
  const auto featureEnd = featureBegin + pointsLength;
  featureVec.insert(featureVec.begin(), featureBegin, featureEnd);
}

std::optional<PNTSFile>
readPNTSFile(const std::string& filepath, const PointAttributes& input_attributes)
{
  std::ifstream fs{ filepath, std::ios_base::in | std::ios_base::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not open .pnts file \"" << filepath << "\"!" << std::endl;
    return std::nullopt;
  }

  fs.unsetf(std::ios::skipws);

  fs.seekg(0, std::ios::end);
  auto fileSize = fs.tellg();
  fs.seekg(0, std::ios::beg);

  std::vector<char> rawData;
  rawData.reserve(fileSize);

  rawData.insert(rawData.begin(), std::istream_iterator<char>(fs), std::istream_iterator<char>());

  PNTSFile pntsFile;
  pntsFile.header = *reinterpret_cast<PNTSHeader*>(rawData.data());

  const auto featureTableJSONBegin = rawData.data() + sizeof(PNTSHeader);
  const auto featureTableJSONEnd =
    featureTableJSONBegin + pntsFile.header.featureTableJSONByteLength;
  // There is no guarantee that the feature table JSON is null-terminated, so
  // 'featureTableJSONBegin' is not a valid C-string
  std::string featureTableJSON{ featureTableJSONBegin, featureTableJSONEnd };

  const auto featureTableBinaryBegin = featureTableJSONEnd;
  //   const auto featureTableBinaryEnd =
  //       featureTableBinaryBegin +
  //       pntsFile.header.featureTableBinaryByteLength;

  // FEATURE Support reading data from the batch table

  rs::Document featureTableJSONDocument;
  if (featureTableJSONDocument.Parse<0>(featureTableJSON.c_str()).HasParseError()) {
    auto parseError = featureTableJSONDocument.GetParseError();
    auto parseErrorMsg = rs::GetParseError_En(parseError);
    std::cerr << "Could not parse feature table JSON in \"" << filepath << "\" (" << parseErrorMsg
              << ")" << std::endl;
    return std::nullopt;
  }

  // TECH_DEBT Error handling for potentially missing JSON members

  const auto pointsLength = featureTableJSONDocument.FindMember("POINTS_LENGTH")->value.GetUint();
  const auto& rtcCenterMember = featureTableJSONDocument.FindMember("RTC_CENTER")->value;
  pntsFile.rtc_center = { rtcCenterMember[0].GetDouble(),
                          rtcCenterMember[1].GetDouble(),
                          rtcCenterMember[2].GetDouble() };

  const auto& positionMember = featureTableJSONDocument.FindMember("POSITION")->value;
  const auto positionByteOffset = positionMember.FindMember("byteOffset")->value.GetUint();

  const auto positionsBegin =
    reinterpret_cast<const Vector3<float>*>(featureTableBinaryBegin + positionByteOffset);
  const auto positionsEnd = positionsBegin + pointsLength;
  std::vector<Vector3<double>> highpPositions;
  highpPositions.reserve(pointsLength);
  std::transform(
    positionsBegin, positionsEnd, std::back_inserter(highpPositions), Vector3<float>::cast<double>);

  std::vector<Vector3<uint8_t>> colors;
  if (has_attribute(input_attributes, PointAttribute::RGB)) {
    extractFeatureArray<Vector3<uint8_t>>(
      colors, "RGB", featureTableJSONDocument, featureTableBinaryBegin, pointsLength);
  }

  std::vector<Vector3<float>> normals;
  // FEATURE Read normals from PNTS file
  if (has_attribute(input_attributes, PointAttribute::Normal)) {
    throw std::runtime_error{ "Reading normals from .pnts files is not supported" };
  }

  std::vector<uint16_t> intensities;
  if (has_attribute(input_attributes, PointAttribute::Intensity)) {
    extractFeatureArray<uint16_t>(
      intensities, "INTENSITY", featureTableJSONDocument, featureTableBinaryBegin, pointsLength);
  }

  std::vector<uint8_t> classifications;
  if (has_attribute(input_attributes, PointAttribute::Classification)) {
    extractFeatureArray<uint8_t>(classifications,
                                 "CLASSIFICATION",
                                 featureTableJSONDocument,
                                 featureTableBinaryBegin,
                                 pointsLength);
  }

  // All other attributes are not supported currently
  if (has_attribute(input_attributes, PointAttribute::EdgeOfFlightLine)) {
    throw std::runtime_error{ "Reading edge of flight lines from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::GPSTime)) {
    throw std::runtime_error{ "Reading GPS times from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::NumberOfReturns)) {
    throw std::runtime_error{ "Reading number of returns from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::PointSourceID)) {
    throw std::runtime_error{ "Reading point source IDs from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::ReturnNumber)) {
    throw std::runtime_error{ "Reading return numbers from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::ScanAngleRank)) {
    throw std::runtime_error{ "Reading scan angle rank from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::ScanDirectionFlag)) {
    throw std::runtime_error{ "Reading scan direction flags from .pnts files is not supported" };
  }
  if (has_attribute(input_attributes, PointAttribute::UserData)) {
    throw std::runtime_error{ "Reading user data from .pnts files is not supported" };
  }

  pntsFile.points = { pointsLength,       std::move(highpPositions), std::move(colors),
                      std::move(normals), std::move(intensities),    std::move(classifications) };

  return std::make_optional<PNTSFile>(std::move(pntsFile));
}
