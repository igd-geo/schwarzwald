#include "PNTSReader.h"

#include <cstddef>
#include <fstream>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

namespace rs = rapidjson;

std::optional<Potree::PNTSFile> Potree::readPNTSFile(
    const std::string& filepath) {
  std::ifstream fs{filepath, std::ios_base::in | std::ios_base::binary};
  if (!fs.is_open()) {
    std::cerr << "Could not open .pnts file \"" << filepath << "\"!"
              << std::endl;
    return std::nullopt;
  }

  fs.unsetf(std::ios::skipws);

  fs.seekg(0, std::ios::end);
  auto fileSize = fs.tellg();
  fs.seekg(0, std::ios::beg);

  std::vector<char> rawData;
  rawData.reserve(fileSize);

  rawData.insert(rawData.begin(), std::istream_iterator<char>(fs),
                 std::istream_iterator<char>());

  PNTSFile pntsFile;
  pntsFile.header = *reinterpret_cast<PNTSHeader*>(rawData.data());

  const auto featureTableJSONBegin = rawData.data() + sizeof(PNTSHeader);
  const auto featureTableJSONEnd =
      featureTableJSONBegin + pntsFile.header.featureTableJSONByteLength;
  // There is no guarantee that the feature table JSON is null-terminated, so
  // 'featureTableJSONBegin' is not a valid C-string
  std::string featureTableJSON{featureTableJSONBegin, featureTableJSONEnd};

  const auto featureTableBinaryBegin = featureTableJSONEnd;
  const auto featureTableBinaryEnd =
      featureTableBinaryBegin + pntsFile.header.featureTableBinaryByteLength;

  // TODO Batch table

  rs::Document featureTableJSONDocument;
  if (featureTableJSONDocument.Parse<0>(featureTableJSON.c_str())
          .HasParseError()) {
    auto parseError = featureTableJSONDocument.GetParseError();
    auto parseErrorMsg = rs::GetParseError_En(parseError);
    std::cerr << "Could not parse feature table JSON in \"" << filepath
              << "\" (" << parseErrorMsg << ")" << std::endl;
    return std::nullopt;
  }

  // TODO Error handling for potentially missing JSON members

  const auto pointsLength =
      featureTableJSONDocument.FindMember("POINTS_LENGTH")->value.GetUint();
  const auto& rtcCenterMember =
      featureTableJSONDocument.FindMember("RTC_CENTER")->value;
  pntsFile.rtc_center = {rtcCenterMember[0].GetDouble(),
                         rtcCenterMember[1].GetDouble(),
                         rtcCenterMember[2].GetDouble()};

  const auto& positionMember =
      featureTableJSONDocument.FindMember("POSITION")->value;
  const auto positionByteOffset =
      positionMember.FindMember("byteOffset")->value.GetUint();

  // TODO Other per-point attributes

  const auto positionsBegin = reinterpret_cast<const Vector3<float>*>(
      featureTableBinaryBegin + positionByteOffset);
  const auto positionsEnd = positionsBegin + pointsLength;
  std::vector<Vector3<double>> highpPositions;
  highpPositions.reserve(pointsLength);
  std::transform(positionsBegin, positionsEnd,
                 std::back_inserter(highpPositions),
                 Vector3<float>::cast<double>);

  pntsFile.points = {pointsLength, std::move(highpPositions)};

  return std::make_optional<PNTSFile>(std::move(pntsFile));
}
