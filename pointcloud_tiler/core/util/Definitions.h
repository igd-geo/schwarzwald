#pragma once

#include "algorithms/Enums.h"
#include "algorithms/Pairs.h"

#include <experimental/filesystem>
#include <string>
#include <unordered_set>

#include <boost/format.hpp>

namespace fs = std::experimental::filesystem;

/**
 * Different output formats
 */
enum class OutputFormat
{
  // Custom binary format (uncompressed)
  BIN,
  // Custom binary format (compressed)
  BINZ,
  // Cesium 3D Tiles format (https://github.com/CesiumGS/3d-tiles)
  CZM_3DTILES,
  // LAS format
  LAS,
  // Compressed LAS format (LAZ)
  LAZ,
  // Entwine format using LAS as file type
  ENTWINE_LAS,
  // Entwine format using LAZ as file type
  ENTWINE_LAZ
};

namespace util {
namespace {
static const std::unordered_set<std::pair<OutputFormat, std::string>,
                                util::PairHash>
  OUTPUT_FORMAT_TO_STRING_MAPPING = {
    { OutputFormat::BIN, "BIN" },
    { OutputFormat::BINZ, "BINZ" },
    { OutputFormat::CZM_3DTILES, "3DTILES" },
    { OutputFormat::LAS, "LAS" },
    { OutputFormat::LAZ, "LAZ" },
    { OutputFormat::ENTWINE_LAS, "ENTWINE_LAS" },
    { OutputFormat::ENTWINE_LAZ, "ENTWINE_LAZ" },
  };
}

template<>
inline const std::string&
to_string(OutputFormat output_format)
{
  const auto iter = std::find_if(
    std::begin(OUTPUT_FORMAT_TO_STRING_MAPPING),
    std::end(OUTPUT_FORMAT_TO_STRING_MAPPING),
    [output_format](const auto& pair) { return pair.first == output_format; });
  if (iter == std::end(OUTPUT_FORMAT_TO_STRING_MAPPING)) {
    throw std::invalid_argument{ (boost::format("Invalid OutputFormat %1%") %
                                  static_cast<int>(output_format))
                                   .str() };
  }

  return iter->second;
}

}

/**
 * Named constants for using compression
 */
enum class Compressed
{
  No,
  Yes
};

namespace progress {
const static std::string LOADING{ "loading" };
const static std::string INDEXING{ "indexing" };
const static std::string CONVERTING{ "converting" };
const static std::string GENERATING_TILESETS{ "generating tilesets" };
} // namespace progress