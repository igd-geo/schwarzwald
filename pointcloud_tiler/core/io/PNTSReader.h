#pragma once

#include "datastructures/PointBuffer.h"

#include <optional>
#include <stdint.h>

struct PNTSHeader
{
  uint8_t magic[4];
  uint32_t version;
  uint32_t byteLength;
  uint32_t featureTableJSONByteLength;
  uint32_t featureTableBinaryByteLength;
  uint32_t batchTableJSONByteLength;
  uint32_t batchTableBinaryByteLength;
};

struct PNTSFile
{
  PNTSHeader header;
  Vector3<double> rtc_center;
  PointBuffer points;
};

/// <summary>
/// Tries to read the given .pnts file. Returns std::nullopt on failure
/// </summary>
std::optional<PNTSFile>
readPNTSFile(const std::string& filepath, const PointAttributes& input_attributes);
