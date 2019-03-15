#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <variant>
#include <vector>

#include "AABB.h"
#include "Point.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "Vector3.h"

#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

namespace Potree {

struct BatchTable {
  // caontains metadata for points
};

// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/TileFormats/PointCloud/README.md

struct RGBA {
  RGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : R(r), G(g), B(b), A(a) {}
  uint8_t R;
  uint8_t G;
  uint8_t B;
  uint8_t A;
};
struct RGB {
  RGB(uint8_t r, uint8_t g, uint8_t b) : R(r), G(g), B(b) {}
  uint8_t R;
  uint8_t G;
  uint8_t B;
};
struct NORMAL_OCT16P {
  NORMAL_OCT16P(uint8_t x, uint8_t y) : x(x), y(y) {}
  uint8_t x;
  uint8_t y;
};

namespace attributes {

/// <summary>
/// Base class for per-point attributes. These attributes extract data from the
/// Potree Point structures and convert them into a binary blob that can be
/// written into the .pnts file. See also:
/// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/specification/TileFormats/PointCloud/README.md#point-semantics
/// </summary>
struct PointAttributeBase {
  using Ptr = std::unique_ptr<PointAttributeBase>;

  virtual ~PointAttributeBase() {}
  virtual void extractFromPoints(const PointBuffer& points) = 0;
  /// <summary>
  /// Gets the name of this attribute for the JSON header of the feature table
  /// </summary>
  virtual std::string getAttributeNameForJSON() const = 0;
  /// <summary>
  /// Returns a iterator-pair that represents the memory range of the data for
  /// this attribute
  /// </summary>
  virtual gsl::span<const std::byte> getBinaryDataRange() const = 0;
  /// <summary>
  /// Returns the byte alignment required for this attribute
  /// </summary>
  virtual uint32_t getAlignmentRequirement() const = 0;
  /// <summary>
  /// Returns the number of entries for this attribute
  /// </summary>
  virtual size_t getNumEntries() const = 0;
};

/// <summary>
/// Base class for global point attributes. Similar to PointAttributeBase but
/// with a single-object store since these attributes are global. See also:
/// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/specification/TileFormats/PointCloud/README.md#global-semantics
/// </summary>
struct GlobalAttributeBase {
  using Ptr = std::unique_ptr<GlobalAttributeBase>;

  virtual ~GlobalAttributeBase() {}
  virtual void serialize(rapidjson::Document& jsonDocument) const = 0;
  // TODO Some sort of set function to set the actual value
};

struct PositionAttribute : PointAttributeBase {
  void extractFromPoints(const PointBuffer& points) override;
  std::string getAttributeNameForJSON() const override;
  gsl::span<const std::byte> getBinaryDataRange() const override;
  uint32_t getAlignmentRequirement() const override;
  size_t getNumEntries() const override { return _positions.size(); }

 private:
  std::vector<Vector3<float>> _positions;
};

struct PositionQuantizedAttribute : PointAttributeBase {
  void extractFromPoints(const PointBuffer& points) override;
  std::string getAttributeNameForJSON() const override;
  gsl::span<const std::byte> getBinaryDataRange() const override;
  uint32_t getAlignmentRequirement() const override;
  size_t getNumEntries() const override { return _quantizedPositions.size(); }

 private:
  std::vector<NORMAL_OCT16P> _quantizedPositions;
};

struct RGBAAttribute : PointAttributeBase {
  void extractFromPoints(const PointBuffer& points) override;
  std::string getAttributeNameForJSON() const override;
  gsl::span<const std::byte> getBinaryDataRange() const override;
  uint32_t getAlignmentRequirement() const override;
  size_t getNumEntries() const override { return _rgbaColors.size(); }

 private:
  std::vector<RGBA> _rgbaColors;
};

struct RGBAttribute : PointAttributeBase {
  void extractFromPoints(const PointBuffer& points) override;
  std::string getAttributeNameForJSON() const override;
  gsl::span<const std::byte> getBinaryDataRange() const override;
  uint32_t getAlignmentRequirement() const override;
  size_t getNumEntries() const override { return _rgbColors.size(); }

 private:
  std::vector<RGB> _rgbColors;
};

struct IntensityAttribute : PointAttributeBase {
  void extractFromPoints(const PointBuffer& points) override;
  std::string getAttributeNameForJSON() const override;
  gsl::span<const std::byte> getBinaryDataRange() const override;
  uint32_t getAlignmentRequirement() const override;
  size_t getNumEntries() const override { return _intensities.size(); }

 private:
  std::vector<uint16_t> _intensities;
};

struct ClassificationAttribute : PointAttributeBase {
  void extractFromPoints(const PointBuffer& points) override;
  std::string getAttributeNameForJSON() const override;
  gsl::span<const std::byte> getBinaryDataRange() const override;
  uint32_t getAlignmentRequirement() const override;
  size_t getNumEntries() const override { return _classifications.size(); }

 private:
  std::vector<uint8_t> _classifications;
};

// TODO Implement other point attributes (normals, batch_id etc.)

}  // namespace attributes

struct FeatureTable {
  // numPoints is the global attribute POINTS_LENGTH, but since it is always
  // mandatory we store it as a raw integer (instead of a GlobalAttributeBase
  // instance) for convenience
  uint32_t numPoints = 0;

  std::vector<attributes::GlobalAttributeBase::Ptr> globalAttributes;
  std::vector<attributes::PointAttributeBase::Ptr> perPointAttributes;
};

/// <summary>
/// Writer for writing the .pnts files of the 3D-Tiles spec:
/// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/specification/TileFormats/PointCloud/README.md
/// </summary>
class PNTSWriter {
 public:
  PNTSWriter(const std::string& filePath,
             const PointAttributes& pointAttributes);
  ~PNTSWriter();

  void writePoints(const PointBuffer& points);

  void flush(const Vector3<double>& localCenter);

  void close();

 private:
  std::string _filePath;
  FeatureTable _featuretable;
  BatchTable _batchtable;

  // header

  const char magic[4] = {'p', 'n', 't', 's'};
  const uint32_t version = 1;
  uint32_t t_byteLength = 0;  // length of the entire tile including header
  uint32_t ftJSON_byteLength =
      0;  // if this equals zero the tile does not need to be rendered
  uint32_t ft_byteLength = 0;
  uint32_t btJSON_byteLength = 0;
  uint32_t bt_byteLength = 0;

  int position_byteLength =
      0;  // This is necessary for byte offset in feature table
  int position_quantized_byteLength = 0;
  int rgba_byteLength = 0;
  int rgb_byteLength = 0;
  int rgb565_byteLength = 0;
  int normal_byteLength = 0;
  int normal_oct_byteLength = 0;
  int batchid_byteLength = 0;

  int number_of_features = 0;

  std::vector<float> positions;
  std::vector<uint8_t> colors;

  struct FeatureTableBlob {
    std::vector<std::byte> bytes;
    uint32_t jsonByteLength;
    uint32_t binaryByteLength;
  };

  /// <summary>
  /// Creates a binary blob containing the feature table for the current set of
  /// points. This also contains the relevant offsets for the .pnts header
  /// </summary>
  FeatureTableBlob createFeatureTableBlob(const Vector3<double>& localCenter);
};

}  // namespace Potree
