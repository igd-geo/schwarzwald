#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "io/PointReader.h"
#include "laszip_api.h"
#include "pointcloud/PointAttributes.h"
#include "proj_api.h"
#include "util/stuff.h"

class LIBLASReader {
private:
  size_t _numPoints;
  AABB _bounds;
  const PointAttributes &_requestedAttributes;

public:
  laszip_POINTER laszip_reader;
  laszip_header *header;
  laszip_point *point;
  int colorScale;
  double coordinates[3];
  uint64_t pointsRead = 0ull;

  LIBLASReader(const std::string &path,
               const PointAttributes &requestedAttributes)
      : _requestedAttributes(requestedAttributes) {
    // TODO Error handling
    laszip_create(&laszip_reader);

    laszip_BOOL request_reader = 1;
    laszip_request_compatibility_mode(laszip_reader, request_reader);

    { // read first x points to find if color is 1 or 2 bytes
      laszip_BOOL is_compressed = iEndsWith(path, ".laz") ? 1 : 0;
      laszip_open_reader(laszip_reader, path.c_str(), &is_compressed);

      laszip_get_header_pointer(laszip_reader, &header);

      if (header->version_major >= 1 && header->version_minor >= 4) {
        _numPoints = header->extended_number_of_point_records;
      } else {
        _numPoints = header->number_of_point_records;
      }

      _bounds.update({header->min_x, header->min_y, header->min_z});
      _bounds.update({header->max_x, header->max_y, header->max_z});

      laszip_get_point_pointer(laszip_reader, &point);

      colorScale = 1;
      for (size_t i = 0; i < 100000 && i < _numPoints; i++) {
        laszip_read_point(laszip_reader);

        auto r = point->rgb[0];
        auto g = point->rgb[1];
        auto b = point->rgb[2];

        if (r > 255 || g > 255 || b > 255) {
          colorScale = 256;
          break;
        };
      }
    }

    laszip_seek_point(laszip_reader, 0);
  }

  ~LIBLASReader() {
    laszip_close_reader(laszip_reader);
    laszip_destroy(laszip_reader);
  }

  size_t numPoints() const { return _numPoints; }

  PointBuffer readNextBatch(size_t maxBatchSize);

  bool isAtEnd() const;

  /// <summary>
  /// Checks whether the underlying LAS/LAZ file supports all the given
  /// attributes. Returns true if it does and false if it doesn't. If attributes
  /// are not present and missingAttributes is not null, the missing attributes
  /// will be pushed into missingAttributes
  /// </summary>
  bool hasAttributes(const PointAttributes &attributes,
                     PointAttributes *missingAttributes = nullptr) const;

  bool readPoint() {
    if (pointsRead < numPoints()) {
      laszip_read_point(laszip_reader);
      pointsRead++;

      return true;
    } else {
      return false;
    }
  }

  Point GetPoint() {
    laszip_get_coordinates(laszip_reader, coordinates);

    Point p;
    p.position = {coordinates[0], coordinates[1], coordinates[2]};
    p.intensity = point->intensity;
    p.classification = point->classification;

    p.color.x = point->rgb[0] / colorScale;
    p.color.y = point->rgb[1] / colorScale;
    p.color.z = point->rgb[2] / colorScale;

    p.returnNumber = point->return_number;
    p.numberOfReturns = point->number_of_returns;
    p.pointSourceID = point->point_source_ID;

    return p;
  }

  AABB getAABB() const;

  Vector3<double> getOffset() const;

private:
  bool hasAttribute(const PointAttribute &attribute) const;

  bool hasColor() const;
  bool hasIntensity() const;
  bool hasNormals() const;
};

class LASPointReader : public PointReader {
private:
  AABB aabb;
  std::string path;
  std::unique_ptr<LIBLASReader> reader;
  std::vector<std::string> files;
  std::vector<std::string>::iterator currentFile;
  const PointAttributes &_requestedAttributes;

public:
  LASPointReader(const std::string &path,
                 const PointAttributes &requestedAttributes);

  ~LASPointReader();

  PointBuffer readPointBatch(size_t maxBatchSize) override;

  AABB getAABB();

  long long numPoints();

  void close();

  Vector3<double> getScale();
};