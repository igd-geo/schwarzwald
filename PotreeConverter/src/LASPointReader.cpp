

#include <fstream>
#include <iostream>
#include <vector>

#include <experimental/filesystem>
#include "laszip_api.h"

#include "LASPointReader.h"
#include "PointAttributes.hpp"
#include "stuff.h"

namespace fs = std::experimental::filesystem;

namespace Potree {

PointBuffer LIBLASReader::readNextBatch(size_t maxBatchSize) {
  const auto remainingPoints = static_cast<size_t>(numPoints() - pointsRead);
  const auto batchSize = std::min(remainingPoints, maxBatchSize);

  // TODO Read all and only those attributes that we care about, based on the
  // PointAttributes structure that we should pass to LASPointReader

  std::vector<Vector3<double>> positions;
  positions.resize(batchSize);

  for (size_t idx = 0; idx < batchSize; ++idx) {
    laszip_read_point(laszip_reader);

    auto currentPositionPtr =
        reinterpret_cast<double *>(positions.data() + idx);
    laszip_get_coordinates(laszip_reader, currentPositionPtr);

    //... other attributes ...
  }

  pointsRead += batchSize;

  return PointBuffer{batchSize, std::move(positions)};
}

bool LIBLASReader::isAtEnd() const { return pointsRead == numPoints(); }

bool LIBLASReader::hasAttributes(const PointAttributes &attributes,
                                 PointAttributes *missingAttributes) const {
  for (auto &attribute : attributes.attributes) {
    if (!hasAttribute(attribute)) {
      if (!missingAttributes) return false;
      missingAttributes->add(attribute);
    }
  }
  return true;
}

AABB LIBLASReader::getAABB() {
  AABB aabb;

  // TODO Implement AABB with coordinate transformation
  // Point minp = transform(header->min_x, header->min_y, header->min_z);
  // Point maxp = transform(header->max_x, header->max_y, header->max_z);
  aabb.update({header->min_x, header->min_y, header->min_z});
  aabb.update({header->max_x, header->max_y, header->max_z});

  return aabb;
}

bool LIBLASReader::hasAttribute(const PointAttribute &attribute) const {
  switch (attribute.ordinal) {
    case attributes::POSITION_CARTESIAN:
      return true;  // LAS always has positions
    case attributes::COLOR_PACKED:
      return hasColor();
    case attributes::INTENSITY:
      return hasIntensity();
    case attributes::NORMAL:
    case attributes::NORMAL_OCT16:
    case attributes::NORMAL_SPHEREMAPPED:
      return hasNormals();
    case attributes::CLASSIFICATION:
      return true;  // LAS always has classifications
    default:
      throw std::runtime_error{"Unrecognized attribute type!"};
  }
}

bool LIBLASReader::hasColor() const {
  // Point data record formats 2 and 3 have RGB colors!
  return header->point_data_format == 2 || header->point_data_format == 3;
}

bool LIBLASReader::hasIntensity() const {
  // TODO Currently, I don't see a way to identify if intensity is really
  // included or not. The spec says that the intensity field is optional, but
  // there seems to be no flag that indicates whether it is included or not...
  return true;
}

bool LIBLASReader::hasNormals() const {
  // TODO Could these be stored in the variable-length headers? If so we would
  // have to check them
  return false;
}

LASPointReader::LASPointReader(const std::string &path,
                               const PointAttributes &requestedAttributes)
    : path(path) {
  if (fs::is_directory(path)) {
    // if directory is specified, find all las and laz files inside directory

    for (fs::directory_iterator it(path); it != fs::directory_iterator();
         it++) {
      fs::path filepath = it->path();
      if (fs::is_regular_file(filepath)) {
        if (icompare(fs::path(filepath).extension().string(), ".las") ||
            icompare(fs::path(filepath).extension().string(), ".laz")) {
          files.push_back(filepath.string());
        }
      }
    }
  } else {
    files.push_back(path);
  }

  // read bounding box
  for (const auto &file : files) {
    LIBLASReader tmpReader(file);
    AABB lAABB = tmpReader.getAABB();

    aabb.update(lAABB.min);
    aabb.update(lAABB.max);

    PointAttributes missingAttributes;
    if (tmpReader.hasAttributes(requestedAttributes, &missingAttributes)) {
      std::cerr << "LAS/LAZ file \"" << path
                << "\" is missing the following requested attributes: ";
      std::cerr << missingAttributes.toString();
      std::cerr << " These attributes will be filled with default values!"
                << std::endl;
    }
  }

  // open first file
  currentFile = files.begin();
  reader = std::make_unique<LIBLASReader>(*currentFile);
  //    cout << "let's go..." << endl;
}

LASPointReader::~LASPointReader() { close(); }

PointBuffer LASPointReader::readPointBatch(size_t maxBatchSize) {
  // If current reader is at end, try reading next file
  if (reader->isAtEnd()) {
    // TODO There has to be a better way to check if we can create a new reader
    // or not, without the double check for == files.end()
    if (currentFile == files.end()) {
      // Done
      return {};
    }

    ++currentFile;

    if (currentFile == files.end()) {
      return {};
    }

    reader = std::make_unique<LIBLASReader>(*currentFile);
  }

  return reader->readNextBatch(maxBatchSize);
}

void LASPointReader::close() { reader = nullptr; }

long long LASPointReader::numPoints() {
  if (reader->header->version_major >= 1 &&
      reader->header->version_minor >= 4) {
    return reader->header->extended_number_of_point_records;
  } else {
    return reader->header->number_of_point_records;
  }
}

AABB LASPointReader::getAABB() { return aabb; }

Vector3<double> LASPointReader::getScale() {
  Vector3<double> scale;
  scale.x = reader->header->x_scale_factor;
  scale.y = reader->header->y_scale_factor;
  scale.z = reader->header->z_scale_factor;

  return scale;
}

}  // namespace Potree
