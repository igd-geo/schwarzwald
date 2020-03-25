#include <fstream>
#include <iostream>
#include <vector>

#include "laszip_api.h"
#include <experimental/filesystem>

#include "io/LASPointReader.h"
#include "pointcloud/PointAttributes.h"
#include "util/stuff.h"

struct PositionDestructure {
  using TargetType_t = Vector3<double>;

  template <typename Iter>
  void operator()(laszip_POINTER las_reader, laszip_point *las_point,
                  Iter out) {
    TargetType_t &out_val = *out;
    laszip_get_coordinates(las_reader, reinterpret_cast<double *>(&out_val));
  }
};

template <typename... Args> void las_point_destructure() {
  // Args define how a single attribute is destructured, i.e. how one attribute
  // of a LASPoint maps to an array entry in a PointBuffer
}

PointBuffer LIBLASReader::readNextBatch(size_t maxBatchSize) {
  const auto remainingPoints = static_cast<size_t>(numPoints() - pointsRead);
  const auto batchSize = std::min(remainingPoints, maxBatchSize);

  // Read all and only those attributes that we care about, based on the
  // PointAttributes structure that we should pass to LASPointReader

  const auto continueWith = [](auto call, auto continuation) {
    return [=](auto &&... args) {
      call(std::forward<decltype(args)>(args)...);
      continuation(std::forward<decltype(args)>(args)...);
    };
  };

  std::function<void(size_t, std::vector<Vector3<double>> &,
                     std::vector<Vector3<uint8_t>> &,
                     std::vector<Vector3<float>> &, std::vector<uint16_t> &,
                     std::vector<uint8_t> &)>
      readCurrentPoint = [this](size_t currentIdx, auto &positions,
                                auto &colors, auto &normals, auto &intensities,
                                auto &classifications) {
        laszip_read_point(laszip_reader);
      }; // TODO This can throw

  std::vector<Vector3<double>> positions;
  std::vector<Vector3<uint8_t>> colors;
  std::vector<Vector3<float>> normals;
  std::vector<uint16_t> intensities;
  std::vector<uint8_t> classifications;

  positions.resize(batchSize);

  readCurrentPoint = continueWith(
      readCurrentPoint,
      [this](size_t currentIdx, auto &positions, auto &colors, auto &normals,
             auto &intensities, auto &classifications) {
        auto currentPositionPtr =
            reinterpret_cast<double *>(positions.data() + currentIdx);
        laszip_get_coordinates(laszip_reader, currentPositionPtr);
      });

  if (has_attribute(_requestedAttributes, attributes::COLOR_PACKED)) {
    colors.reserve(batchSize);

    readCurrentPoint = continueWith(
        readCurrentPoint,
        [this](size_t currentIdx, auto &positions, auto &colors, auto &normals,
               auto &intensities, auto &classifications) {
          colors.push_back({static_cast<uint8_t>(point->rgb[0] / colorScale),
                            static_cast<uint8_t>(point->rgb[1] / colorScale),
                            static_cast<uint8_t>(point->rgb[2] / colorScale)});
        });
  }

  if (has_attribute(_requestedAttributes, attributes::INTENSITY)) {
    intensities.reserve(batchSize);

    readCurrentPoint = continueWith(
        readCurrentPoint,
        [this](size_t currentIdx, auto &positions, auto &colors, auto &normals,
               auto &intensities, auto &classifications) {
          intensities.push_back(point->intensity);
        });
  }

  if (has_attribute(_requestedAttributes, attributes::COLOR_FROM_INTENSITY)) {
    colors.reserve(batchSize);

    readCurrentPoint = continueWith(
        readCurrentPoint,
        [this](size_t currentIdx, auto &positions, auto &colors, auto &normals,
               auto &intensities, auto &classifications) {
          colors.push_back(intensityToRGB_Log(point->intensity));
        });
  }

  if (has_attribute(_requestedAttributes, attributes::CLASSIFICATION)) {
    classifications.reserve(batchSize);

    readCurrentPoint = continueWith(
        readCurrentPoint,
        [this](size_t currentIdx, auto &positions, auto &colors, auto &normals,
               auto &intensities, auto &classifications) {
          classifications.push_back(point->classification);
        });
  }

  for (size_t idx = 0; idx < batchSize; ++idx) {
    readCurrentPoint(idx, positions, colors, normals, intensities,
                     classifications);
  }

  pointsRead += batchSize;

  return PointBuffer{
      batchSize,          std::move(positions),   std::move(colors),
      std::move(normals), std::move(intensities), std::move(classifications)};
}

bool LIBLASReader::isAtEnd() const { return pointsRead == numPoints(); }

bool LIBLASReader::hasAttributes(const PointAttributes &attributes,
                                 PointAttributes *missingAttributes) const {
  auto foundMissingAttributes = false;
  for (auto &attribute : attributes) {
    if (!hasAttribute(attribute)) {
      foundMissingAttributes = true;
      if (!missingAttributes)
        break;
      missingAttributes->push_back(attribute);
    }
  }
  return !foundMissingAttributes;
}

AABB LIBLASReader::getAABB() const { return _bounds; }

Vector3<double> LIBLASReader::getOffset() const {
  return {header->x_offset, header->y_offset, header->z_offset};
}

bool LIBLASReader::hasAttribute(const PointAttribute &attribute) const {
  switch (attribute.ordinal) {
  case attributes::POSITION_CARTESIAN:
    return true; // LAS always has positions
  case attributes::COLOR_PACKED:
    return hasColor();
  case attributes::INTENSITY:
  case attributes::COLOR_FROM_INTENSITY:
    return hasIntensity();
  case attributes::NORMAL:
  case attributes::NORMAL_OCT16:
  case attributes::NORMAL_SPHEREMAPPED:
    return hasNormals();
  case attributes::CLASSIFICATION:
    return true; // LAS always has classifications
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
    : path(path), _requestedAttributes(requestedAttributes) {
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
    LIBLASReader tmpReader(file, _requestedAttributes);
    AABB lAABB = tmpReader.getAABB();

    aabb.update(lAABB.min);
    aabb.update(lAABB.max);

    // PointAttributes missingAttributes;
    // if (!tmpReader.hasAttributes(requestedAttributes, &missingAttributes)) {
    //   std::cerr << "LAS/LAZ file \"" << path
    //             << "\" is missing the following requested attributes: ";
    //   std::cerr << missingAttributes.toString();
    //   std::cerr << " These attributes will be filled with default values!" <<
    //   std::endl;
    // }
  }

  // open first file
  currentFile = files.begin();
  reader = std::make_unique<LIBLASReader>(*currentFile, _requestedAttributes);
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

    reader = std::make_unique<LIBLASReader>(*currentFile, _requestedAttributes);
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