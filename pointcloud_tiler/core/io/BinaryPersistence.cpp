#include "io/BinaryPersistence.h"

#include "io/LASPointReader.h"
#include "laszip_api.h"
#include "util/Transformation.h"

#include <experimental/filesystem>

namespace bio = boost::iostreams;

BinaryPersistence::BinaryPersistence(const std::string &work_dir,
                                     const PointAttributes &point_attributes,
                                     Compressed compressed)
    : _work_dir(work_dir), _point_attributes(point_attributes),
      _compressed(compressed),
      _file_extension(compressed == Compressed::Yes ? ".binz" : ".bin") {}

BinaryPersistence::~BinaryPersistence() {}

void BinaryPersistence::persist_points(PointBuffer const &points,
                                       const AABB &bounds,
                                       const std::string &node_name) {
  if (!points.count())
    throw std::runtime_error{"No points selected"};

  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);

  bio::file_sink fs{file_path, std::ios::out | std::ios::binary};
  if (!fs.is_open()) {
    std::cerr << "Could not write points file " << file_path << std::endl;
    return;
  }

  bio::filtering_ostream stream;
  if (_compressed == Compressed::Yes) {
    bio::zlib_params zlib_params;
    zlib_params.level = bio::zlib::best_speed;
    stream.push(bio::zlib_compressor{zlib_params, 1 << 18});
  }
  stream.push(fs);

  const uint32_t properties_bitmask =
      (points.hasColors() ? COLOR_BIT : 0u) |
      (points.hasNormals() ? NORMAL_BIT : 0u) |
      (points.hasIntensities() ? INTENSITY_BIT : 0u) |
      (points.hasClassifications() ? CLASSIFICATION_BIT : 0u);

  write_binary(properties_bitmask, stream);
  write_binary(static_cast<uint64_t>(points.count()), stream);

  for (auto point : points) {
    write_binary(point.position(), stream);
  }

  if (points.hasColors()) {
    for (auto point : points) {
      write_binary(*point.rgbColor(), stream);
    }
  }

  if (points.hasNormals()) {
    for (auto point : points) {
      write_binary(*point.normal(), stream);
    }
  }

  if (points.hasIntensities()) {
    for (auto point : points) {
      write_binary(*point.intensity(), stream);
    }
  }

  if (points.hasClassifications()) {
    for (auto point : points) {
      write_binary(*point.classification(), stream);
    }
  }

  // stream << std::flush;
  stream.pop();
}

void BinaryPersistence::retrieve_points(const std::string &node_name,
                                        PointBuffer &points) {
  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  if (!std::experimental::filesystem::exists(file_path))
    return;

  bio::file_source fs{file_path, std::ios::in | std::ios::binary};
  if (!fs.is_open()) {
    std::cerr << "Could not read points file " << file_path << std::endl;
    return;
  }

  bio::filtering_istream stream;
  if (_compressed == Compressed::Yes) {
    stream.push(bio::zlib_decompressor{});
  }
  stream.push(fs);

  uint32_t properties_bitmask;
  uint64_t points_count;

  read_binary(properties_bitmask, stream);
  read_binary(points_count, stream);

  const auto has_colors = (properties_bitmask & COLOR_BIT) != 0;
  const auto has_normals = (properties_bitmask & NORMAL_BIT) != 0;
  const auto has_intensities = (properties_bitmask & INTENSITY_BIT) != 0;
  const auto has_classifications =
      (properties_bitmask & CLASSIFICATION_BIT) != 0;

  std::vector<Vector3<double>> positions;
  std::vector<Vector3<uint8_t>> colors;
  std::vector<Vector3<float>> normals;
  std::vector<uint16_t> intensities;
  std::vector<uint8_t> classifications;

  positions.resize(points_count);
  for (auto &position : positions) {
    read_binary(position, stream);
  }

  if (has_colors) {
    colors.resize(points_count);
    for (auto &color : colors) {
      read_binary(color, stream);
    }
  }

  if (has_normals) {
    normals.resize(points_count);
    for (auto &normal : normals) {
      read_binary(normal, stream);
    }
  }

  if (has_intensities) {
    intensities.resize(points_count);
    for (auto &intensity : intensities) {
      read_binary(intensity, stream);
    }
  }

  if (has_classifications) {
    classifications.resize(points_count);
    for (auto &classification : classifications) {
      read_binary(classification, stream);
    }
  }

  points = PointBuffer{points_count,           std::move(positions),
                       std::move(colors),      std::move(normals),
                       std::move(intensities), std::move(classifications)};
}