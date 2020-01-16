#include "io/BinaryPersistence.h"

#include "LASPointReader.h"
#include "Transformation.h"
#include "io/io_util.h"
#include "laszip_api.h"

#include <experimental/filesystem>

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/scope_exit.hpp>

namespace bio = boost::iostreams;

constexpr static uint32_t COLOR_BIT = (1 << 0);
constexpr static uint32_t NORMAL_BIT = (1 << 1);
constexpr static uint32_t INTENSITY_BIT = (1 << 2);
constexpr static uint32_t CLASSIFICATION_BIT = (1 << 3);

BinaryPersistence::BinaryPersistence(const std::string& work_dir,
                                     const PointAttributes& point_attributes,
                                     Compressed compressed)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _compressed(compressed)
  , _file_extension(compressed == Compressed::Yes ? ".binz" : ".bin")
{}

BinaryPersistence::~BinaryPersistence() {}

void
BinaryPersistence::persist_points(gsl::span<PointBuffer::PointReference> points,
                                  const AABB& bounds,
                                  const std::string& node_name)
{
  if (!points.size())
    return;

  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);

  bio::file_sink fs{ file_path, std::ios::out | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not write points file " << file_path << std::endl;
    return;
  }

  bio::filtering_ostream stream;
  if (_compressed == Compressed::Yes) {
    bio::zlib_params zlib_params;
    zlib_params.level = bio::zlib::best_speed;
    stream.push(bio::zlib_compressor{ zlib_params, 1 << 18 });
  }
  stream.push(fs);

  const auto has_colors = points[0].rgbColor() != nullptr;
  const auto has_normals = points[0].normal() != nullptr;
  const auto has_intensities = points[0].intensity() != nullptr;
  const auto has_classifications = points[0].classification() != nullptr;

  const uint32_t properties_bitmask =
    (has_colors ? COLOR_BIT : 0u) | (has_normals ? NORMAL_BIT : 0u) |
    (has_intensities ? INTENSITY_BIT : 0u) | (has_classifications ? CLASSIFICATION_BIT : 0u);

  write_binary(properties_bitmask, stream);
  write_binary(static_cast<uint64_t>(points.size()), stream);

  for (auto& point : points) {
    write_binary(point.position(), stream);
  }

  if (has_colors) {
    for (auto& point : points) {
      write_binary(*point.rgbColor(), stream);
    }
  }

  if (has_normals) {
    for (auto& point : points) {
      write_binary(*point.normal(), stream);
    }
  }

  if (has_intensities) {
    for (auto& point : points) {
      write_binary(*point.intensity(), stream);
    }
  }

  if (has_classifications) {
    for (auto& point : points) {
      write_binary(*point.classification(), stream);
    }
  }

  // stream << std::flush;
  stream.pop();
}

void
BinaryPersistence::persist_points(PointBuffer const& points,
                                  const AABB& bounds,
                                  const std::string& node_name)
{
  if (!points.count())
    throw std::runtime_error{ "No points selected" };

  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);

  bio::file_sink fs{ file_path, std::ios::out | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not write points file " << file_path << std::endl;
    return;
  }

  bio::filtering_ostream stream;
  if (_compressed == Compressed::Yes) {
    bio::zlib_params zlib_params;
    zlib_params.level = bio::zlib::best_speed;
    stream.push(bio::zlib_compressor{ zlib_params, 1 << 18 });
  }
  stream.push(fs);

  const uint32_t properties_bitmask = (points.hasColors() ? COLOR_BIT : 0u) |
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

void
BinaryPersistence::persist_indices(gsl::span<MortonIndex64> indices, const std::string& node_name)
{
  // TODO REFACTOR This is identical to Cesium3DTilesPersistence
  const auto file_path = concat(_work_dir, "/", node_name, ".idx");
  std::ofstream fs{ file_path, std::ios::out | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not write index file " << file_path << std::endl;
    return;
  }

  const uint64_t num_indices = indices.size();
  fs.write(reinterpret_cast<char const*>(&num_indices), sizeof(uint64_t));
  for (auto idx : indices) {
    const uint64_t idx_val = idx.get();
    fs.write(reinterpret_cast<char const*>(&idx_val), sizeof(uint64_t));
  }

  fs.flush();
  fs.close();
}

void
BinaryPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  if (!std::experimental::filesystem::exists(file_path))
    return;

  bio::file_source fs{ file_path, std::ios::in | std::ios::binary };
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
  const auto has_classifications = (properties_bitmask & CLASSIFICATION_BIT) != 0;

  std::vector<Vector3<double>> positions;
  std::vector<Vector3<uint8_t>> colors;
  std::vector<Vector3<float>> normals;
  std::vector<uint16_t> intensities;
  std::vector<uint8_t> classifications;

  positions.resize(points_count);
  for (auto& position : positions) {
    read_binary(position, stream);
  }

  if (has_colors) {
    colors.resize(points_count);
    for (auto& color : colors) {
      read_binary(color, stream);
    }
  }

  if (has_normals) {
    normals.resize(points_count);
    for (auto& normal : normals) {
      read_binary(normal, stream);
    }
  }

  if (has_intensities) {
    intensities.resize(points_count);
    for (auto& intensity : intensities) {
      read_binary(intensity, stream);
    }
  }

  if (has_classifications) {
    classifications.resize(points_count);
    for (auto& classification : classifications) {
      read_binary(classification, stream);
    }
  }

  points = PointBuffer{ points_count,       std::move(positions),   std::move(colors),
                        std::move(normals), std::move(intensities), std::move(classifications) };
}

void
BinaryPersistence::retrieve_indices(const std::string& node_name,
                                    std::vector<MortonIndex64>& indices)
{
  static_assert(sizeof(MortonIndex64) == sizeof(uint64_t), "");

  const auto file_path = concat(_work_dir, "/", node_name, ".idx");
  std::ifstream fs{ file_path, std::ios::in | std::ios::binary };
  if (!fs.is_open()) {
    std::cerr << "Could not read index file " << file_path << std::endl;
    return;
  }

  fs.unsetf(std::ios::skipws);

  fs.seekg(0, std::ios::end);
  auto fileSize = fs.tellg();
  fs.seekg(0, std::ios::beg);

  std::vector<char> rawData;
  rawData.reserve(fileSize);

  rawData.insert(rawData.begin(), std::istream_iterator<char>(fs), std::istream_iterator<char>());

  const auto indices_begin =
    reinterpret_cast<MortonIndex64 const*>(rawData.data() + sizeof(size_t));
  const auto indices_end = reinterpret_cast<MortonIndex64 const*>(rawData.data() + rawData.size());

  indices = { indices_begin, indices_end };
}