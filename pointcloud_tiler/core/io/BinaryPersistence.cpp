#include "io/BinaryPersistence.h"

#include "util/Transformation.h"

#include <experimental/filesystem>

namespace bio = boost::iostreams;

static_assert(sizeof(uint8_t) == 1,
              "uint8_t is not 8-bit wide on this machine. This breaks BinaryPersistence encoding!");
static_assert(
  sizeof(uint16_t) == 2,
  "uint16_t is not 16-bit wide on this machine. This breaks BinaryPersistence encoding!");
static_assert(
  sizeof(uint32_t) == 4,
  "uint32_t is not 32-bit wide on this machine. This breaks BinaryPersistence encoding!");
static_assert(
  sizeof(uint64_t) == 8,
  "uint64_t is not 64-bit wide on this machine. This breaks BinaryPersistence encoding!");

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

  const uint32_t properties_bitmask =
    (points.hasColors() ? COLOR_BIT : 0u) | (points.hasNormals() ? NORMAL_BIT : 0u) |
    (points.hasIntensities() ? INTENSITY_BIT : 0u) |
    (points.hasClassifications() ? CLASSIFICATION_BIT : 0u) |
    (points.has_edge_of_flight_lines() ? EDGE_OF_FLIGHT_LINE_BIT : 0u) |
    (points.has_gps_times() ? GPS_TIME_BIT : 0u) |
    (points.has_number_of_returns() ? NUMBER_OF_RETURN_BIT : 0u) |
    (points.has_return_numbers() ? RETURN_NUMBER_BIT : 0u) |
    (points.has_point_source_ids() ? POINT_SOURCE_ID_BIT : 0u) |
    (points.has_scan_angle_ranks() ? SCAN_ANGLE_RANK_BIT : 0u) |
    (points.has_scan_direction_flags() ? SCAN_DIRECTION_FLAG_BIT : 0u) |
    (points.has_user_data() ? USER_DATA_BIT : 0u);

  write_binary(properties_bitmask, stream);
  write_binary(static_cast<uint64_t>(points.count()), stream);

  // TODO Why did I implement the writing with a loop over individual points, if the PointBuffer
  // exposes all attributes already in array-form?

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

  if (points.has_edge_of_flight_lines()) {
    for (auto point : points) {
      write_binary(*point.edge_of_flight_line(), stream);
    }
  }

  if (points.has_gps_times()) {
    for (auto point : points) {
      write_binary(*point.gps_time(), stream);
    }
  }

  if (points.has_number_of_returns()) {
    for (auto point : points) {
      write_binary(*point.number_of_returns(), stream);
    }
  }

  if (points.has_return_numbers()) {
    for (auto point : points) {
      write_binary(*point.return_number(), stream);
    }
  }

  if (points.has_point_source_ids()) {
    for (auto point : points) {
      write_binary(*point.point_source_id(), stream);
    }
  }

  if (points.has_scan_angle_ranks()) {
    for (auto point : points) {
      write_binary(*point.scan_angle_rank(), stream);
    }
  }

  if (points.has_scan_direction_flags()) {
    for (auto point : points) {
      write_binary(*point.scan_direction_flag(), stream);
    }
  }

  if (points.has_user_data()) {
    for (auto point : points) {
      write_binary(*point.user_data(), stream);
    }
  }

  // stream << std::flush;
  stream.pop();
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
  const auto has_edge_of_flight_lines = (properties_bitmask & EDGE_OF_FLIGHT_LINE_BIT) != 0;
  const auto has_gps_times = (properties_bitmask & GPS_TIME_BIT) != 0;
  const auto has_number_of_returns = (properties_bitmask & NUMBER_OF_RETURN_BIT) != 0;
  const auto has_return_numbers = (properties_bitmask & RETURN_NUMBER_BIT) != 0;
  const auto has_point_source_ids = (properties_bitmask & POINT_SOURCE_ID_BIT) != 0;
  const auto has_scan_angle_ranks = (properties_bitmask & SCAN_ANGLE_RANK_BIT) != 0;
  const auto has_scan_direction_flags = (properties_bitmask & SCAN_DIRECTION_FLAG_BIT) != 0;
  const auto has_user_data = (properties_bitmask & USER_DATA_BIT) != 0;

  std::vector<Vector3<double>> positions;
  std::vector<Vector3<uint8_t>> colors;
  std::vector<Vector3<float>> normals;
  std::vector<uint16_t> intensities;
  std::vector<uint8_t> classifications;
  std::vector<uint8_t> edge_of_flight_lines;
  std::vector<double> gps_times;
  std::vector<uint8_t> number_of_returns;
  std::vector<uint8_t> return_numbers;
  std::vector<uint16_t> point_source_ids;
  std::vector<int8_t> scan_angle_ranks;
  std::vector<uint8_t> scan_direction_flags;
  std::vector<uint8_t> user_data;

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

  if (has_edge_of_flight_lines) {
    edge_of_flight_lines.resize(points_count);
    for (auto& edge_of_flight_line : edge_of_flight_lines) {
      read_binary(edge_of_flight_line, stream);
    }
  }

  if (has_gps_times) {
    gps_times.resize(points_count);
    for (auto& gps_time : gps_times) {
      read_binary(gps_time, stream);
    }
  }

  if (has_number_of_returns) {
    number_of_returns.resize(points_count);
    for (auto& number : number_of_returns) {
      read_binary(number, stream);
    }
  }

  if (has_return_numbers) {
    return_numbers.resize(points_count);
    for (auto& return_number : return_numbers) {
      read_binary(return_number, stream);
    }
  }

  if (has_point_source_ids) {
    point_source_ids.resize(points_count);
    for (auto& id : point_source_ids) {
      read_binary(id, stream);
    }
  }

  if (has_scan_angle_ranks) {
    scan_angle_ranks.resize(points_count);
    for (auto& rank : scan_angle_ranks) {
      read_binary(rank, stream);
    }
  }

  if (has_scan_direction_flags) {
    scan_direction_flags.resize(points_count);
    for (auto& flag : scan_direction_flags) {
      read_binary(flag, stream);
    }
  }

  if (has_user_data) {
    user_data.resize(points_count);
    for (auto& data : user_data) {
      read_binary(data, stream);
    }
  }

  points = { points_count,
             std::move(positions),
             std::move(colors),
             std::move(normals),
             std::move(intensities),
             std::move(classifications),
             std::move(edge_of_flight_lines),
             std::move(gps_times),
             std::move(number_of_returns),
             std::move(return_numbers),
             std::move(point_source_ids),
             std::move(scan_direction_flags),
             std::move(scan_angle_ranks),
             std::move(user_data) };
}

bool
BinaryPersistence::node_exists(const std::string& node_name) const
{
  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  return fs::exists(file_path);
}