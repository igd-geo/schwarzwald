#include "catch.hpp"

#include "io/LASFile.h"
#include "io/LASPersistence.h"

#include <boost/scope_exit.hpp>
#include <random>
#include <sstream>

#define _STR(x) #x
#define STR(x) _STR(x)
#define PRINT_MEMBER(member_name, obj) "[" << STR(member_name) << "] " << obj.member_name << "\n"

namespace Catch {
template<>
struct StringMaker<laszip_header>
{
  static std::string convert(laszip_header const& value)
  {
    std::stringstream ss;
    ss << PRINT_MEMBER(extended_number_of_point_records, value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[0], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[1], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[2], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[3], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[4], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[5], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[6], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[7], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[8], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[9], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[10], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[11], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[12], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[13], value);
    ss << PRINT_MEMBER(extended_number_of_points_by_return[14], value);
    ss << PRINT_MEMBER(file_creation_day, value);
    ss << PRINT_MEMBER(file_creation_year, value);
    ss << PRINT_MEMBER(file_source_ID, value);
    ss << PRINT_MEMBER(generating_software, value);
    ss << PRINT_MEMBER(global_encoding, value);
    ss << PRINT_MEMBER(header_size, value);
    ss << PRINT_MEMBER(max_x, value);
    ss << PRINT_MEMBER(max_y, value);
    ss << PRINT_MEMBER(max_z, value);
    ss << PRINT_MEMBER(min_x, value);
    ss << PRINT_MEMBER(min_y, value);
    ss << PRINT_MEMBER(min_z, value);
    ss << PRINT_MEMBER(number_of_extended_variable_length_records, value);
    ss << PRINT_MEMBER(number_of_point_records, value);
    ss << PRINT_MEMBER(number_of_points_by_return[0], value);
    ss << PRINT_MEMBER(number_of_points_by_return[1], value);
    ss << PRINT_MEMBER(number_of_points_by_return[2], value);
    ss << PRINT_MEMBER(number_of_points_by_return[3], value);
    ss << PRINT_MEMBER(number_of_points_by_return[4], value);
    ss << PRINT_MEMBER(number_of_variable_length_records, value);
    ss << PRINT_MEMBER(offset_to_point_data, value);
    ss << PRINT_MEMBER(point_data_format, value);
    ss << PRINT_MEMBER(point_data_record_length, value);
    ss << PRINT_MEMBER(project_ID_GUID_data_1, value);
    ss << PRINT_MEMBER(project_ID_GUID_data_2, value);
    ss << PRINT_MEMBER(project_ID_GUID_data_3, value);
    ss << PRINT_MEMBER(project_ID_GUID_data_4, value);
    ss << PRINT_MEMBER(start_of_first_extended_variable_length_record, value);
    ss << PRINT_MEMBER(start_of_waveform_data_packet_record, value);
    ss << PRINT_MEMBER(system_identifier, value);
    ss << PRINT_MEMBER(user_data_after_header_size, value);
    ss << PRINT_MEMBER(user_data_in_header_size, value);
    ss << PRINT_MEMBER(version_major, value);
    ss << PRINT_MEMBER(version_minor, value);
    ss << PRINT_MEMBER(x_offset, value);
    ss << PRINT_MEMBER(y_offset, value);
    ss << PRINT_MEMBER(z_offset, value);
    ss << PRINT_MEMBER(x_scale_factor, value);
    ss << PRINT_MEMBER(y_scale_factor, value);
    ss << PRINT_MEMBER(z_scale_factor, value);
    return ss.str();
  }
};
}

static PointBuffer
generate_random_points(size_t count, const AABB& bounds)
{
  std::mt19937 mt;
  std::uniform_real_distribution<double> x_dist{ bounds.min.x, bounds.max.x };
  std::uniform_real_distribution<double> y_dist{ bounds.min.y, bounds.max.y };
  std::uniform_real_distribution<double> z_dist{ bounds.min.z, bounds.max.z };

  std::vector<Vector3<double>> positions;
  positions.reserve(count);
  std::generate_n(std::back_inserter(positions), count, [&]() -> Vector3<double> {
    return { x_dist(mt), y_dist(mt), z_dist(mt) };
  });

  std::uniform_int_distribution<uint8_t> color_dist;
  std::vector<Vector3<uint8_t>> colors;
  colors.reserve(count);
  std::generate_n(std::back_inserter(colors), count, [&]() -> Vector3<uint8_t> {
    return { color_dist(mt), color_dist(mt), color_dist(mt) };
  });

  std::uniform_int_distribution<uint16_t> intensity_dist;
  std::vector<uint16_t> intensities;
  intensities.reserve(count);
  std::generate_n(
    std::back_inserter(intensities), count, [&]() -> uint16_t { return intensity_dist(mt); });

  // The seemingly random values for the following distributions are part of the LAS specification
  // For example, while classifications are stored in 8 bits, only 5 of those bits are used for the
  // actual classification value, while the upper three bits store flags

  std::uniform_int_distribution<uint8_t> classification_dist{ 0, 31 };
  std::vector<uint8_t> classifications;
  classifications.reserve(count);
  std::generate_n(std::back_inserter(classifications), count, [&]() -> uint8_t {
    return classification_dist(mt);
  });

  std::uniform_int_distribution<uint8_t> eof_dist{ 0, 1 };
  std::vector<uint8_t> eof_lines;
  eof_lines.reserve(count);
  std::generate_n(std::back_inserter(eof_lines), count, [&]() -> uint8_t { return eof_dist(mt); });

  std::uniform_real_distribution<double> gps_time_dist;
  std::vector<double> gps_times;
  gps_times.reserve(count);
  std::generate_n(
    std::back_inserter(gps_times), count, [&]() -> double { return gps_time_dist(mt); });

  std::uniform_int_distribution<uint8_t> number_of_returns_dist{ 1, 5 };
  std::vector<uint8_t> number_of_returns;
  number_of_returns.reserve(count);
  std::generate_n(std::back_inserter(number_of_returns), count, [&]() -> uint8_t {
    return number_of_returns_dist(mt);
  });

  std::uniform_int_distribution<uint8_t> return_numbers_dist{ 1, 5 };
  std::vector<uint8_t> return_numbers;
  return_numbers.reserve(count);
  std::generate_n(std::back_inserter(return_numbers), count, [&]() -> uint8_t {
    return return_numbers_dist(mt);
  });

  std::uniform_int_distribution<uint16_t> point_source_ids_dist;
  std::vector<uint16_t> point_source_ids;
  point_source_ids.reserve(count);
  std::generate_n(std::back_inserter(point_source_ids), count, [&]() -> uint8_t {
    return point_source_ids_dist(mt);
  });

  std::uniform_int_distribution<int8_t> scan_angle_ranks_dist;
  std::vector<int8_t> scan_angle_ranks;
  scan_angle_ranks.reserve(count);
  std::generate_n(std::back_inserter(scan_angle_ranks), count, [&]() -> uint8_t {
    return scan_angle_ranks_dist(mt);
  });

  std::uniform_int_distribution<uint8_t> scan_direction_flags_dist{ 0, 1 };
  std::vector<uint8_t> scan_direction_flags;
  scan_direction_flags.reserve(count);
  std::generate_n(std::back_inserter(scan_direction_flags), count, [&]() -> uint8_t {
    return scan_direction_flags_dist(mt);
  });

  std::uniform_int_distribution<uint8_t> user_data_dist;
  std::vector<uint8_t> user_data;
  user_data.reserve(count);
  std::generate_n(
    std::back_inserter(user_data), count, [&]() -> uint8_t { return user_data_dist(mt); });

  return { count,
           std::move(positions),
           std::move(colors),
           {}, // No normals, LAS doesn't support normals
           std::move(intensities),
           std::move(classifications),
           std::move(eof_lines),
           std::move(gps_times),
           std::move(number_of_returns),
           std::move(return_numbers),
           std::move(point_source_ids),
           std::move(scan_direction_flags),
           std::move(scan_angle_ranks),
           std::move(user_data) };
}

static void
compare_points(const PointBuffer& source, const PointBuffer& target)
{
  REQUIRE(source.count() == target.count());
  for (size_t idx = 0; idx < source.count(); ++idx) {
    const auto distance = source.positions()[idx].distanceTo(target.positions()[idx]);
    REQUIRE(distance <= 0.001);

    REQUIRE(source.rgbColors()[idx] == target.rgbColors()[idx]);
    REQUIRE(source.intensities()[idx] == target.intensities()[idx]);
    REQUIRE(source.classifications()[idx] == target.classifications()[idx]);
    REQUIRE(source.edge_of_flight_lines()[idx] == target.edge_of_flight_lines()[idx]);
    REQUIRE(source.gps_times()[idx] == target.gps_times()[idx]);
    REQUIRE(source.number_of_returns()[idx] == target.number_of_returns()[idx]);
    REQUIRE(source.return_numbers()[idx] == target.return_numbers()[idx]);
    REQUIRE(source.point_source_ids()[idx] == target.point_source_ids()[idx]);
    REQUIRE(source.scan_angle_ranks()[idx] == target.scan_angle_ranks()[idx]);
    REQUIRE(source.scan_direction_flags()[idx] == target.scan_direction_flags()[idx]);
    REQUIRE(source.user_data()[idx] == target.user_data()[idx]);
  }
}

bool
operator==(laszip_header const& l, laszip_header const& r)
{
  return std::memcmp(&l, &r, sizeof(laszip_header)) == 0;
}

SCENARIO("LASFile in read-mode")
{
  const size_t count = 1024;
  const auto bounds = AABB{ { 0, 0, 0 }, { 1, 1, 1 } };
  const auto expected_points = generate_random_points(count, bounds);
  PointAttributes attributes;
  attributes.insert(PointAttribute::Position);
  attributes.insert(PointAttribute::RGB);
  attributes.insert(PointAttribute::Intensity);
  attributes.insert(PointAttribute::Classification);
  attributes.insert(PointAttribute::EdgeOfFlightLine);
  attributes.insert(PointAttribute::GPSTime);
  attributes.insert(PointAttribute::NumberOfReturns);
  attributes.insert(PointAttribute::ReturnNumber);
  attributes.insert(PointAttribute::PointSourceID);
  attributes.insert(PointAttribute::ScanAngleRank);
  attributes.insert(PointAttribute::ScanDirectionFlag);
  attributes.insert(PointAttribute::UserData);

  fs::path file_path = "./tmp_testlasfile.las";
  LASPersistence las_persistence{ ".", attributes };
  las_persistence.persist_points(expected_points, bounds, "tmp_testlasfile");

  BOOST_SCOPE_EXIT(&file_path) { fs::remove(file_path); }
  BOOST_SCOPE_EXIT_END

  WHEN("The file is opened in read-mode")
  {
    LASFile file{ file_path, LASFile::OpenMode::Read };

    THEN("The file metadata is loaded correctly")
    {
      REQUIRE(pc::get_bounds(file) == bounds);
      REQUIRE(pc::get_point_count(file) == count);
    }

    THEN("The begin and end iterators point to a correct range")
    {
      const auto begin = std::cbegin(file);
      const auto end = std::cend(file);
      REQUIRE(begin.distance_to_end() == count);
      REQUIRE(std::distance(begin, end) == count);
    }

    WHEN("The points are read using iterators")
    {
      const auto begin = std::cbegin(file);
      PointBuffer actual_points;
      pc::read_points(begin, count, pc::metadata(file), attributes, actual_points);

      THEN("All points are correct")
      {
        REQUIRE(actual_points.count() == count);
        compare_points(expected_points, actual_points);
      }
    }
  }
}

SCENARIO("LASFile in write-mode")
{
  const size_t count = 1024;
  const auto bounds = AABB{ { 0, 0, 0 }, { 1, 1, 1 } };
  const auto expected_points = generate_random_points(count, bounds);
  PointAttributes attributes;
  attributes.insert(PointAttribute::Position);
  attributes.insert(PointAttribute::RGB);
  attributes.insert(PointAttribute::Intensity);
  attributes.insert(PointAttribute::Classification);
  attributes.insert(PointAttribute::EdgeOfFlightLine);
  attributes.insert(PointAttribute::GPSTime);
  attributes.insert(PointAttribute::NumberOfReturns);
  attributes.insert(PointAttribute::ReturnNumber);
  attributes.insert(PointAttribute::PointSourceID);
  attributes.insert(PointAttribute::ScanAngleRank);
  attributes.insert(PointAttribute::ScanDirectionFlag);
  attributes.insert(PointAttribute::UserData);

  fs::path file_path = "./tmp_testlasfile.las";

  GIVEN("A LASFile opened in write-mode")
  {
    LASFile file{ file_path, LASFile::OpenMode::Write };

    WHEN("Metadata is written")
    {
      auto meta = file.get_metadata();
      meta.number_of_point_records = count;
      meta.number_of_points_by_return[0] = count;
      meta.x_offset = bounds.min.x;
      meta.y_offset = bounds.min.y;
      meta.z_offset = bounds.min.z;
      meta.min_x = bounds.min.x;
      meta.min_y = bounds.min.y;
      meta.min_z = bounds.min.z;
      meta.max_x = bounds.max.x;
      meta.max_y = bounds.max.y;
      meta.max_z = bounds.max.z;
      meta.x_scale_factor = meta.y_scale_factor = meta.z_scale_factor = 0.001;

      meta.offset_to_point_data = meta.header_size;
      meta.number_of_variable_length_records = 0;
      meta.point_data_format = 3;
      meta.point_data_record_length = 34;
      file.set_metadata(meta);

      THEN("Correct metadata can be retrieved from the file")
      {
        file.close();
        file.open(file_path, LASFile::OpenMode::Read);
        const auto new_meta = file.get_metadata();
        REQUIRE(new_meta == meta);
      }

      // We can only write points if the header has the corresponding point count set
      WHEN("Points are written using output iterator")
      {

        auto out_iter = std::begin(file);
        for (auto point : expected_points) {
          laszip_point las_point = {};
          las_point.X = static_cast<laszip_I32>(
            std::round((point.position().x - bounds.min.x) / meta.x_scale_factor));
          las_point.Y = static_cast<laszip_I32>(
            std::round((point.position().y - bounds.min.y) / meta.y_scale_factor));
          las_point.Z = static_cast<laszip_I32>(
            std::round((point.position().z - bounds.min.z) / meta.z_scale_factor));

          las_point.rgb[0] = point.rgbColor()->x << 8;
          las_point.rgb[1] = point.rgbColor()->y << 8;
          las_point.rgb[2] = point.rgbColor()->z << 8;

          las_point.intensity = *point.intensity();
          las_point.classification = *point.classification();
          las_point.edge_of_flight_line = *point.edge_of_flight_line();
          las_point.gps_time = *point.gps_time();
          las_point.number_of_returns = *point.number_of_returns();
          las_point.return_number = *point.return_number();
          las_point.point_source_ID = *point.point_source_id();
          las_point.scan_angle_rank = *point.scan_angle_rank();
          las_point.scan_direction_flag = *point.scan_direction_flag();
          las_point.user_data = *point.user_data();

          out_iter = las_point;
        }
        file.close();
        file.open(file_path, LASFile::OpenMode::Read);

        THEN("Correct points can be retrieved from the file")
        {
          const auto begin = std::cbegin(file);
          PointBuffer actual_points;
          pc::read_points(begin, count, pc::metadata(file), attributes, actual_points);
          compare_points(expected_points, actual_points);
        }
      }
    }
  }

  fs::remove(file_path);
}