#include "catch.hpp"

#include "io/LASFile.h"

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

static void
write_tmp_file(fs::path const& path, AABB const& bounds, PointBuffer const& points)
{
  // Handrolled LAS file writing used for verification. It would be even better to have an existing
  // file that these tests read, but that is harder to set up
  laszip_POINTER laswriter = nullptr;
  REQUIRE(laszip_create(&laswriter) == 0);

  laszip_header* las_header;
  REQUIRE(laszip_get_header_pointer(laswriter, &las_header) == 0);

  las_header->number_of_point_records = points.count();
  las_header->number_of_points_by_return[0] = points.count();
  las_header->number_of_points_by_return[1] = las_header->number_of_points_by_return[2] =
    las_header->number_of_points_by_return[3] = las_header->number_of_points_by_return[4] = 0;
  las_header->version_major = 1;
  las_header->version_minor = 2;
  std::memcpy(las_header->generating_software, "pointcloud_tiler", sizeof("pointcloud_tiler"));
  las_header->offset_to_point_data = las_header->header_size;
  las_header->number_of_variable_length_records = 0;
  las_header->point_data_format = 2;
  las_header->point_data_record_length = 26;

  las_header->x_offset = bounds.min.x;
  las_header->y_offset = bounds.min.y;
  las_header->z_offset = bounds.min.z;
  las_header->min_x = bounds.min.x; // - local_offset_to_world.x;
  las_header->min_y = bounds.min.y; // - local_offset_to_world.y;
  las_header->min_z = bounds.min.z; // - local_offset_to_world.z;
  las_header->max_x = bounds.max.x; // - local_offset_to_world.x;
  las_header->max_y = bounds.max.y; // - local_offset_to_world.y;
  las_header->max_z = bounds.max.z; // - local_offset_to_world.z;

  las_header->x_scale_factor = las_header->y_scale_factor = las_header->z_scale_factor = 0.001;

  laszip_open_writer(laswriter, path.c_str(), false);

  laszip_point* laspoint;
  REQUIRE(laszip_get_point_pointer(laswriter, &laspoint) == 0);

  for (const auto& point_ref : points) {
    const auto pos = point_ref.position();
    laszip_F64 coordinates[3] = { pos.x, pos.y, pos.z };
    REQUIRE(laszip_set_coordinates(laswriter, coordinates) == 0);

    const auto rgb = point_ref.rgbColor();
    if (rgb) {
      laspoint->rgb[0] = rgb->x;
      laspoint->rgb[1] = rgb->y;
      laspoint->rgb[2] = rgb->z;
      laspoint->rgb[3] = static_cast<uint8_t>(255);
    }

    const auto intensity = point_ref.intensity();
    if (intensity) {
      laspoint->intensity = *intensity;
    }

    const auto classification = point_ref.classification();
    if (classification) {
      laspoint->classification = *classification;
    }

    REQUIRE(laszip_write_point(laswriter) == 0);
  }

  auto ec = laszip_close_writer(laswriter);
  REQUIRE(ec == 0);
  ec = laszip_destroy(laswriter);
  REQUIRE(ec == 0);
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

  return { count, std::move(positions) };
}

static void
compare_points(const PointBuffer& source, const PointBuffer& target)
{
  REQUIRE(source.count() == target.count());
  for (size_t idx = 0; idx < source.count(); ++idx) {
    const auto distance = source.positions()[idx].distanceTo(target.positions()[idx]);
    REQUIRE(distance <= 0.001);
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
  attributes.push_back(attributes::POSITION_CARTESIAN);
  fs::path file_path = "./tmp_testlasfile.las";
  write_tmp_file(file_path, bounds, expected_points);

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

  fs::remove(file_path);
}

SCENARIO("LASFile in write-mode")
{
  const size_t count = 1024;
  const auto bounds = AABB{ { 0, 0, 0 }, { 1, 1, 1 } };
  const auto expected_points = generate_random_points(count, bounds);
  PointAttributes attributes;
  attributes.push_back(attributes::POSITION_CARTESIAN);
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
      meta.point_data_format = 2;
      meta.point_data_record_length = 26;
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