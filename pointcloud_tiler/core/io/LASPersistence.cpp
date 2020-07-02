#include "io/LASPersistence.h"

#include "io/LASFile.h"
#include "laszip_api.h"
#include "util/Transformation.h"
#include "util/stuff.h"

#include <experimental/filesystem>

#include <boost/scope_exit.hpp>

/**
 * Computes the scale factor for the LAS file based on the size of the given
 * bounding box. This code is adopted straight from Potree
 */
double
compute_las_scale_from_bounds(const AABB& bounds)
{
  const auto bounds_diagonal = bounds.extent().length();
  if (bounds_diagonal > 1'000'000) {
    return 0.01;
  } else if (bounds_diagonal > 100'000) {
    return 0.001;
  } else if (bounds_diagonal > 1) {
    return 0.001;
  }
  return 0.0001;
}

PointAttributes
LASPersistence::supported_output_attributes()
{
  return { PointAttribute::Classification, PointAttribute::EdgeOfFlightLine,
           PointAttribute::GPSTime,        PointAttribute::Intensity,
           PointAttribute::Normal,         PointAttribute::NumberOfReturns,
           PointAttribute::PointSourceID,  PointAttribute::Position,
           PointAttribute::ReturnNumber,   PointAttribute::RGB,
           PointAttribute::ScanAngleRank,  PointAttribute::ScanDirectionFlag,
           PointAttribute::UserData };
}

LASPersistence::LASPersistence(const std::string& work_dir,
                               const PointAttributes& point_attributes,
                               Compressed compressed)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _compressed(compressed)
  , _file_extension(compressed == Compressed::Yes ? ".laz" : ".las")
{}

LASPersistence::~LASPersistence() {}

void
LASPersistence::persist_points(PointBuffer const& points,
                               const AABB& bounds,
                               const std::string& node_name)
{
  laszip_POINTER laswriter = nullptr;
  laszip_create(&laswriter);

  if (!laswriter) {
    std::cerr << "Could not create LAS writer for node " << node_name << std::endl;
    return;
  }

  BOOST_SCOPE_EXIT(&laswriter) { laszip_destroy(laswriter); }
  BOOST_SCOPE_EXIT_END

  laszip_header* las_header;
  if (laszip_get_header_pointer(laswriter, &las_header)) {
    std::cerr << "Could not write LAS header for node " << node_name << std::endl;
    return;
  }

  const auto point_data_format =
    static_cast<uint8_t>((points.has_gps_times() ? 1 : 0) + (points.hasColors() ? 2 : 0));
  const auto point_record_length =
    static_cast<uint16_t>(20 + (points.has_gps_times() ? 8 : 0) + (points.hasColors() ? 6 : 0));

  // las_header->file_source_ID = 0;
  // las_header->global_encoding = 0;
  // las_header->project_ID_GUID_data_1 = 0;
  // las_header->project_ID_GUID_data_2 = 0;
  // las_header->project_ID_GUID_data_3 = 0;
  // std::memset(las_header->project_ID_GUID_data_4, 0, 8);
  // las_header->header_size = sizeof(laszip_header);
  las_header->number_of_point_records = points.count();
  las_header->number_of_points_by_return[0] = points.count();
  las_header->number_of_points_by_return[1] = las_header->number_of_points_by_return[2] =
    las_header->number_of_points_by_return[3] = las_header->number_of_points_by_return[4] = 0;
  las_header->version_major = 1;
  las_header->version_minor = 2;
  std::memcpy(las_header->generating_software, "pointcloud_tiler", sizeof("pointcloud_tiler"));
  las_header->offset_to_point_data = las_header->header_size;
  las_header->number_of_variable_length_records = 0;
  las_header->point_data_format = point_data_format;
  las_header->point_data_record_length = point_record_length;

  las_header->x_offset = bounds.min.x;
  las_header->y_offset = bounds.min.y;
  las_header->z_offset = bounds.min.z;
  las_header->min_x = bounds.min.x; // - local_offset_to_world.x;
  las_header->min_y = bounds.min.y; // - local_offset_to_world.y;
  las_header->min_z = bounds.min.z; // - local_offset_to_world.z;
  las_header->max_x = bounds.max.x; // - local_offset_to_world.x;
  las_header->max_y = bounds.max.y; // - local_offset_to_world.y;
  las_header->max_z = bounds.max.z; // - local_offset_to_world.z;

  las_header->x_scale_factor = las_header->y_scale_factor = las_header->z_scale_factor =
    compute_las_scale_from_bounds(bounds);

  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  if (laszip_open_writer(laswriter, file_path.c_str(), (_compressed == Compressed::Yes))) {
    std::cerr << "Could not write LAS file for node " << node_name << std::endl;
    return;
  }

  BOOST_SCOPE_EXIT(&laswriter) { laszip_close_writer(laswriter); }
  BOOST_SCOPE_EXIT_END

  laszip_point* laspoint;
  if (laszip_get_point_pointer(laswriter, &laspoint)) {
    std::cerr << "Could not write LAS points for node " << node_name << std::endl;
    return;
  }

  for (const auto point_ref : points) {
    const auto pos = point_ref.position();
    laszip_F64 coordinates[3] = { pos.x, pos.y, pos.z };
    if (laszip_set_coordinates(laswriter, coordinates)) {
      std::cerr << "Could not set coordinates for LAS point at node " << node_name << std::endl;
      return;
    }

    const auto rgb = point_ref.rgbColor();
    if (rgb) {
      // See comment in templated version of persist_points in LASPersistence.h for
      // an explanation of the bit-shift
      laspoint->rgb[0] = rgb->x << 8;
      laspoint->rgb[1] = rgb->y << 8;
      laspoint->rgb[2] = rgb->z << 8;
    }

    const auto intensity = point_ref.intensity();
    if (intensity) {
      laspoint->intensity = *intensity;
    }

    const auto classification = point_ref.classification();
    if (classification) {
      laspoint->classification = *classification;
    }

    const auto eof_line = point_ref.edge_of_flight_line();
    if (eof_line) {
      laspoint->edge_of_flight_line = *eof_line;
    }

    const auto gps_time = point_ref.gps_time();
    if (gps_time) {
      laspoint->gps_time = *gps_time;
    }

    const auto number_of_returns = point_ref.number_of_returns();
    if (number_of_returns) {
      laspoint->number_of_returns = *number_of_returns;
    }

    const auto return_number = point_ref.return_number();
    if (return_number) {
      laspoint->return_number = *return_number;
    }

    const auto point_source_id = point_ref.point_source_id();
    if (point_source_id) {
      laspoint->point_source_ID = *point_source_id;
    }

    const auto scan_angle_rank = point_ref.scan_angle_rank();
    if (scan_angle_rank) {
      laspoint->scan_angle_rank = *scan_angle_rank;
    }

    const auto scan_direction_flag = point_ref.scan_direction_flag();
    if (scan_direction_flag) {
      laspoint->scan_direction_flag = *scan_direction_flag;
    }

    const auto user_data = point_ref.user_data();
    if (user_data) {
      laspoint->user_data = *user_data;
    }

    if (laszip_write_point(laswriter)) {
      std::cerr << "Could not write LAS point for node " << node_name << std::endl;
      return;
    }
  }
}

void
LASPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  if (!std::experimental::filesystem::exists(file_path))
    return;
  LASFile las_file{ file_path, LASFile::OpenMode::Read };
  pc::read_points(
    std::cbegin(las_file), las_file.size(), las_file.get_metadata(), _point_attributes, points);
}

bool
LASPersistence::node_exists(const std::string& node_name) const
{
  const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
  return fs::exists(file_path);
}