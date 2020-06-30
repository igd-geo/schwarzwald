#pragma once

#include "datastructures/PointBuffer.h"
#include "io/LASFile.h"
#include "laszip_api.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "util/Definitions.h"
#include "util/stuff.h"

#include <boost/scope_exit.hpp>

struct SRSTransformHelper;

double
compute_las_scale_from_bounds(const AABB& bounds);

/**
 * Sink for writing LAS files
 */
struct LASPersistence
{
  LASPersistence(const std::string& work_dir,
                 const PointAttributes& point_attributes,
                 Compressed compressed = Compressed::No);
  ~LASPersistence();

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    const auto points_count = std::distance(points_begin, points_end);
    if (!points_count)
      return;

    laszip_POINTER laswriter = nullptr;
    laszip_create(&laswriter);

    if (!laswriter) {
      std::cerr << "Could not create LAS writer for node " << node_name
                << std::endl;
      return;
    }

    BOOST_SCOPE_EXIT_TPL(&laswriter) { laszip_destroy(laswriter); }
    BOOST_SCOPE_EXIT_END

    laszip_header* las_header;
    if (laszip_get_header_pointer(laswriter, &las_header)) {
      std::cerr << "Could not write LAS header for node " << node_name
                << std::endl;
      return;
    }

    const auto has_gps_time = (*points_begin).gps_time() != nullptr;
    const auto has_colors = (*points_begin).rgbColor() != nullptr;

    const auto point_data_format =
      static_cast<uint8_t>((has_gps_time ? 1 : 0) + (has_colors ? 2 : 0));
    const auto point_record_length =
      static_cast<uint16_t>(20 + (has_gps_time ? 8 : 0) + (has_colors ? 6 : 0));

    // las_header->file_source_ID = 0;
    // las_header->global_encoding = 0;
    // las_header->project_ID_GUID_data_1 = 0;
    // las_header->project_ID_GUID_data_2 = 0;
    // las_header->project_ID_GUID_data_3 = 0;
    // std::memset(las_header->project_ID_GUID_data_4, 0, 8);
    // las_header->header_size = sizeof(laszip_header);
    las_header->number_of_point_records = points_count;
    las_header->number_of_points_by_return[0] = points_count;
    las_header->number_of_points_by_return[1] =
      las_header->number_of_points_by_return[2] =
        las_header->number_of_points_by_return[3] =
          las_header->number_of_points_by_return[4] = 0;
    las_header->version_major = 1;
    las_header->version_minor = 2;
    std::memcpy(las_header->generating_software,
                "pointcloud_tiler",
                sizeof("pointcloud_tiler"));
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

    las_header->x_scale_factor = las_header->y_scale_factor =
      las_header->z_scale_factor = compute_las_scale_from_bounds(bounds);

    const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
    if (laszip_open_writer(
          laswriter, file_path.c_str(), (_compressed == Compressed::Yes))) {
      std::cerr << "Could not write LAS file for node " << node_name
                << std::endl;
      return;
    }

    BOOST_SCOPE_EXIT_TPL(&laswriter) { laszip_close_writer(laswriter); }
    BOOST_SCOPE_EXIT_END

    laszip_point* laspoint;
    if (laszip_get_point_pointer(laswriter, &laspoint)) {
      std::cerr << "Could not write LAS points for node " << node_name
                << std::endl;
      return;
    }

    std::for_each(points_begin, points_end, [&](const auto& point_ref) {
      const auto pos = point_ref.position();
      laszip_F64 coordinates[3] = { pos.x, pos.y, pos.z };
      if (laszip_set_coordinates(laswriter, coordinates)) {
        std::cerr << "Could not set coordinates for LAS point at node "
                  << node_name << std::endl;
        return;
      }

      const auto rgb = point_ref.rgbColor();
      if (rgb) {
        // Shifting by 8 bits to get 16-bit color values. This is per the LAS
        // specification, which states: "The Red, Green, Blue values should
        // always be normalized to 16 bit values. For example, when encoding an
        // 8 bit per channel pixel, multiply each channel value by 256 prior to
        // storage in these fields. This normalization allows color values from
        // different camera bit depths to be accurately merged." [LAS
        // Specification, Version 1.4 - R13, 15 July 2013]
        // (https://www.asprs.org/wp-content/uploads/2010/12/LAS_1_4_r13.pdf)
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
        std::cerr << "Could not write LAS point for node " << node_name
                  << std::endl;
        return;
      }
    });
  }

  void persist_points(PointBuffer const& points,
                      const AABB& bounds,
                      const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

  bool node_exists(const std::string& node_name) const;

  inline bool is_lossless() const { return false; }

private:
  std::string _work_dir;
  const PointAttributes& _point_attributes;
  Compressed _compressed;
  std::string _file_extension;
};