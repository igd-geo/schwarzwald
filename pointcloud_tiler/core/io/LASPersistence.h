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
      std::cerr << "Could not create LAS writer for node " << node_name << std::endl;
      return;
    }

    BOOST_SCOPE_EXIT_TPL(&laswriter) { laszip_destroy(laswriter); }
    BOOST_SCOPE_EXIT_END

    laszip_header* las_header;
    if (laszip_get_header_pointer(laswriter, &las_header)) {
      std::cerr << "Could not write LAS header for node " << node_name << std::endl;
      return;
    }

    // las_header->file_source_ID = 0;
    // las_header->global_encoding = 0;
    // las_header->project_ID_GUID_data_1 = 0;
    // las_header->project_ID_GUID_data_2 = 0;
    // las_header->project_ID_GUID_data_3 = 0;
    // std::memset(las_header->project_ID_GUID_data_4, 0, 8);
    // las_header->header_size = sizeof(laszip_header);
    las_header->number_of_point_records = points_count;
    las_header->number_of_points_by_return[0] = points_count;
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

    las_header->x_scale_factor = las_header->y_scale_factor = las_header->z_scale_factor =
      compute_las_scale_from_bounds(bounds);

    const auto file_path = concat(_work_dir, "/", node_name, _file_extension);
    if (laszip_open_writer(laswriter, file_path.c_str(), (_compressed == Compressed::Yes))) {
      std::cerr << "Could not write LAS file for node " << node_name << std::endl;
      return;
    }

    BOOST_SCOPE_EXIT_TPL(&laswriter) { laszip_close_writer(laswriter); }
    BOOST_SCOPE_EXIT_END

    laszip_point* laspoint;
    if (laszip_get_point_pointer(laswriter, &laspoint)) {
      std::cerr << "Could not write LAS points for node " << node_name << std::endl;
      return;
    }

    std::for_each(points_begin, points_end, [&](const auto& point_ref) {
      const auto pos = point_ref.position();
      laszip_F64 coordinates[3] = { pos.x, pos.y, pos.z };
      if (laszip_set_coordinates(laswriter, coordinates)) {
        std::cerr << "Could not set coordinates for LAS point at node " << node_name << std::endl;
        return;
      }

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

      if (laszip_write_point(laswriter)) {
        std::cerr << "Could not write LAS point for node " << node_name << std::endl;
        return;
      }
    });
  }

  void persist_points(PointBuffer const& points, const AABB& bounds, const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

  bool node_exists(const std::string& node_name) const;

private:
  std::string _work_dir;
  const PointAttributes& _point_attributes;
  Compressed _compressed;
  std::string _file_extension;
};