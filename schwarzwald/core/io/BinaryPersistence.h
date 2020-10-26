#pragma once

#include "datastructures/PointBuffer.h"
#include "io/io_util.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "util/Definitions.h"
#include "util/stuff.h"

#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/scope_exit.hpp>

struct SRSTransformHelper;

/**
 * Sink for writing binary point files
 */
struct BinaryPersistence
{

  constexpr static uint32_t COLOR_BIT = (1 << 0);
  constexpr static uint32_t NORMAL_BIT = (1 << 1);
  constexpr static uint32_t INTENSITY_BIT = (1 << 2);
  constexpr static uint32_t CLASSIFICATION_BIT = (1 << 3);
  constexpr static uint32_t EDGE_OF_FLIGHT_LINE_BIT = (1 << 4);
  constexpr static uint32_t GPS_TIME_BIT = (1 << 5);
  constexpr static uint32_t NUMBER_OF_RETURN_BIT = (1 << 6);
  constexpr static uint32_t RETURN_NUMBER_BIT = (1 << 7);
  constexpr static uint32_t POINT_SOURCE_ID_BIT = (1 << 8);
  constexpr static uint32_t SCAN_DIRECTION_FLAG_BIT = (1 << 9);
  constexpr static uint32_t SCAN_ANGLE_RANK_BIT = (1 << 10);
  constexpr static uint32_t USER_DATA_BIT = (1 << 11);

  static PointAttributes supported_output_attributes();

  BinaryPersistence(const std::string& work_dir,
                    const PointAttributes& input_attributes,
                    const PointAttributes& output_attributes,
                    Compressed compressed = Compressed::Yes);
  ~BinaryPersistence();

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    const auto points_count = std::distance(points_begin, points_end);
    if (!points_count)
      return;

    const auto file_path = concat(_work_dir, "/", node_name, _file_extension);

    boost::iostreams::file_sink fs{ file_path, std::ios::out | std::ios::binary };
    if (!fs.is_open()) {
      std::cerr << "Could not write points file " << file_path << std::endl;
      return;
    }

    boost::iostreams::filtering_ostream stream;
    if (_compressed == Compressed::Yes) {
      boost::iostreams::zlib_params zlib_params;
      zlib_params.level = boost::iostreams::zlib::best_speed;
      stream.push(boost::iostreams::zlib_compressor{ zlib_params, 1 << 18 });
    }
    stream.push(fs);

    const auto has_colors = ((points_begin->rgbColor() != nullptr) &&
                             has_attribute(_output_attributes, PointAttribute::RGB));
    const auto has_normals = ((points_begin->normal() != nullptr) &&
                              has_attribute(_output_attributes, PointAttribute::Normal));
    const auto has_intensities = ((points_begin->intensity() != nullptr) &&
                                  has_attribute(_output_attributes, PointAttribute::Intensity));
    const auto has_classifications =
      ((points_begin->classification() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::Classification));
    const auto has_edge_of_flight_lines =
      ((points_begin->edge_of_flight_line() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::EdgeOfFlightLine));
    const auto has_gps_times = ((points_begin->gps_time() != nullptr) &&
                                has_attribute(_output_attributes, PointAttribute::GPSTime));
    const auto has_number_of_returns =
      ((points_begin->number_of_returns() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::NumberOfReturns));
    const auto has_return_numbers =
      ((points_begin->return_number() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::ReturnNumber));
    const auto has_point_source_ids =
      ((points_begin->point_source_id() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::PointSourceID));
    const auto has_scan_angle_ranks =
      ((points_begin->scan_angle_rank() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::ScanAngleRank));
    const auto has_scan_direction_flags =
      ((points_begin->scan_direction_flag() != nullptr) &&
       has_attribute(_output_attributes, PointAttribute::ScanDirectionFlag));
    const auto has_user_data = ((points_begin->user_data() != nullptr) &&
                                has_attribute(_output_attributes, PointAttribute::UserData));

    const uint32_t properties_bitmask =
      (has_colors ? COLOR_BIT : 0u) | (has_normals ? NORMAL_BIT : 0u) |
      (has_intensities ? INTENSITY_BIT : 0u) | (has_classifications ? CLASSIFICATION_BIT : 0u) |
      (has_edge_of_flight_lines ? EDGE_OF_FLIGHT_LINE_BIT : 0u) |
      (has_gps_times ? GPS_TIME_BIT : 0u) | (has_number_of_returns ? NUMBER_OF_RETURN_BIT : 0u) |
      (has_return_numbers ? RETURN_NUMBER_BIT : 0u) |
      (has_point_source_ids ? POINT_SOURCE_ID_BIT : 0u) |
      (has_scan_angle_ranks ? SCAN_ANGLE_RANK_BIT : 0u) |
      (has_scan_direction_flags ? SCAN_DIRECTION_FLAG_BIT : 0u) |
      (has_user_data ? USER_DATA_BIT : 0u);

    write_binary(properties_bitmask, stream);
    write_binary(static_cast<uint64_t>(points_count), stream);

    std::for_each(
      points_begin, points_end, [&stream](auto& point) { write_binary(point.position(), stream); });

    if (has_colors) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.rgbColor(), stream);
      });
    }

    if (has_normals) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.normal(), stream);
      });
    }

    if (has_intensities) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.intensity(), stream);
      });
    }

    if (has_classifications) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.classification(), stream);
      });
    }

    if (has_edge_of_flight_lines) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.edge_of_flight_line(), stream);
      });
    }

    if (has_gps_times) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.gps_time(), stream);
      });
    }

    if (has_number_of_returns) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.number_of_returns(), stream);
      });
    }

    if (has_return_numbers) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.return_number(), stream);
      });
    }

    if (has_point_source_ids) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.point_source_id(), stream);
      });
    }

    if (has_scan_angle_ranks) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.scan_angle_rank(), stream);
      });
    }

    if (has_scan_direction_flags) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.scan_direction_flag(), stream);
      });
    }

    if (has_user_data) {
      std::for_each(points_begin, points_end, [&stream](auto& point) {
        write_binary(*point.user_data(), stream);
      });
    }

    stream.pop();
  }

  void persist_points(PointBuffer const& points, const AABB& bounds, const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

  bool node_exists(const std::string& node_name) const;

  inline bool is_lossless() const { return true; }

private:
  std::string _work_dir;
  PointAttributes _input_attributes;
  PointAttributes _output_attributes;
  Compressed _compressed;
  std::string _file_extension;
};