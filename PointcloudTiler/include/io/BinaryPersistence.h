#pragma once

#include "AABB.h"
#include "Definitions.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "io/io_util.h"
#include "stuff.h"

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

  BinaryPersistence(const std::string& work_dir,
                    const PointAttributes& point_attributes,
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

    const auto has_colors = points_begin->rgbColor() != nullptr;
    const auto has_normals = points_begin->normal() != nullptr;
    const auto has_intensities = points_begin->intensity() != nullptr;
    const auto has_classifications = points_begin->classification() != nullptr;

    const uint32_t properties_bitmask =
      (has_colors ? COLOR_BIT : 0u) | (has_normals ? NORMAL_BIT : 0u) |
      (has_intensities ? INTENSITY_BIT : 0u) | (has_classifications ? CLASSIFICATION_BIT : 0u);

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

    stream.pop();
  }

  void persist_points(PointBuffer const& points, const AABB& bounds, const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

private:
  std::string _work_dir;
  const PointAttributes& _point_attributes;
  Compressed _compressed;
  std::string _file_extension;
};