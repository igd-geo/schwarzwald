#pragma once

#include "datastructures/PointBuffer.h"
#include "io/PNTSWriter.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "pointcloud/Tileset.h"
#include "util/stuff.h"

#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>

struct SRSTransformHelper;

/**
 * Sink for writing 3D Tiles files
 */
struct Cesium3DTilesPersistence
{
  Cesium3DTilesPersistence(const std::string& work_dir,
                           const PointAttributes& point_attributes,
                           float spacing_at_root,
                           const Vector3<double>& global_offset);
  Cesium3DTilesPersistence(Cesium3DTilesPersistence&&) = default;
  ~Cesium3DTilesPersistence();

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    if (std::distance(points_begin, points_end) == 0) {
      throw std::runtime_error{ "persist_points requires a non-empty range" };
    }

    PNTSWriter writer{ concat(_work_dir, "/", node_name, ".pnts"),
                       _point_attributes };
    // OPTIMIZATION This is not optimal, writer should be able to take iterator
    // pair
    PointBuffer tmp_points;
    std::for_each(
      points_begin, points_end, [&tmp_points](const auto& point_ref) {
        tmp_points.push_point(point_ref);
      });
    writer.write_points(tmp_points);
    writer.flush(_global_offset);

    on_write_node(node_name, bounds);
  }
  void persist_points(PointBuffer const& points,
                      const AABB& bounds,
                      const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

  bool node_exists(const std::string& node_name) const;

  inline bool is_lossless() const { return true; }

private:
  void on_write_node(const std::string& node_name, const AABB& node_bounds);
  void write_tilesets() const;

  std::string _work_dir;
  const PointAttributes& _point_attributes;
  float _spacing_at_root;
  Vector3<double> _global_offset;

  std::unique_ptr<std::mutex> _tilesets_lock;
  std::optional<Tileset> _root_tileset;
};