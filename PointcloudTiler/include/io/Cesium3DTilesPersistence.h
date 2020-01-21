#pragma once

#include "AABB.h"
#include "PNTSWriter.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "stuff.h"

struct SRSTransformHelper;

/**
 * Sink for writing 3D Tiles files
 */
struct Cesium3DTilesPersistence
{
  Cesium3DTilesPersistence(const std::string& work_dir,
                           const PointAttributes& point_attributes,
                           const SRSTransformHelper& transform_helper);
  ~Cesium3DTilesPersistence();

  template<typename Iter>
  void persist_points(Iter points_begin,
                      Iter points_end,
                      const AABB& bounds,
                      const std::string& node_name)
  {
    PNTSWriter writer{ concat(_work_dir, "/", node_name, ".pnts"), _point_attributes };
    auto local_offset_to_world = setOriginToSmallestPoint(points_begin, points_end);

    // TODO This is not optimal, writer should be able to take iterator pair
    PointBuffer tmp_points;
    std::for_each(points_begin, points_end, [&tmp_points](const auto& point_ref) {
      tmp_points.push_point(point_ref);
    });
    writer.write_points(tmp_points);
    writer.flush(local_offset_to_world);
  }
  void persist_points(PointBuffer const& points, const AABB& bounds, const std::string& node_name);

  void retrieve_points(const std::string& node_name, PointBuffer& points);

private:
  std::string _work_dir;
  const PointAttributes& _point_attributes;
  const SRSTransformHelper& _transform_helper;
};