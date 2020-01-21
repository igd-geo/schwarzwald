#include "io/Cesium3DTilesPersistence.h"

#include "PNTSReader.h"
#include "PNTSWriter.h"
#include "PointAttributes.hpp"
#include "Transformation.h"
#include "stuff.h"

#include <fstream>

Cesium3DTilesPersistence::Cesium3DTilesPersistence(const std::string& work_dir,
                                                   const PointAttributes& point_attributes,
                                                   const SRSTransformHelper& transform_helper)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _transform_helper(transform_helper)
{}

Cesium3DTilesPersistence::~Cesium3DTilesPersistence() {}

void
Cesium3DTilesPersistence::persist_points(PointBuffer const& points,
                                         const AABB& bounds,
                                         const std::string& node_name)
{
  PNTSWriter writer{ concat(_work_dir, "/", node_name, ".pnts"), _point_attributes };
  auto points_copy = points;
  auto local_offset_to_world = setOriginToSmallestPoint(points_copy.positions());

  writer.write_points(points_copy);
  writer.flush(local_offset_to_world);
}

void
Cesium3DTilesPersistence::retrieve_points(const std::string& node_name, PointBuffer& points)
{
  const auto file_path = concat(_work_dir, "/", node_name, ".pnts");
  if (!std::experimental::filesystem::exists(file_path))
    return;

  auto pnts_content = readPNTSFile(file_path);
  points = std::move(pnts_content->points);

  // Transform back into world space
  for (auto& position : points.positions()) {
    position += pnts_content->rtc_center;
  }
}