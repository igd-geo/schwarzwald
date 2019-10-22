#include "Tileset.h"
#include "type_util.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static BoundingRegion
boundingRegionFromAABB(const AABB& aabb, const SRSTransformHelper& transformHelper)
{
  auto minX = aabb.min.x;
  auto minY = aabb.min.y;
  auto minHeight = aabb.min.z;
  auto maxX = aabb.max.x;
  auto maxY = aabb.max.y;
  auto maxHeight = aabb.max.z;

  Vector3<double> vertices[] = { { minX, minY, minHeight },
                                 { minX, maxY, minHeight },
                                 { maxX, maxY, minHeight },
                                 { maxX, maxY, minHeight } };

  // Bounding volumes in 3Dtiles are specified in WGS84 lat/lon coordinates
  transformHelper.transformPositionsTo(TargetSRS::WGS84, gsl::make_span(vertices));

  // x is longitude, y is latitude

  auto newMinLon =
    std::min(vertices[0].x, std::min(vertices[1].x, std::min(vertices[2].x, vertices[3].x)));
  auto newMinLat =
    std::min(vertices[0].y, std::min(vertices[1].y, std::min(vertices[2].y, vertices[3].y)));
  auto newMaxLon =
    std::max(vertices[0].x, std::max(vertices[1].x, std::max(vertices[2].x, vertices[3].x)));
  auto newMaxLat =
    std::max(vertices[0].y, std::max(vertices[1].y, std::max(vertices[2].y, vertices[3].y)));

  BoundingRegion boundingRegion;
  boundingRegion.east = newMaxLon;
  boundingRegion.west = newMinLon;
  boundingRegion.north = newMaxLat;
  boundingRegion.south = newMinLat;
  boundingRegion.minHeight = minHeight;
  boundingRegion.maxHeight = maxHeight;

  return boundingRegion;
}

static BoundingBox
boundingBoxFromAABB(const AABB& aabb, const SRSTransformHelper& transformHelper)
{
  const auto aabbCenter = aabb.getCenter();
  // Transform center vertex and vertices spanning the x, y and z vectors into target
  // coordinate system. From this, we can reconstruct an OBB in target space
  Vector3<double> aabb_center_xyz[] = { { aabbCenter.x, aabbCenter.y, aabbCenter.z },
                                        { aabb.max.x, aabbCenter.y, aabbCenter.z },
                                        { aabbCenter.x, aabb.max.y, aabbCenter.z },
                                        { aabbCenter.x, aabbCenter.y, aabb.max.z } };

  transformHelper.transformPositionsTo(TargetSRS::CesiumWorld, gsl::make_span(aabb_center_xyz));

  const auto& world_center = aabb_center_xyz[0];
  const auto& world_x = aabb_center_xyz[1] - aabb_center_xyz[0];
  const auto& world_y = aabb_center_xyz[2] - aabb_center_xyz[0];
  const auto& world_z = aabb_center_xyz[3] - aabb_center_xyz[0];

  BoundingBox ret;
  ret.cx = world_center.x;
  ret.cy = world_center.y;
  ret.cz = world_center.z;

  ret.xx = world_x.x;
  ret.xy = world_x.y;
  ret.xz = world_x.z;

  ret.yx = world_y.x;
  ret.yy = world_y.y;
  ret.yz = world_y.z;

  ret.zx = world_z.x;
  ret.zy = world_z.y;
  ret.zz = world_z.z;

  return ret;
}
#pragma GCC diagnostic pop

BoundingVolume_t
boundingVolumeFromAABB(const AABB& aabb, const SRSTransformHelper& transformHelper)
{
  // TODO Configure what type of bounding region to generate here
  // return boundingRegionFromAABB(aabb, transformHelper);
  return boundingBoxFromAABB(aabb, transformHelper);
}

std::vector<double>
boundingVolumeToArray(const BoundingVolume_t& boundingVolume)
{
  return std::visit(overloaded{ [](const BoundingRegion& br) -> std::vector<double> {
                                 return { br.west,  br.south,     br.east,
                                          br.north, br.minHeight, br.maxHeight };
                               },
                                [](const BoundingBox& bb) -> std::vector<double> {
                                  return { bb.cx, bb.cy, bb.cz, bb.xx, bb.xy, bb.xz,
                                           bb.yx, bb.yy, bb.yz, bb.zx, bb.zy, bb.zz };
                                } },
                    boundingVolume);
}
