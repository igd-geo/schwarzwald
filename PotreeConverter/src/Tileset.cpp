#include "Tileset.h"
#include "type_util.h"

static BoundingRegion boundingRegionFromAABB(
    const AABB& aabb, const Potree::SRSTransformHelper& transformHelper) {
  auto minX = aabb.min.x;
  auto minY = aabb.min.y;
  auto minHeight = aabb.min.z;
  auto maxX = aabb.max.x;
  auto maxY = aabb.max.y;
  auto maxHeight = aabb.max.z;

  Vector3<double> vertices[] = {{minX, minY, minHeight},
                                {minX, maxY, minHeight},
                                {maxX, maxY, minHeight},
                                {maxX, maxY, minHeight}};

  // Bounding volumes in 3Dtiles are specified in WGS84 lat/lon coordinates
  transformHelper.transformPositionsTo(Potree::TargetSRS::WGS84,
                                       gsl::make_span(vertices));

  // x is longitude, y is latitude

  auto newMinLon =
      std::min(vertices[0].x,
               std::min(vertices[1].x, std::min(vertices[2].x, vertices[3].x)));
  auto newMinLat =
      std::min(vertices[0].y,
               std::min(vertices[1].y, std::min(vertices[2].y, vertices[3].y)));
  auto newMaxLon =
      std::max(vertices[0].x,
               std::max(vertices[1].x, std::max(vertices[2].x, vertices[3].x)));
  auto newMaxLat =
      std::max(vertices[0].y,
               std::max(vertices[1].y, std::max(vertices[2].y, vertices[3].y)));

  BoundingRegion boundingRegion;
  boundingRegion.east = newMaxLon;
  boundingRegion.west = newMinLon;
  boundingRegion.north = newMaxLat;
  boundingRegion.south = newMinLat;
  boundingRegion.minHeight = minHeight;
  boundingRegion.maxHeight = maxHeight;

  return boundingRegion;
}

BoundingVolume_t boundingVolumeFromAABB(
    const AABB& aabb, const Potree::SRSTransformHelper& transformHelper) {
  // TODO Configure what type of bounding region to generate here
  return boundingRegionFromAABB(aabb, transformHelper);
}

std::vector<double> boundingVolumeToArray(
    const BoundingVolume_t& boundingVolume) {
  return std::visit(
      overloaded{[](const BoundingRegion& br) -> std::vector<double> {
        return {br.west,  br.south,     br.east,
                br.north, br.minHeight, br.maxHeight};
      }},
      boundingVolume);
}
