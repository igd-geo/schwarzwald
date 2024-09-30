#include "util/Transformation.h"
#include "math/AABB.h"
#include "pointcloud/Point.h"

#include <algorithm>
#include <array>

using namespace std::string_literals;

static void
transformAABBWithProj4(AABB& aabb, PJ* transformation)
{
  // Construct the 8 corner vertices
  std::array<Vector3<double>, 8> aabbVertices;
  for (auto idx = 0; idx < 8; ++idx) {
    aabbVertices[idx] = { (idx & 0b001) ? aabb.max.x : aabb.min.x,
                          (idx & 0b010) ? aabb.max.y : aabb.min.y,
                          (idx & 0b100) ? aabb.max.z : aabb.min.z };
  }

  // Reset AABB for the process of finding min and max positions
  aabb.min = { std::numeric_limits<double>::max(),
               std::numeric_limits<double>::max(),
               std::numeric_limits<double>::max() };
  aabb.max = { std::numeric_limits<double>::lowest(),
               std::numeric_limits<double>::lowest(),
               std::numeric_limits<double>::lowest() };

  // Transform into new SRS while simultaneously getting the min and max
  // positions
  std::for_each(aabbVertices.begin(),
                aabbVertices.end(),
                [&aabb, transformation](auto& vertex) {
                  PJ_COORD transformed = proj_trans(transformation, PJ_FWD, proj_coord(vertex.x, vertex.y, vertex.z, 0.0));
                  
                  aabb.min.x = std::min(aabb.min.x, transformed.xyz.x);
                  aabb.min.y = std::min(aabb.min.y, transformed.xyz.y);
                  aabb.min.z = std::min(aabb.min.z, transformed.xyz.z);

                  aabb.max.x = std::max(aabb.max.x, transformed.xyz.x);
                  aabb.max.y = std::max(aabb.max.y, transformed.xyz.y);
                  aabb.max.z = std::max(aabb.max.z, transformed.xyz.z);
                });
}

SRSTransformHelper::~SRSTransformHelper() {}

IdentityTransform::~IdentityTransform() {}

void
IdentityTransform::transformPositionsTo(TargetSRS targetSRS,
                                        gsl::span<Vector3<double>> positions) const
{}

void
IdentityTransform::transformPositionToSourceFrom(TargetSRS currentSRS,
                                                 gsl::span<Vector3<double>> positions) const
{}

void
IdentityTransform::transformPointsTo(TargetSRS targetSRS,
                                     gsl::span<PointBuffer::PointReference> points) const
{}

void
IdentityTransform::transformPointsTo(TargetSRS targetSRS,
                                     util::Range<PointBuffer::PointIterator> points) const
{}

void
IdentityTransform::transformAABBsTo(TargetSRS targetSRS, gsl::span<AABB> aabbs) const
{}

Proj4Transform::Proj4Transform(const std::string& sourceProjection)
{
  _source_to_wgs84 = proj_create_crs_to_crs(PJ_DEFAULT_CTX, sourceProjection.c_str(), "+proj=longlat +datum=WGS84 +no_defs ", nullptr);
  if (_source_to_wgs84 == nullptr) {
    throw std::runtime_error{ "Source projection "s + sourceProjection + " not recognized!" };
  }

  _source_to_cesiumWorld = proj_create_crs_to_crs(PJ_DEFAULT_CTX, sourceProjection.c_str(), "+proj=geocent +datum=WGS84 +no_defs ", nullptr);
  if (_source_to_cesiumWorld == nullptr) {
    throw std::runtime_error{ "Source projection "s + sourceProjection + " not recognized!" };
  }
}

Proj4Transform::~Proj4Transform() {
  proj_destroy(_source_to_wgs84);
  proj_destroy(_source_to_cesiumWorld);
}

void
Proj4Transform::transformAABBsTo(TargetSRS targetSRS, gsl::span<AABB> aabbs) const
{
  const auto transformation = getTargetTransformation(targetSRS);
  if (!transformation) {
    throw std::invalid_argument{ "Unrecognized TargetSRS parameter!" };
  }

  for (auto& aabb : aabbs) {
    transformAABBWithProj4(aabb, transformation);
  }
}

void
Proj4Transform::transformPositionsTo(TargetSRS targetSRS,
                                     gsl::span<Vector3<double>> positions) const
{
  if (!positions.size())
    return;

  const auto transformation = getTargetTransformation(targetSRS);
  if (!transformation) {
    throw std::invalid_argument{ "Unrecognized TargetSRS parameter!" };
  }

  auto positionsPtr = positions.data();
  auto xPtr = &positionsPtr[0].x;
  auto yPtr = &positionsPtr[0].y;
  auto zPtr = &positionsPtr[0].z;
  const auto positionsCount = static_cast<long>(positions.size());
  const auto stride = sizeof(Vector3<double>); // 3 doubles per Vector3<double>

  proj_trans_generic(transformation, PJ_FWD, 
    xPtr, stride, positionsCount,
    yPtr, stride, positionsCount,
    zPtr, stride, positionsCount,
    0, 0, 0
  );
}

void
Proj4Transform::transformPositionToSourceFrom(TargetSRS currentSRS,
                                              gsl::span<Vector3<double>> positions) const
{
  if (!positions.size())
    return;

  const auto transformation = getTargetTransformation(currentSRS);
  if (!transformation) {
    throw std::invalid_argument{ "Unrecognized TargetSRS parameter!" };
  }
  auto positionsPtr = positions.data();
  auto xPtr = &positionsPtr[0].x;
  auto yPtr = &positionsPtr[0].y;
  auto zPtr = &positionsPtr[0].z;
  const auto positionsCount = static_cast<long>(positions.size());
  const auto stride = sizeof(Vector3<double>); // 3 doubles per Vector3<double>

  proj_trans_generic(transformation, PJ_INV, 
    xPtr, stride, positionsCount,
    yPtr, stride, positionsCount,
    zPtr, stride, positionsCount,
    0, 0, 0
  );
}

void
Proj4Transform::transformPointsTo(TargetSRS targetSRS,
                                  gsl::span<PointBuffer::PointReference> points) const
{
  if (!points.size())
    return;

  const auto transformation = getTargetTransformation(targetSRS);
  if (!transformation) {
    throw std::invalid_argument{ "Unrecognized TargetSRS parameter!" };
  }

  for (auto& point_ref : points) {
    auto& pos = point_ref.position();
    
    PJ_COORD new_pos = proj_trans(transformation, PJ_FWD, proj_coord(pos.x, pos.y, pos.z, 0.0));
    pos.x = new_pos.xyz.x;
    pos.y = new_pos.xyz.y;
    pos.z = new_pos.xyz.z;
  }
}

void
Proj4Transform::transformPointsTo(TargetSRS targetSRS,
                                  util::Range<PointBuffer::PointIterator> points) const
{
  if (!points.size())
    return;

  const auto transformation = getTargetTransformation(targetSRS);
  if (!transformation) {
    throw std::invalid_argument{ "Unrecognized TargetSRS parameter!" };
  }

  for (auto point_ref : points) {
    auto& pos = point_ref.position();
    PJ_COORD new_pos = proj_trans(transformation, PJ_FWD, proj_coord(pos.x, pos.y, pos.z, 0.0));
    pos.x = new_pos.xyz.x;
    pos.y = new_pos.xyz.y;
    pos.z = new_pos.xyz.z;
  }
}

PJ* Proj4Transform::getTargetTransformation(TargetSRS targetSRS) const
{
  switch (targetSRS) {
    case TargetSRS::WGS84:
      return _source_to_wgs84;
    case TargetSRS::CesiumWorld:
      return _source_to_cesiumWorld;
    default:
      return nullptr;
  }
}

Vector3<double>
setOriginToSmallestPoint(std::vector<Vector3<double>>& points)
{
  const auto dblMax = std::numeric_limits<double>::max();
  const auto smallestPoint = std::accumulate(points.begin(),
                                             points.end(),
                                             Vector3<double>{ dblMax, dblMax, dblMax },
                                             Vector3<double>::minByAxis);

  for (auto& point : points) {
    point -= smallestPoint;
  }

  return smallestPoint;
}

Vector3<double>
setOriginToSmallestPoint(gsl::span<PointBuffer::PointReference> points)
{
  const auto dblMax = std::numeric_limits<double>::max();
  const auto smallestPoint =
    std::accumulate(points.begin(),
                    points.end(),
                    Vector3<double>{ dblMax, dblMax, dblMax },
                    [](const auto& accum, const auto& point) {
                      return Vector3<double>::minByAxis(accum, point.position());
                    });

  for (auto& point : points) {
    point.position() -= smallestPoint;
  }

  return smallestPoint;
}
