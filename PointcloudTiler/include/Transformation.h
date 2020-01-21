#pragma once

#include "PointBuffer.h"
#include "Vector3.h"
#include "proj_api.h"

#include <string>

#include <gsl/gsl>

class Point;
struct AABB;

/// <summary>
/// Spatial reference systems that the SRSTransformHelper can transform into
/// </summary>
enum class TargetSRS
{
  /// <summary>
  /// WGS84 specified with latitude, longitude (radians) and height over
  /// ellipsoid (meters)
  /// </summary>
  WGS84,
  /// <summary>
  /// Cesium world coordinate system (WGS84 geocentric)
  /// </summary>
  CesiumWorld
};

/// <summary>
/// Utility class for transforming positions between spatial reference systems
/// </summary>
struct SRSTransformHelper
{
  virtual ~SRSTransformHelper();

  /// <summary>
  /// Transforms a range of positions into the target SRS. Transformation
  /// modifies the source points
  /// </summary>
  virtual void transformPositionsTo(TargetSRS targetSRS,
                                    gsl::span<Vector3<double>> positions) const = 0;

  /// <summary>
  /// Transforms a range of position from the given SRS back into source
  /// coordinate space
  /// </summary>
  virtual void transformPositionToSourceFrom(TargetSRS currentSRS,
                                             gsl::span<Vector3<double>> positions) const = 0;

  virtual void transformPointsTo(TargetSRS targetSRS,
                                 gsl::span<PointBuffer::PointReference> points) const = 0;

  /// <summary>
  /// Transforms a range of axis-aligned bounding boxes into the target SRS.
  /// Transformation happens in-place. The axis-aligned property will be
  /// preserved after the transformation, which can result in an increase in
  /// volume of the AABBs
  /// </summary>
  virtual void transformAABBsTo(TargetSRS targetSRS, gsl::span<AABB> aabbs) const = 0;
};

/// <summary>
/// Identity transformation, i.e. a transformation that does nothing
/// </summary>
struct IdentityTransform : SRSTransformHelper
{
  virtual ~IdentityTransform();

  void transformPositionsTo(TargetSRS targetSRS,
                            gsl::span<Vector3<double>> positions) const override;
  void transformPositionToSourceFrom(TargetSRS currentSRS,
                                     gsl::span<Vector3<double>> positions) const override;
  void transformPointsTo(TargetSRS targetSRS,
                         gsl::span<PointBuffer::PointReference> points) const override;
  void transformAABBsTo(TargetSRS targetSRS, gsl::span<AABB> aabbs) const override;
};

/// <summary>
/// Transformation based on proj4 coordinate system conversion
/// </summary>
struct Proj4Transform : SRSTransformHelper
{
  explicit Proj4Transform(const std::string& sourceTransformation);
  virtual ~Proj4Transform();

  void transformPositionsTo(TargetSRS targetSRS,
                            gsl::span<Vector3<double>> positions) const override;
  void transformPositionToSourceFrom(TargetSRS currentSRS,
                                     gsl::span<Vector3<double>> positions) const override;
  void transformPointsTo(TargetSRS targetSRS,
                         gsl::span<PointBuffer::PointReference> points) const override;
  void transformAABBsTo(TargetSRS targetSRS, gsl::span<AABB> aabbs) const override;

private:
  projPJ getTargetTransformation(TargetSRS targetSRS) const;

  projPJ _sourceTransformation;
  projPJ _wgs84;
  projPJ _cesiumWorld;
};

/// <summary>
/// Finds the smallest point of the given points and subtracts it from all
/// points. This sets the origin of the points to the smallest point. The
/// smallest point prior to subtraction is returned
/// </summary>
Vector3<double>
setOriginToSmallestPoint(std::vector<Vector3<double>>& points);

template<typename Iter>
Vector3<double>
setOriginToSmallestPoint(Iter points_begin, Iter points_end)
{
  const auto dblMax = std::numeric_limits<double>::max();
  const auto smallestPoint =
    std::accumulate(points_begin,
                    points_end,
                    Vector3<double>{ dblMax, dblMax, dblMax },
                    [](const auto& accum, const auto& point) {
                      return Vector3<double>::minByAxis(accum, point.position());
                    });

  std::for_each(
    points_begin, points_end, [smallestPoint](auto& point) { point.position() -= smallestPoint; });

  return smallestPoint;
}