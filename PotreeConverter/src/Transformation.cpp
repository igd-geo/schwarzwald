#include "Transformation.h"
#include "AABB.h"
#include "Point.h"

#include <algorithm>
#include <array>

using namespace std::string_literals;

static void transformAABBWithProj4(Potree::AABB& aabb, projPJ sourceProjection,
                                   projPJ targetProjection) {
  // Construct the 8 corner vertices
  std::array<Potree::Vector3<double>, 8> aabbVertices;
  for (auto idx = 0; idx < 8; ++idx) {
    aabbVertices[idx] = {(idx & 0b001) ? aabb.max.x : aabb.min.x,
                         (idx & 0b010) ? aabb.max.y : aabb.min.y,
                         (idx & 0b100) ? aabb.max.z : aabb.min.z};
  }

  // Reset AABB for the process of finding min and max positions
  aabb.min = {std::numeric_limits<double>::max(),
              std::numeric_limits<double>::max(),
              std::numeric_limits<double>::max()};
  aabb.max = {std::numeric_limits<double>::lowest(),
              std::numeric_limits<double>::lowest(),
              std::numeric_limits<double>::lowest()};

  // Transform into new SRS while simultaneously getting the min and max
  // positions
  std::for_each(aabbVertices.begin(), aabbVertices.end(),
                [&aabb, sourceProjection, targetProjection](auto& vertex) {
                  pj_transform(sourceProjection, targetProjection, 1, 1,
                               &vertex.x, &vertex.y, &vertex.z);

                  aabb.min.x = std::min(aabb.min.x, vertex.x);
                  aabb.min.y = std::min(aabb.min.y, vertex.y);
                  aabb.min.z = std::min(aabb.min.z, vertex.z);

                  aabb.max.x = std::max(aabb.max.x, vertex.x);
                  aabb.max.y = std::max(aabb.max.y, vertex.y);
                  aabb.max.z = std::max(aabb.max.z, vertex.z);
                });
}

Potree::SRSTransformHelper::~SRSTransformHelper() {}

Potree::IdentityTransform::~IdentityTransform() {}

void Potree::IdentityTransform::transformPositionsTo(
    TargetSRS targetSRS, gsl::span<Vector3<double>> positions) const {}
void Potree::IdentityTransform::transformAABBsTo(TargetSRS targetSRS,
                                                 gsl::span<AABB> aabbs) const {}

Potree::Proj4Transform::Proj4Transform(const std::string& sourceProjection) {
  _sourceTransformation = pj_init_plus(sourceProjection.c_str());

  if (_sourceTransformation == nullptr) {
    throw std::runtime_error{"Source projection "s + sourceProjection +
                             " not recognized!"};
  }

  _wgs84 = pj_init_plus("+proj=longlat +datum=WGS84 +no_defs ");
  _cesiumWorld = pj_init_plus("+proj=geocent +datum=WGS84 +no_defs ");
}

Potree::Proj4Transform::~Proj4Transform() {}

void Potree::Proj4Transform::transformAABBsTo(TargetSRS targetSRS,
                                              gsl::span<AABB> aabbs) const {
  const auto targetTransformation = getTargetTransformation(targetSRS);
  if (!targetTransformation) {
    throw std::invalid_argument{"Unrecognized TargetSRS parameter!"};
  }

  for (auto& aabb : aabbs) {
    transformAABBWithProj4(aabb, _sourceTransformation, targetTransformation);
  }
}

void Potree::Proj4Transform::transformPositionsTo(
    TargetSRS targetSRS, gsl::span<Vector3<double>> positions) const {
  if (!positions.size()) return;

  const auto targetTransformation = getTargetTransformation(targetSRS);
  if (!targetTransformation) {
    throw std::invalid_argument{"Unrecognized TargetSRS parameter!"};
  }

  // This is a bit dangerous as it assumes that Vector3 is layed out linearily
  // and packed tightly, so we have some compile-time checks here to ensure this
  static_assert(
      sizeof(Vector3<double>) == (3 * sizeof(double)),
      "Revise the Proj4Transform::transformPositionsTo method. It assumes that "
      "the binary layout of Vector3<double> if [x:double;y:double;z:double]");
  static_assert(
      offsetof(Vector3<double>, x) == 0,
      "Revise the Proj4Transform::transformPositionsTo method. It assumes that "
      "the binary layout of Vector3<double> if [x:double;y:double;z:double]");
  static_assert(
      offsetof(Vector3<double>, y) == sizeof(double),
      "Revise the Proj4Transform::transformPositionsTo method. It assumes that "
      "the binary layout of Vector3<double> if [x:double;y:double;z:double]");
  static_assert(
      offsetof(Vector3<double>, z) == (2 * sizeof(double)),
      "Revise the Proj4Transform::transformPositionsTo method. It assumes that "
      "the binary layout of Vector3<double> if [x:double;y:double;z:double]");

  auto positionsPtr = positions.data();
  auto xPtr = reinterpret_cast<double*>(positionsPtr);
  auto yPtr = xPtr + 1;
  auto zPtr = xPtr + 2;
  const auto positionsCount = static_cast<long>(positions.size());
  const auto stride = 3;  // 3 doubles per Vector3<double>

  pj_transform(_sourceTransformation, targetTransformation, positionsCount,
               stride, xPtr, yPtr, zPtr);
}

projPJ Potree::Proj4Transform::getTargetTransformation(
    TargetSRS targetSRS) const {
  switch (targetSRS) {
    case Potree::TargetSRS::WGS84:
      return _wgs84;
    case Potree::TargetSRS::CesiumWorld:
      return _cesiumWorld;
    default:
      return nullptr;
  }
}

Potree::Vector3<double> Potree::setOriginToSmallestPoint(
    std::vector<Vector3<double>>& points) {
  const auto dblMax = std::numeric_limits<double>::max();
  const auto smallestPoint = std::accumulate(
      points.begin(), points.end(), Vector3<double>{dblMax, dblMax, dblMax},
      Vector3<double>::minByAxis);

  for (auto& point : points) {
    point -= smallestPoint;
  }

  return smallestPoint;
}
