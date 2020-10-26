#include "catch.hpp"

#include "io/LASPersistence.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "tiling/OctreeAlgorithms.h"

#include <random>
#include <string>

using namespace std::string_literals;

static PointBuffer
generate_random_points(size_t count, const AABB& bounds)
{
  std::mt19937 mt;
  std::uniform_real_distribution<double> x_dist{ bounds.min.x, bounds.max.x };
  std::uniform_real_distribution<double> y_dist{ bounds.min.y, bounds.max.y };
  std::uniform_real_distribution<double> z_dist{ bounds.min.z, bounds.max.z };

  std::vector<Vector3<double>> positions;
  positions.reserve(count);
  std::generate_n(std::back_inserter(positions), count, [&]() -> Vector3<double> {
    return { x_dist(mt), y_dist(mt), z_dist(mt) };
  });

  return { count, std::move(positions) };
}

template<typename Iter>
static std::vector<PointBuffer::PointReference>
point_references_from_indexed_points(Iter begin, Iter end)
{
  std::vector<PointBuffer::PointReference> point_references;
  point_references.reserve(std::distance(begin, end));
  std::transform(begin, end, std::back_inserter(point_references), [](const auto& indexed_point) {
    return indexed_point.point_reference;
  });
  return point_references;
}

static void
compare_points(const std::vector<PointBuffer::PointReference>& source, const PointBuffer& target)
{
  REQUIRE(source.size() == target.count());
  auto points_match = true;
  auto max_distance = 0.0;
  for (size_t idx = 0; idx < source.size(); ++idx) {
    points_match &= (source[idx].position() == target.positions()[idx]);
    max_distance =
      std::max(max_distance, source[idx].position().distanceTo(target.positions()[idx]));
  }
  REQUIRE(!points_match);
  REQUIRE(max_distance < 0.001);
}

/**
 * This test will always fail! It is a showcase demonstrating that LAS files are
 * not lossless: Given a point P, storing it in a LAS file and retrieving it
 * again can yield a slightly different point P'. This has two reasons:
 * 1) LAS files store coordinates with 32-bit precision, this tool uses 64-bit
 * precision
 * 2) LAS files use a signed integer representation for the
 * coordinates, which is obtained from the actual coordinate of the point, the
 * coordinate offset in the LAS file and the bounding box of the LAS file
 */

TEST_CASE("LASPersistence store and retrieve to LAS does not modify points")
{
  const auto root_folder = "."s;
  PointAttributes attributes;
  attributes.insert(PointAttribute::Position);
  LASPersistence persistence{ root_folder, attributes, attributes, Compressed::No };

  AABB bounds{ { 0, 0, 0 }, { 1, 1, 1 } };

  static constexpr size_t PointsCount = 4096;
  auto points = generate_random_points(PointsCount, bounds);

  std::vector<IndexedPoint<MortonIndex64Levels>> indexed_points;
  indexed_points.reserve(PointsCount);

  index_points<MortonIndex64Levels>(std::begin(points),
                                    std::end(points),
                                    std::back_inserter(indexed_points),
                                    bounds,
                                    OutlierPointsBehaviour::Abort);

  std::sort(std::begin(indexed_points), std::end(indexed_points));

  // Partition into the eight octants of the root bounding box
  const auto octant_partitions =
    partition_points_into_child_octants(std::begin(indexed_points), std::end(indexed_points), 0);

  // For each octant, store all contained points with the LASPersistence and
  // immediately retrieve them. Compare the retrieved points to the initial
  // points for equality
  for (auto& octant_range : octant_partitions) {
    if (octant_range.size() == 0)
      continue;

    const auto points_begin = std::begin(octant_range);
    const auto points_end = std::end(octant_range);

    const auto node_name = "_persistence_test_"s;
    const auto node_bounds = get_bounds_from_morton_index(points_begin->morton_index, bounds, 0);

    auto point_references = point_references_from_indexed_points(points_begin, points_end);
    persistence.persist_points(
      std::begin(point_references), std::end(point_references), node_bounds, node_name);

    PointBuffer retrieved_points;
    persistence.retrieve_points(node_name, retrieved_points);

    // Delete temporary file of LASPersistence so that we don't leave any
    // garbage when running this test!
    const auto tmp_file_name = concat(root_folder, "/", node_name, ".las");
    fs::remove(tmp_file_name);

    compare_points(point_references, retrieved_points);
  }
}