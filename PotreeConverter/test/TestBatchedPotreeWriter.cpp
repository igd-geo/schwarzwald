#include "catch.hpp"

#include "BatchedPotreeWriter.h"
#include "SparseGrid.h"
#include "io/MemoryPersistence.h"
#include "octree/OctreeAlgorithms.h"
#include "ui/ProgressReporter.h"

#include <random>
#include <unordered_set>

#include <boost/functional/hash.hpp>

using namespace Potree;

namespace std {
template<>
struct hash<Potree::Vector3<double>>
{
  typedef Potree::Vector3<double> argument_type;
  typedef std::size_t result_type;

  result_type operator()(Potree::Vector3<double> const& s) const noexcept
  {
    size_t seed = std::hash<double>{}(s.x);
    boost::hash_combine(seed, std::hash<double>{}(s.y));
    boost::hash_combine(seed, std::hash<double>{}(s.z));
    return seed;
  }
};
}

static PointBuffer
create_random_dataset(size_t num_points)
{
  std::default_random_engine rnd{ static_cast<uint32_t>(time(nullptr)) };
  std::uniform_real_distribution<float> dist;

  // Create num_points random points in [0;1]³
  std::vector<Vector3<double>> positions;
  positions.reserve(num_points);
  std::generate_n(std::back_inserter(positions), num_points, [&]() -> Vector3<double> {
    return { dist(rnd), dist(rnd), dist(rnd) };
  });

  PointBuffer point_buffer{ num_points, std::move(positions) };
  return point_buffer;
}

// static bool
// points_obey_minimum_distance(const PointBuffer& points, const AABB& bounds, float min_distance)
// {
//   SparseGrid grid{ bounds, min_distance };

//   for (auto& pos : points.positions()) {
//     if (!grid.add(pos))
//       return false;
//   }
//   return true;
// }

static bool
points_are_contained_in_bounds(const PointBuffer& points, const AABB& bounds)
{
  for (auto& pos : points.positions()) {
    if (!bounds.isInside(pos))
      return false;
  }
  return true;
}

TEST_CASE("BatchedPotreeWriter works", "[BatchedPotreeWriter]")
{
  constexpr size_t NumPoints = 1'000'000;

  const auto dataset = create_random_dataset(NumPoints);
  AABB bounds{ { 0, 0, 0 }, { 1, 1, 1 } };
  auto spacing_at_root = 0.1f;
  PointAttributes attributes;
  attributes.add(::attributes::POSITION_CARTESIAN);
  IdentityTransform transform;
  MemoryPersistence persistence;

  BatchedPotreeWriter writer{ "",         bounds,     spacing_at_root,
                              20,         attributes, ConversionQuality::DEFAULT,
                              transform,  1024,       nullptr,
                              persistence };

  writer.cache(dataset);
  writer.index();
  writer.close();

  // Two properties have to hold:
  //  1) The converter processes and stores ALL points (except when the tree is too shallow due to
  //  max_depth)
  //  2) All points of each node must be contained within the bounds of their respective node

  // There is also the spacing property (min distance), however this depends on the sampling
  // strategy used. The naive version of my algorithm uses the 'random sorted grid' approach, which
  // can result in arbitrarily close points. Once other sampling strategies are implemented, there
  // will be tests for their correctness!

  // For fast matching of processed points with existing points we use unordered_set
  std::unordered_set<Vector3<double>> expected_points;
  expected_points.reserve(NumPoints);
  for (auto& pos : dataset.positions()) {
    expected_points.insert(pos);
  }

  const auto processed_points = persistence.get_points();
  for (auto& kv : processed_points) {
    const auto& node_name = kv.first;
    const auto& points = kv.second;

    const auto node_key = from_string<21>(node_name);
    const auto node_level = node_name.size() - 1; // Node level, root = -1
    const auto node_bounds = get_bounds_from_octree_key(node_key, bounds, node_level);
    // const auto min_distance_at_node = spacing_at_root / std::pow(2, node_level + 1);
    // REQUIRE(points_obey_minimum_distance(points, node_bounds, min_distance_at_node));
    REQUIRE(points_are_contained_in_bounds(points, node_bounds));

    for (auto& pos : points.positions()) {
      const auto point_iter = expected_points.find(pos);
      const auto point_exists_in_dataset = (expected_points.find(pos) != expected_points.end());
      REQUIRE(point_exists_in_dataset);
      expected_points.erase(point_iter); // Make sure we don't allow duplicate points!
    }
  }

  const auto num_processed_points = (dataset.count() - expected_points.size());
  REQUIRE(num_processed_points == NumPoints);
}

TEST_CASE("BatchedPotreeWriter with deep tree works", "[BatchedPotreeWriter]")
{
  constexpr size_t NumPoints = 1'000'000;

  const auto dataset = create_random_dataset(NumPoints);
  // Very large bounds will force the octree to become deeper than the 21 levels that
  // OctreeNodeKey64 can support. This should trigger the re-indexing mechanism, which is what we
  // want to test with this test case! What we don't want is dropped points, everything should still
  // be indexed correctly!
  AABB bounds{ { 0, 0, 0 }, { 1 << 20, 1 << 20, 1 << 20 } };
  auto spacing_at_root = (1 << 20) / 10.f;
  PointAttributes attributes;
  attributes.add(::attributes::POSITION_CARTESIAN);
  IdentityTransform transform;
  MemoryPersistence persistence;

  BatchedPotreeWriter writer{ "",         bounds,     spacing_at_root,
                              40,         attributes, ConversionQuality::DEFAULT,
                              transform,  1024,       nullptr,
                              persistence };

  writer.cache(dataset);
  writer.index();
  writer.close();

  // Two properties have to hold:
  //  1) The converter processes and stores ALL points (except when the tree is too shallow due to
  //  max_depth)
  //  2) All points of each node must be contained within the bounds of their respective node

  // There is also the spacing property (min distance), however this depends on the sampling
  // strategy used. The naive version of my algorithm uses the 'random sorted grid' approach, which
  // can result in arbitrarily close points. Once other sampling strategies are implemented, there
  // will be tests for their correctness!

  // For fast matching of processed points with existing points we use unordered_set
  std::unordered_set<Vector3<double>> expected_points;
  expected_points.reserve(NumPoints);
  for (auto& pos : dataset.positions()) {
    expected_points.insert(pos);
  }

  const auto processed_points = persistence.get_points();
  for (auto& kv : processed_points) {
    const auto& node_name = kv.first;
    const auto& points = kv.second;

    const auto node_key = from_string<21>(node_name);
    const auto node_level = node_name.size() - 1; // Node level, root = -1
    const auto node_bounds = get_bounds_from_octree_key(node_key, bounds, node_level);
    // const auto min_distance_at_node = spacing_at_root / std::pow(2, node_level + 1);
    // REQUIRE(points_obey_minimum_distance(points, node_bounds, min_distance_at_node));
    REQUIRE(points_are_contained_in_bounds(points, node_bounds));

    for (auto& pos : points.positions()) {
      const auto point_iter = expected_points.find(pos);
      const auto point_exists_in_dataset = (expected_points.find(pos) != expected_points.end());
      REQUIRE(point_exists_in_dataset);
      expected_points.erase(point_iter); // Make sure we don't allow duplicate points!
    }
  }

  const auto num_processed_points = (dataset.count() - expected_points.size());
  REQUIRE(num_processed_points == NumPoints);
}