#include "catch.hpp"

#include "datastructures/SparseGrid.h"
#include "io/BinaryPersistence.h"
#include "io/MemoryPersistence.h"
#include "octree/OctreeAlgorithms.h"
#include "process/Tiler.h"
#include <debug/ProgressReporter.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/format.hpp>
#include <boost/functional/hash.hpp>
#include <boost/scope_exit.hpp>
#include <random>
#include <unordered_set>

namespace std {
template <> struct hash<Vector3<double>> {
  typedef Vector3<double> argument_type;
  typedef std::size_t result_type;

  result_type operator()(Vector3<double> const &s) const noexcept {
    size_t seed = std::hash<double>{}(s.x);
    boost::hash_combine(seed, std::hash<double>{}(s.y));
    boost::hash_combine(seed, std::hash<double>{}(s.z));
    return seed;
  }
};
} // namespace std

static PointBuffer create_random_dataset(size_t num_points) {
  std::default_random_engine rnd{static_cast<uint32_t>(time(nullptr))};
  std::uniform_real_distribution<float> dist;

  // Create num_points random points in [0;1]³
  std::vector<Vector3<double>> positions;
  positions.reserve(num_points);
  std::generate_n(std::back_inserter(positions), num_points,
                  [&]() -> Vector3<double> {
                    return {dist(rnd), dist(rnd), dist(rnd)};
                  });

  PointBuffer point_buffer{num_points, std::move(positions)};
  return point_buffer;
}

// static bool
// points_obey_minimum_distance(const PointBuffer& points, const AABB& bounds,
// float min_distance)
// {
//   SparseGrid grid{ bounds, min_distance };

//   for (auto& pos : points.positions()) {
//     if (!grid.add(pos))
//       return false;
//   }
//   return true;
// }

static bool
points_are_contained_in_bounds(const PointBuffer &points, const AABB &bounds,
                               Vector3<double> *fail_point = nullptr) {
  for (auto &pos : points.positions()) {
    if (!bounds.isInside(pos)) {
      if (fail_point) {
        *fail_point = pos;
      }
      return false;
    }
  }
  return true;
}

TEST_CASE("Tiler works", "[Tiler]") {
  constexpr size_t NumPoints = 1'000'000;
  constexpr size_t MaxPointsPerNode = 20'000;

  const auto dataset = create_random_dataset(NumPoints);
  AABB bounds{{0, 0, 0}, {1, 1, 1}};
  auto spacing_at_root = 0.1f;
  const auto sampling_strategy =
      make_sampling_strategy<RandomSortedGridSampling>(MaxPointsPerNode);
  PointsPersistence persistence{MemoryPersistence{}};

  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = spacing_at_root;
  tiler_meta_parameters.max_depth = 40;
  tiler_meta_parameters.max_points_per_node = MaxPointsPerNode;
  tiler_meta_parameters.internal_cache_size =
      100'000; // Use small internal cache size to guarantee
               // out-of-core processing

  Tiler writer{bounds,  tiler_meta_parameters, sampling_strategy,
               nullptr, persistence,           "."};

  writer.cache(dataset);
  writer.index();
  writer.close();

  // Two properties have to hold:
  //  1) The converter processes and stores ALL points (except when the tree is
  //  too shallow due to max_depth) 2) All points of each node must be contained
  //  within the bounds of their respective node

  // There is also the spacing property (min distance), however this depends on
  // the sampling strategy used. The naive version of my algorithm uses the
  // 'random sorted grid' approach, which can result in arbitrarily close
  // points. Once other sampling strategies are implemented, there will be tests
  // for their correctness!

  // For fast matching of processed points with existing points we use
  // unordered_set
  std::unordered_set<Vector3<double>> expected_points;
  expected_points.reserve(NumPoints);
  for (auto &pos : dataset.positions()) {
    expected_points.insert(pos);
  }

  const auto processed_points =
      persistence.get<MemoryPersistence>().get_points();
  for (auto &kv : processed_points) {
    const auto &node_name = kv.first;
    const auto &points = kv.second;

    const auto node_key = from_string<21>(node_name);
    const auto node_level = node_name.size() - 1; // Node level, root = -1
    const auto node_bounds =
        get_bounds_from_morton_index(node_key, bounds, node_level);
    // const auto min_distance_at_node = spacing_at_root / std::pow(2,
    // node_level + 1); REQUIRE(points_obey_minimum_distance(points,
    // node_bounds, min_distance_at_node));
    REQUIRE(points_are_contained_in_bounds(points, node_bounds));

    for (auto &pos : points.positions()) {
      const auto point_iter = expected_points.find(pos);
      const auto point_exists_in_dataset =
          (expected_points.find(pos) != expected_points.end());
      REQUIRE(point_exists_in_dataset);
      expected_points.erase(
          point_iter); // Make sure we don't allow duplicate points!
    }
  }

  const auto num_processed_points = (dataset.count() - expected_points.size());
  REQUIRE(num_processed_points == NumPoints);
}

TEST_CASE("Tiler with deep tree works", "[Tiler]") {
  constexpr size_t NumPoints = 1'000'000;
  constexpr size_t MaxPointsPerNode = 20'000;

  const auto dataset = create_random_dataset(NumPoints);
  // Very large bounds will force the octree to become deeper than the 21 levels
  // that MortonIndex64 can support. This should trigger the re-indexing
  // mechanism, which is what we want to test with this test case! What we don't
  // want is dropped points, everything should still be indexed correctly!
  AABB bounds{{0, 0, 0}, {1 << 20, 1 << 20, 1 << 20}};
  auto spacing_at_root = (1 << 20) / 10.f;
  const auto sampling_strategy =
      make_sampling_strategy<RandomSortedGridSampling>(MaxPointsPerNode);
  PointsPersistence persistence{MemoryPersistence{}};

  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = spacing_at_root;
  tiler_meta_parameters.max_depth = 40;
  tiler_meta_parameters.max_points_per_node = MaxPointsPerNode;
  tiler_meta_parameters.internal_cache_size =
      100'000; // Use small internal cache size to guarantee
               // out-of-core processing
  Tiler writer{bounds,  tiler_meta_parameters, sampling_strategy,
               nullptr, persistence,           "."};

  writer.cache(dataset);
  writer.index();
  writer.close();

  // Two properties have to hold:
  //  1) The converter processes and stores ALL points (except when the tree is
  //  too shallow due to max_depth) 2) All points of each node must be contained
  //  within the bounds of their respective node

  // There is also the spacing property (min distance), however this depends on
  // the sampling strategy used. The naive version of my algorithm uses the
  // 'random sorted grid' approach, which can result in arbitrarily close
  // points. Once other sampling strategies are implemented, there will be tests
  // for their correctness!

  // For fast matching of processed points with existing points we use
  // unordered_set
  std::unordered_set<Vector3<double>> expected_points;
  expected_points.reserve(NumPoints);
  for (auto &pos : dataset.positions()) {
    expected_points.insert(pos);
  }

  const auto processed_points =
      persistence.get<MemoryPersistence>().get_points();
  for (auto &kv : processed_points) {
    const auto &node_name = kv.first;
    const auto &points = kv.second;

    const auto node_key = from_string<21>(node_name);
    const auto node_level = node_name.size() - 1; // Node level, root = -1
    const auto node_bounds =
        get_bounds_from_morton_index(node_key, bounds, node_level);
    // const auto min_distance_at_node = spacing_at_root / std::pow(2,
    // node_level + 1); REQUIRE(points_obey_minimum_distance(points,
    // node_bounds, min_distance_at_node));
    REQUIRE(points_are_contained_in_bounds(points, node_bounds));

    for (auto &pos : points.positions()) {
      const auto point_iter = expected_points.find(pos);
      const auto point_exists_in_dataset =
          (expected_points.find(pos) != expected_points.end());
      REQUIRE(point_exists_in_dataset);
      expected_points.erase(
          point_iter); // Make sure we don't allow duplicate points!
    }
  }

  const auto num_processed_points = (dataset.count() - expected_points.size());
  REQUIRE(num_processed_points == NumPoints);
}

TEST_CASE("Tiler with GridCenter sampling", "[Tiler]") {
  constexpr size_t NumPoints = 1'000'000;
  constexpr size_t MaxPointsPerNode = 20'000;

  const auto dataset = create_random_dataset(NumPoints);
  AABB bounds{{0, 0, 0}, {1, 1, 1}};
  auto spacing_at_root = 0.1f;
  const auto sampling_strategy =
      make_sampling_strategy<GridCenterSampling>(MaxPointsPerNode);
  PointsPersistence persistence{MemoryPersistence{}};

  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = spacing_at_root;
  tiler_meta_parameters.max_depth = 40;
  tiler_meta_parameters.max_points_per_node = MaxPointsPerNode;
  tiler_meta_parameters.internal_cache_size =
      100'000; // Use small internal cache size to guarantee
               // out-of-core processing
  Tiler writer{bounds,  tiler_meta_parameters, sampling_strategy,
               nullptr, persistence,           "."};

  writer.cache(dataset);
  writer.index();
  writer.close();

  // Two properties have to hold:
  //  1) The converter processes and stores ALL points (except when the tree is
  //  too shallow due to max_depth) 2) All points of each node must be contained
  //  within the bounds of their respective node

  // For fast matching of processed points with existing points we use
  // unordered_set
  std::unordered_set<Vector3<double>> expected_points;
  expected_points.reserve(NumPoints);
  for (auto &pos : dataset.positions()) {
    expected_points.insert(pos);
  }

  const auto processed_points =
      persistence.get<MemoryPersistence>().get_points();
  for (auto &kv : processed_points) {
    const auto &node_name = kv.first;
    const auto &points = kv.second;

    const auto node_key = from_string<21>(node_name);
    const auto node_level = node_name.size() - 1; // Node level, root = -1
    const auto node_bounds =
        get_bounds_from_morton_index(node_key, bounds, node_level);
    // const auto min_distance_at_node = spacing_at_root / std::pow(2,
    // node_level + 1); REQUIRE(points_obey_minimum_distance(points,
    // node_bounds, min_distance_at_node));
    REQUIRE(points_are_contained_in_bounds(points, node_bounds));

    for (auto &pos : points.positions()) {
      const auto point_iter = expected_points.find(pos);
      const auto point_exists_in_dataset =
          (expected_points.find(pos) != expected_points.end());
      REQUIRE(point_exists_in_dataset);
      expected_points.erase(
          point_iter); // Make sure we don't allow duplicate points!
    }
  }

  const auto num_processed_points = (dataset.count() - expected_points.size());
  REQUIRE(num_processed_points == NumPoints);
}

TEST_CASE("Tiler with PossionDisk sampling", "[Tiler]") {
  constexpr size_t NumPoints = 1'000'000;
  constexpr size_t MaxPointsPerNode = 20'000;

  const auto dataset = create_random_dataset(NumPoints);
  AABB bounds{{0, 0, 0}, {1, 1, 1}};
  auto spacing_at_root = 0.05f;
  const auto sampling_strategy =
      make_sampling_strategy<PoissonDiskSampling>(MaxPointsPerNode);
  PointsPersistence persistence{MemoryPersistence{}};

  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = spacing_at_root;
  tiler_meta_parameters.max_depth = 40;
  tiler_meta_parameters.max_points_per_node = MaxPointsPerNode;
  tiler_meta_parameters.internal_cache_size =
      100'000; // Use small internal cache size to guarantee
               // out-of-core processing

  Tiler writer{bounds,  tiler_meta_parameters, sampling_strategy,
               nullptr, persistence,           "."};

  writer.cache(dataset);
  writer.index();
  writer.close();

  // Two properties have to hold:
  //  1) The converter processes and stores ALL points (except when the tree is
  //  too shallow due to max_depth) 2) All points of each node must be contained
  //  within the bounds of their respective node

  // For fast matching of processed points with existing points we use
  // unordered_set
  std::unordered_set<Vector3<double>> expected_points;
  expected_points.reserve(NumPoints);
  for (auto &pos : dataset.positions()) {
    expected_points.insert(pos);
  }

  const auto points_obey_minimum_distance =
      [](const auto &points, const auto &bounds, double min_distance) -> bool {
    SparseGrid grid{bounds, static_cast<float>(min_distance)};
    for (auto &point : points) {
      if (!grid.add(point))
        return false;
    }
    return true;
  };

  const auto is_leaf_node = [](const std::string &node_name,
                               const auto &all_nodes) {
    for (auto &kv : all_nodes) {
      const auto &other_node_name = kv.first;
      if (other_node_name.size() != (node_name.size() + 1))
        continue;
      if (boost::starts_with(other_node_name, node_name))
        return false;
    }
    return true;
  };

  const auto processed_points =
      persistence.get<MemoryPersistence>().get_points();
  for (auto &kv : processed_points) {
    const auto &node_name = kv.first;
    const auto &points = kv.second;

    const auto node_key = from_string<21>(node_name);
    const auto node_level =
        static_cast<int32_t>(node_name.size()) - 2; // Node level, root = -1
    const auto node_bounds =
        get_bounds_from_morton_index(node_key, bounds, node_level + 1);
    const auto min_distance_at_node =
        spacing_at_root / std::pow(2, node_level + 1);

    const auto is_leaf = is_leaf_node(node_name, processed_points);
    if (!is_leaf) {
      REQUIRE(points_obey_minimum_distance(points.positions(), node_bounds,
                                           min_distance_at_node));
    }
    Vector3<double> fail_point;
    const auto points_all_contained =
        points_are_contained_in_bounds(points, node_bounds, &fail_point);
    if (!points_all_contained) {
      UNSCOPED_INFO("Position " << fail_point.x << " " << fail_point.y << " "
                                << fail_point.z << " not contained in bounds!");
    }
    REQUIRE(points_all_contained);

    for (auto &pos : points.positions()) {
      const auto point_iter = expected_points.find(pos);
      const auto point_exists_in_dataset =
          (expected_points.find(pos) != expected_points.end());
      REQUIRE(point_exists_in_dataset);
      expected_points.erase(
          point_iter); // Make sure we don't allow duplicate points!
    }
  }

  const auto num_processed_points = (dataset.count() - expected_points.size());
  REQUIRE(num_processed_points == NumPoints);
}

TEST_CASE("Tiler with BinaryPersistence writer", "[Tiler]") {
  constexpr size_t NumPoints = 1'000'000;
  constexpr size_t MaxPointsPerNode = 20'000;

  const auto dataset = create_random_dataset(NumPoints);
  AABB bounds{{0, 0, 0}, {1, 1, 1}};
  auto spacing_at_root = 0.1f;
  const auto sampling_strategy =
      make_sampling_strategy<RandomSortedGridSampling>(MaxPointsPerNode);
  // MemoryPersistence persistence;
  PointAttributes attributes;
  attributes.push_back(attributes::POSITION_CARTESIAN);

  const auto tmp_directory =
      (boost::format("./tmp_%1%") %
       (std::chrono::high_resolution_clock::now().time_since_epoch().count()))
          .str();
  if (!fs::exists(tmp_directory)) {
    if (!fs::create_directories(tmp_directory)) {
      FAIL("Could not create temporary output directory");
    }
  }

  BOOST_SCOPE_EXIT(&tmp_directory) {
    std::error_code ec;
    fs::remove_all(tmp_directory, ec);
    if (ec) {
      UNSCOPED_INFO("Could not remove temporary output directory");
    }
  }
  BOOST_SCOPE_EXIT_END

  PointsPersistence persistence{BinaryPersistence{tmp_directory, attributes}};

  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = spacing_at_root;
  tiler_meta_parameters.max_depth = 40;
  tiler_meta_parameters.max_points_per_node = MaxPointsPerNode;
  tiler_meta_parameters.internal_cache_size =
      100'000; // Use small internal cache size to guarantee
               // out-of-core processing

  Tiler writer{bounds,  tiler_meta_parameters, sampling_strategy,
               nullptr, persistence,           "."};

  writer.cache(dataset);
  writer.index();
  writer.close();
}
