#include "catch.hpp"

#include "tiling/OctreeAlgorithms.h"

#include <random>

using V3 = Vector3<double>;

namespace Catch {
template<unsigned int Levels>
struct StringMaker<MortonIndex<Levels>>
{
  static std::string convert(MortonIndex<Levels> const& value) { return to_string(value); }
};
} // namespace Catch

template<unsigned int Levels>
static V3
position_from_octant_indices(const std::array<uint8_t, Levels>& indices, const AABB& bounds)
{
  auto octant_bounds = bounds;
  for (auto octant : indices) {
    if ((octant & 0b1)) {
      octant_bounds.min.z += octant_bounds.extent().z / 2;
    } else {
      octant_bounds.max.z -= octant_bounds.extent().z / 2;
    }
    if ((octant & 0b10)) {
      octant_bounds.min.y += octant_bounds.extent().y / 2;
    } else {
      octant_bounds.max.y -= octant_bounds.extent().y / 2;
    }
    if ((octant & 0b100)) {
      octant_bounds.min.x += octant_bounds.extent().x / 2;
    } else {
      octant_bounds.max.x -= octant_bounds.extent().x / 2;
    }
  }
  return octant_bounds.getCenter();
}

TEST_CASE("Position from octree indices computed correctly", "[position_from_octant_indices]")
{
  constexpr uint32_t Levels = 2;

  AABB bounds{ V3{ 0, 0, 0 }, { 8, 8, 8 } };

  std::array<uint8_t, Levels> indices1 = { 0, 0 };
  std::array<uint8_t, Levels> indices2 = { 3, 0 };
  std::array<uint8_t, Levels> indices3 = { 0, 5 };
  std::array<uint8_t, Levels> indices4 = { 2, 6 };

  V3 expected_p1 = { 1, 1, 1 };
  V3 expected_p2 = { 1, 5, 5 };
  V3 expected_p3 = { 3, 1, 3 };
  V3 expected_p4 = { 3, 7, 1 };

  const auto p1 = position_from_octant_indices<Levels>(indices1, bounds);
  const auto p2 = position_from_octant_indices<Levels>(indices2, bounds);
  const auto p3 = position_from_octant_indices<Levels>(indices3, bounds);
  const auto p4 = position_from_octant_indices<Levels>(indices4, bounds);

  REQUIRE(p1 == expected_p1);
  REQUIRE(p2 == expected_p2);
  REQUIRE(p3 == expected_p3);
  REQUIRE(p4 == expected_p4);
}

TEST_CASE("Octree keys for first level are computed correctly", "[calculate_morton_indexs]")
{
  std::vector<V3> positions = {
    V3{ 0.25, 0.25, 0.25 }, // octant 0
    V3{ 0.25, 0.25, 0.75 }, // octant 1
    V3{ 0.25, 0.75, 0.25 }, // octant 2
    V3{ 0.75, 0.25, 0.25 }  // octant 4
  };

  PointBuffer points{ 4ull, positions };

  AABB bounds{ V3{ 0, 0, 0 }, V3{ 1, 1, 1 } };

  std::vector<MortonIndex<1>> octree_keys;
  octree_keys.resize(points.count());
  calculate_morton_indices_for_points<1>(
    points.positions().begin(), points.positions().end(), octree_keys.begin(), bounds);

  REQUIRE(octree_keys.size() == 4);
  REQUIRE(octree_keys[0].get_octant_at_level(0) == 0);
  REQUIRE(octree_keys[1].get_octant_at_level(0) == 1);
  REQUIRE(octree_keys[2].get_octant_at_level(0) == 2);
  REQUIRE(octree_keys[3].get_octant_at_level(0) == 4);
}

TEST_CASE("Octree key for multiple levels is computed correctly", "[calculate_morton_indexs]")
{
  constexpr uint32_t Levels = 20;

  std::array<uint8_t, Levels> expected_octants{ 5, 3, 7, 4, 0, 1, 6, 4, 3, 5,
                                                3, 6, 7, 3, 2, 1, 4, 0, 2, 5 };
  AABB bounds{ V3{ 0, 0, 0 }, V3{ 1 << 20, 1 << 20, 1 << 20 } };

  std::vector<V3> positions = { position_from_octant_indices<Levels>(expected_octants, bounds) };

  PointBuffer points{ 1ull, positions };

  std::vector<MortonIndex<Levels>> octree_indices;
  octree_indices.resize(points.count());
  calculate_morton_indices_for_points<Levels>(
    points.positions().begin(), points.positions().end(), octree_indices.begin(), bounds);

  REQUIRE(octree_indices.size() == 1);
  const auto key = octree_indices[0];
  for (uint32_t idx = 0; idx < Levels; ++idx) {
    REQUIRE(key.get_octant_at_level(idx) == expected_octants.at(idx));
  }
}

TEST_CASE("Octree keys for sub-range computed correctly", "[calculate_morton_indexs]")
{
  std::vector<V3> positions = {
    V3{ 0.25, 0.25, 0.25 }, // octant 0
    V3{ 0.25, 0.25, 0.75 }, // octant 1
    V3{ 0.25, 0.75, 0.25 }, // octant 2
    V3{ 0.75, 0.25, 0.25 }  // octant 4
  };

  PointBuffer points{ 4ull, positions };

  AABB bounds{ V3{ 0, 0, 0 }, V3{ 1, 1, 1 } };

  std::vector<MortonIndex<1>> octree_keys;
  octree_keys.resize(points.count());
  calculate_morton_indices_for_points<1>(
    points.positions().begin(), points.positions().begin() + 2, octree_keys.begin(), bounds);

  REQUIRE(octree_keys[0].get_octant_at_level(0) == 0);
  REQUIRE(octree_keys[1].get_octant_at_level(0) == 1);
  // Must not touch keys outside of provided range!
  REQUIRE(octree_keys[2].get() == 0);
  REQUIRE(octree_keys[3].get() == 0);

  // Try again but with the back part of the range
  octree_keys = {};
  octree_keys.resize(points.count());
  calculate_morton_indices_for_points<1>(
    points.positions().begin() + 2, points.positions().end(), octree_keys.begin() + 2, bounds);

  REQUIRE(octree_keys[0].get() == 0);
  REQUIRE(octree_keys[1].get() == 0);
  REQUIRE(octree_keys[2].get_octant_at_level(0) == 2);
  REQUIRE(octree_keys[3].get_octant_at_level(0) == 4);
}

TEST_CASE("Octree point filtering works correctly", "[filter_points_for_octree_node]")
{
  // Generate a 3D grid of points where the spacing is a power of 0.5 times the
  // octree size Pick different spaced sets of points
  constexpr uint32_t Levels = 5;
  constexpr double SideLength = 1 << Levels;
  constexpr size_t MaxPointsPerNode = 16;
  auto sampling_strategy = make_sampling_strategy<RandomSortedGridSampling>(MaxPointsPerNode);

  std::vector<V3> positions;
  positions.reserve(SideLength * SideLength * SideLength);

  for (size_t x = 0; x < SideLength; ++x) {
    for (size_t y = 0; y < SideLength; ++y) {
      for (size_t z = 0; z < SideLength; ++z) {
        positions.push_back({ x + 0.5, y + 0.5, z + 0.5 });
      }
    }
  }

  AABB bounds{ V3{ 0, 0, 0 }, V3{ SideLength, SideLength, SideLength } };
  PointBuffer points{ positions.size(), std::move(positions) };

  std::vector<MortonIndex<Levels>> octree_indices;
  octree_indices.resize(points.count());
  calculate_morton_indices_for_points<Levels>(
    points.positions().begin(), points.positions().end(), octree_indices.begin(), bounds);

  // Create a vector of PointReferences with their associated MortonIndexs for
  // filtering
  std::vector<IndexedPoint<Levels>> points_and_keys;
  points_and_keys.reserve(points.count());
  for (size_t idx = 0; idx < points.count(); ++idx) {
    auto point_ref = *(points.begin() + idx);
    points_and_keys.push_back(IndexedPoint<Levels>{ point_ref, octree_indices[idx] });
  }

  // Make sure the indexed points are sorted in ascending order, otherwise the
  // filter algorithm doesn't work
  std::sort(points_and_keys.begin(), points_and_keys.end(), [](const auto& l, const auto& r) {
    return l.morton_index.get() < r.morton_index.get();
  });

  MortonIndex<Levels> root_key;
  const auto partition_point_at_l0 =
    filter_points_for_octree_node(points_and_keys.begin(),
                                  points_and_keys.end(),
                                  root_key,
                                  0,
                                  bounds,
                                  SideLength,
                                  SamplingBehaviour::TakeAllWhenCountBelowMaxPoints,
                                  sampling_strategy);

  std::vector<V3> expected_positions = { V3{ 0.5, 0.5, 0.5 },   V3{ 0.5, 0.5, 16.5 },
                                         V3{ 0.5, 16.5, 0.5 },  V3{ 0.5, 16.5, 16.5 },
                                         V3{ 16.5, 0.5, 0.5 },  V3{ 16.5, 0.5, 16.5 },
                                         V3{ 16.5, 16.5, 0.5 }, V3{ 16.5, 16.5, 16.5 } };

  const auto num_selected_points = std::distance(points_and_keys.begin(), partition_point_at_l0);
  REQUIRE(num_selected_points == 8);

  std::vector<V3> actual_positions;
  actual_positions.resize(expected_positions.size());
  std::transform(
    points_and_keys.begin(),
    partition_point_at_l0,
    actual_positions.begin(),
    [](const auto& point_and_key) { return point_and_key.point_reference.position(); });

  // filter_points_for_octree_node should be stable, so the relative order of
  // points must not change!
  REQUIRE(actual_positions == expected_positions);
}

TEST_CASE("Octree point filtering is stable", "[filter_points_for_octree_node]")
{
  constexpr uint32_t Levels = 10;
  constexpr size_t NumPoints = 1024;
  constexpr float SideLength = 1024.f;
  constexpr size_t MaxPointsPerNode = 16;
  auto sampling_strategy = make_sampling_strategy<RandomSortedGridSampling>(MaxPointsPerNode);

  std::default_random_engine rnd{ (uint32_t)time(nullptr) };
  std::uniform_int_distribution<int> dist{ 1024, 2048 };

  AABB bounds{ V3{ 1024, 1024, 1024 }, V3{ 2048, 2048, 2048 } };

  std::vector<V3> rnd_points;
  rnd_points.reserve(NumPoints);

  std::generate_n(std::back_inserter(rnd_points), NumPoints, [&]() {
    return V3{ static_cast<double>(dist(rnd)),
               static_cast<double>(dist(rnd)),
               static_cast<double>(dist(rnd)) };
  });

  PointBuffer points{ rnd_points.size(), rnd_points };

  std::vector<MortonIndex<Levels>> octree_indices;
  octree_indices.resize(points.count());
  calculate_morton_indices_for_points<Levels>(
    points.positions().begin(), points.positions().end(), octree_indices.begin(), bounds);

  // Create a vector of PointReferences with their associated MortonIndexs for
  // filtering
  std::vector<IndexedPoint<Levels>> points_and_keys;
  points_and_keys.reserve(points.count());
  for (size_t idx = 0; idx < points.count(); ++idx) {
    auto point_ref = *(points.begin() + idx);
    points_and_keys.push_back(IndexedPoint<Levels>{ point_ref, octree_indices[idx] });
  }

  // Make sure the indexed points are sorted in ascending order, otherwise the
  // filter algorithm doesn't work
  std::sort(points_and_keys.begin(), points_and_keys.end(), [](const auto& l, const auto& r) {
    return l.morton_index.get() < r.morton_index.get();
  });

  MortonIndex<Levels> root_key;
  const auto partition_point_at_l0 =
    filter_points_for_octree_node(points_and_keys.begin(),
                                  points_and_keys.end(),
                                  root_key,
                                  0,
                                  bounds,
                                  SideLength,
                                  SamplingBehaviour::TakeAllWhenCountBelowMaxPoints,
                                  sampling_strategy);

  // All taken points have to be sorted still
  const auto taken_points_first_non_sorted_iter = std::adjacent_find(
    points_and_keys.begin(), partition_point_at_l0, [](const auto& l, const auto& r) {
      return l.morton_index.get() > r.morton_index.get();
    });
  const auto taken_points_are_sorted =
    (taken_points_first_non_sorted_iter == partition_point_at_l0);
  REQUIRE(taken_points_are_sorted);

  // Everything from 'partition_point_at_l0' to the end has to be sorted still!
  const auto first_non_sorted_pair_iter = std::adjacent_find(
    partition_point_at_l0, points_and_keys.end(), [](const auto& l, const auto& r) {
      return l.morton_index.get() > r.morton_index.get();
    });
  const auto non_taken_points_are_sorted = first_non_sorted_pair_iter == points_and_keys.end();
  REQUIRE(non_taken_points_are_sorted);
}

TEST_CASE("Partitioning points at root level into child octants works correctly",
          "[partition_points_into_child_octants]")
{
  // Use a lot of levels so that we can test the 'parent_node_level' parameter
  // of 'partition_points_into_child_octants'
  constexpr uint32_t Levels = 10;

  std::vector<V3> positions = {
    V3{ 1, 1, 1 }, // octant 0
    V3{ 1, 1, 3 }, // octant 1
    V3{ 1, 3, 1 }, // octant 2
    V3{ 1, 3, 3 }, // octant 3
    V3{ 3, 1, 1 }, // octant 4
    V3{ 3, 1, 3 }, // octant 5
    V3{ 3, 3, 1 }, // octant 6
    V3{ 3, 3, 3 }, // octant 7
  };

  PointBuffer points{ positions.size(), positions };

  AABB bounds{ V3{ 0, 0, 0 }, V3{ 4, 4, 4 } };

  std::vector<MortonIndex<Levels>> octree_indices;
  octree_indices.resize(points.count());
  calculate_morton_indices_for_points<Levels>(
    points.positions().begin(), points.positions().end(), octree_indices.begin(), bounds);

  std::vector<IndexedPoint<Levels>> indexed_points;
  indexed_points.reserve(positions.size());
  for (size_t idx = 0; idx < positions.size(); ++idx) {
    indexed_points.push_back(IndexedPoint<Levels>{ *(points.begin() + idx), octree_indices[idx] });
  }

  using Iter = std::vector<IndexedPoint<Levels>>::iterator;
  std::array<util::Range<Iter>, 8> expected_ranges = {
    util::Range<Iter>{ indexed_points.begin(), indexed_points.begin() + 1 },
    util::Range<Iter>{ indexed_points.begin() + 1, indexed_points.begin() + 2 },
    util::Range<Iter>{ indexed_points.begin() + 2, indexed_points.begin() + 3 },
    util::Range<Iter>{ indexed_points.begin() + 3, indexed_points.begin() + 4 },
    util::Range<Iter>{ indexed_points.begin() + 4, indexed_points.begin() + 5 },
    util::Range<Iter>{ indexed_points.begin() + 5, indexed_points.begin() + 6 },
    util::Range<Iter>{ indexed_points.begin() + 6, indexed_points.begin() + 7 },
    util::Range<Iter>{ indexed_points.begin() + 7, indexed_points.end() }
  };

  const auto actual_ranges =
    partition_points_into_child_octants(indexed_points.begin(), indexed_points.end(), 0);

  for (size_t idx = 0; idx < 8; ++idx) {
    REQUIRE(actual_ranges[idx].begin() == expected_ranges[idx].begin());
    REQUIRE(actual_ranges[idx].end() == expected_ranges[idx].end());
  }
}

TEST_CASE("Partitioning points at non-root level into child octants works correctly",
          "[partition_points_into_child_octants]")
{
  // Use multiple levels so that we can test the 'parent_node_level' parameter
  // of 'partition_points_into_child_octants'
  constexpr uint32_t Levels = 5;

  AABB bounds{ V3{ 0, 0, 0 }, V3{ 32, 32, 32 } };

  // Define octant indices first, generate positions from that and then check
  // the partitioning
  std::vector<std::array<uint8_t, Levels>> octant_indices = {
    { 3, 4, 5, 2, 0 }, { 3, 4, 5, 2, 0 }, { 3, 4, 5, 2, 3 },
    { 3, 4, 5, 2, 5 }, { 3, 4, 5, 2, 5 }, { 3, 4, 5, 2, 6 }
  };

  std::vector<V3> positions;
  std::transform(
    octant_indices.begin(),
    octant_indices.end(),
    std::back_inserter(positions),
    [&](const auto& indices) { return position_from_octant_indices<Levels>(indices, bounds); });

  PointBuffer points{ positions.size(), positions };

  std::vector<MortonIndex<Levels>> octree_indices;
  octree_indices.resize(points.count());
  calculate_morton_indices_for_points<Levels>(
    points.positions().begin(), points.positions().end(), octree_indices.begin(), bounds);

  std::vector<IndexedPoint<Levels>> indexed_points;
  indexed_points.reserve(positions.size());
  for (size_t idx = 0; idx < positions.size(); ++idx) {
    indexed_points.push_back(IndexedPoint<Levels>{ *(points.begin() + idx), octree_indices[idx] });
  }

  // The expected ranges are: [0;2) - (2,2) - (2,2) - [2,3) - (3,3) - [3,5) -
  // [5,6) - (6,6) We only store the offsets to the initial element here as this
  // is easier to report in a test
  std::array<std::pair<int, int>, 8> expected_ranges = {
    std::pair<int, int>{ 0, 2 }, std::pair<int, int>{ 2, 2 }, std::pair<int, int>{ 2, 2 },
    std::pair<int, int>{ 2, 3 }, std::pair<int, int>{ 3, 3 }, std::pair<int, int>{ 3, 5 },
    std::pair<int, int>{ 5, 6 }, std::pair<int, int>{ 6, 6 }
  };

  const auto actual_ranges =
    partition_points_into_child_octants(indexed_points.begin(), indexed_points.end(), 4);

  for (size_t idx = 0; idx < 8; ++idx) {
    const auto& cur_range = actual_ranges[idx];
    const auto actual_start_offset = std::distance(indexed_points.begin(), cur_range.begin());
    const auto actual_end_offset = std::distance(indexed_points.begin(), cur_range.end());
    REQUIRE(actual_start_offset == expected_ranges[idx].first);
    REQUIRE(actual_end_offset == expected_ranges[idx].second);
  }
}

TEST_CASE("Getting octant bounds from AABB works correctly", "[get_octant_bounds]")
{
  AABB root_bounds{ V3{ 0, 0, 0 }, V3{ 4, 4, 4 } };

  const auto bounds_octant0 = get_octant_bounds(uint8_t(0), root_bounds);
  const auto bounds_octant1 = get_octant_bounds(uint8_t(1), root_bounds);
  const auto bounds_octant2 = get_octant_bounds(uint8_t(2), root_bounds);
  const auto bounds_octant3 = get_octant_bounds(uint8_t(3), root_bounds);
  const auto bounds_octant4 = get_octant_bounds(uint8_t(4), root_bounds);
  const auto bounds_octant5 = get_octant_bounds(uint8_t(5), root_bounds);
  const auto bounds_octant6 = get_octant_bounds(uint8_t(6), root_bounds);
  const auto bounds_octant7 = get_octant_bounds(uint8_t(7), root_bounds);

  AABB expected_bounds_octant0{ V3{ 0, 0, 0 }, V3{ 2, 2, 2 } };
  AABB expected_bounds_octant1{ V3{ 0, 0, 2 }, V3{ 2, 2, 4 } };
  AABB expected_bounds_octant2{ V3{ 0, 2, 0 }, V3{ 2, 4, 2 } };
  AABB expected_bounds_octant3{ V3{ 0, 2, 2 }, V3{ 2, 4, 4 } };
  AABB expected_bounds_octant4{ V3{ 2, 0, 0 }, V3{ 4, 2, 2 } };
  AABB expected_bounds_octant5{ V3{ 2, 0, 2 }, V3{ 4, 2, 4 } };
  AABB expected_bounds_octant6{ V3{ 2, 2, 0 }, V3{ 4, 4, 2 } };
  AABB expected_bounds_octant7{ V3{ 2, 2, 2 }, V3{ 4, 4, 4 } };

  REQUIRE(bounds_octant0 == expected_bounds_octant0);
  REQUIRE(bounds_octant1 == expected_bounds_octant1);
  REQUIRE(bounds_octant2 == expected_bounds_octant2);
  REQUIRE(bounds_octant3 == expected_bounds_octant3);
  REQUIRE(bounds_octant4 == expected_bounds_octant4);
  REQUIRE(bounds_octant5 == expected_bounds_octant5);
  REQUIRE(bounds_octant6 == expected_bounds_octant6);
  REQUIRE(bounds_octant7 == expected_bounds_octant7);
}

TEST_CASE("Partitioning points into child octants ensures points are still "
          "contained in child bounds",
          "[partition_points_into_child_octants]")
{
  constexpr uint32_t Levels = 10;
  constexpr size_t NumPoints = 1024;

  std::default_random_engine rnd{ (uint32_t)time(nullptr) };
  std::uniform_int_distribution<int> dist{ 1024, 2048 };

  AABB bounds{ V3{ 1024, 1024, 1024 }, V3{ 2048, 2048, 2048 } };

  std::vector<V3> rnd_points;
  rnd_points.reserve(NumPoints);

  std::generate_n(std::back_inserter(rnd_points), NumPoints, [&]() {
    return V3{ static_cast<double>(dist(rnd)),
               static_cast<double>(dist(rnd)),
               static_cast<double>(dist(rnd)) };
  });

  PointBuffer points{ rnd_points.size(), rnd_points };

  std::vector<MortonIndex<Levels>> octree_indices;
  octree_indices.resize(points.count());
  calculate_morton_indices_for_points<Levels>(
    points.positions().begin(), points.positions().end(), octree_indices.begin(), bounds);

  std::vector<IndexedPoint<Levels>> indexed_points;
  indexed_points.reserve(rnd_points.size());
  for (size_t idx = 0; idx < rnd_points.size(); ++idx) {
    indexed_points.push_back(IndexedPoint<Levels>{ *(points.begin() + idx), octree_indices[idx] });
  }

  std::sort(indexed_points.begin(), indexed_points.end(), [](const auto& l, const auto& r) {
    return l.morton_index.get() < r.morton_index.get();
  });

  const auto partitioned_points_at_l0 =
    partition_points_into_child_octants(indexed_points.begin(), indexed_points.end(), 0);

  std::vector<AABB> child_bounds;
  child_bounds.reserve(8);
  for (uint8_t idx = 0; idx < uint8_t(8); ++idx) {
    child_bounds.push_back(get_octant_bounds(idx, bounds));
  }

  for (size_t idx = 0; idx < 8; ++idx) {
    const auto& child_range = partitioned_points_at_l0[idx];
    const auto& cur_bounds = child_bounds[idx];

    for (const auto& indexed_point : child_range) {
      REQUIRE(cur_bounds.isInside(indexed_point.point_reference.position()));
    }
  }
}

TEST_CASE("get_bounds_from_morton_index with one level works", "[get_bounds_from_morton_index]")
{
  MortonIndex64 key;
  key.set_octant_at_level(0, uint8_t(0));

  AABB bounds{ { 0, 0, 0 }, { 2, 2, 2 } };

  AABB expected_bounds{ { 0, 0, 0 }, { 1, 1, 1 } };
  const auto actual_bounds = get_bounds_from_morton_index(key, bounds, 1);
  REQUIRE(expected_bounds == actual_bounds);
}

TEST_CASE("get_bounds_from_morton_index with multiple levels works",
          "[get_bounds_from_morton_index]")
{
  MortonIndex64 key;
  key.set_octant_at_level(0, uint8_t(1)); // x € [0;4], y € [0;4], z € [4;8]
  key.set_octant_at_level(1, uint8_t(4)); // x € [2;4], y € [0;2], z € [4;6]
  key.set_octant_at_level(2, uint8_t(5)); // x € [3;4], y € [0;1], z € [5;6]

  AABB bounds{ { 0, 0, 0 }, { 8, 8, 8 } };

  AABB expected_bounds{ { 3, 0, 5 }, { 4, 1, 6 } };
  const auto actual_bounds = get_bounds_from_morton_index(key, bounds, 3);
  REQUIRE(expected_bounds == actual_bounds);
}

TEST_CASE("smart octree key calculation works")
{
  constexpr uint32_t Levels = 20;

  std::array<uint8_t, Levels> expected_octants{ 5, 3, 7, 4, 0, 1, 6, 4, 3, 5,
                                                3, 6, 7, 3, 2, 1, 4, 0, 2, 5 };
  AABB bounds{ V3{ 0, 0, 0 }, V3{ 1 << Levels, 1 << Levels, 1 << Levels } };

  const auto pos = position_from_octant_indices<Levels>(expected_octants, bounds);
  MortonIndex<Levels> expected_key{ expected_octants };

  const auto naive_key = calculate_morton_index_naive<Levels>(pos, bounds);
  const auto smart_key = calculate_morton_index<Levels>(pos, bounds);

  REQUIRE(smart_key == naive_key);
}