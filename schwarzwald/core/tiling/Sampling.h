#pragma once

#include "algorithms/Algorithm.h"
#include "datastructures/MortonGrid.h"
#include "datastructures/MortonIndex.h"
#include "datastructures/PointBuffer.h"
#include "datastructures/SparseGrid.h"
#include "math/AABB.h"

#include <random>
#include <unordered_set>
#include <variant>

constexpr static uint32_t NUM_PERMUTATIONS = 16;

constexpr static std::array<std::array<uint32_t, 16>, NUM_PERMUTATIONS>
  PERMUTATIONS_16 = {
    { { { 1, 3, 6, 2, 9, 4, 12, 11, 15, 7, 5, 10, 16, 13, 14, 8 } },
      { { 1, 3, 2, 6, 7, 13, 5, 12, 15, 8, 16, 11, 9, 14, 10, 4 } },
      { { 1, 9, 8, 13, 5, 12, 15, 16, 14, 11, 6, 2, 4, 10, 3, 7 } },
      { { 1, 2, 6, 5, 10, 4, 11, 13, 16, 14, 7, 15, 12, 8, 3, 9 } },
      { { 1, 8, 9, 13, 5, 11, 4, 7, 2, 10, 15, 14, 16, 12, 6, 3 } },
      { { 1, 4, 11, 6, 3, 5, 10, 14, 12, 13, 9, 2, 8, 16, 15, 7 } },
      { { 1, 2, 6, 12, 10, 4, 7, 3, 5, 13, 8, 15, 14, 11, 16, 9 } },
      { { 1, 6, 14, 16, 8, 5, 12, 13, 11, 15, 10, 4, 7, 3, 9, 2 } },
      { { 1, 5, 3, 9, 2, 4, 12, 8, 11, 10, 15, 16, 13, 7, 14, 6 } },
      { { 1, 8, 4, 7, 2, 6, 11, 5, 13, 14, 16, 15, 12, 10, 3, 9 } },
      { { 1, 5, 8, 13, 7, 9, 16, 14, 15, 12, 4, 10, 3, 11, 6, 2 } },
      { { 1, 9, 2, 4, 10, 7, 11, 14, 13, 5, 3, 8, 15, 16, 12, 6 } },
      { { 1, 7, 15, 8, 11, 3, 2, 4, 9, 6, 13, 14, 12, 16, 10, 5 } },
      { { 1, 2, 9, 7, 12, 16, 10, 5, 11, 3, 6, 14, 13, 15, 8, 4 } },
      { { 1, 9, 14, 6, 5, 12, 8, 11, 15, 16, 13, 7, 2, 4, 10, 3 } },
      { { 1, 8, 5, 10, 6, 14, 7, 13, 15, 9, 12, 16, 11, 3, 4, 2 } } }
  };

constexpr static std::array<std::array<uint32_t, 32>, NUM_PERMUTATIONS>
  PERMUTATIONS_32 = {
    { { { 1,  14, 7, 5,  6,  17, 13, 3,  10, 19, 23, 8,  22, 32, 29, 20,
          15, 21, 9, 25, 28, 27, 11, 26, 31, 18, 30, 24, 16, 2,  4,  12 } },
      { { 1, 12, 5, 18, 7,  13, 11, 25, 27, 26, 31, 21, 28, 32, 24, 19,
          3, 15, 2, 17, 20, 29, 23, 8,  9,  6,  22, 30, 16, 4,  14, 10 } },
      { { 1,  17, 10, 4,  9, 21, 5,  6,  20, 8,  18, 7,  14, 11, 24, 15,
          30, 22, 26, 12, 2, 13, 19, 27, 29, 25, 28, 23, 32, 31, 16, 3 } },
      { { 1,  9,  6,  17, 10, 13, 4,  5,  18, 14, 19, 21, 8,  3,  2,  11,
          25, 29, 23, 7,  22, 32, 24, 30, 20, 27, 15, 31, 16, 28, 26, 12 } },
      { { 1,  10, 7,  21, 9, 8,  24, 15, 16, 27, 19, 32, 30, 25, 29, 31,
          18, 23, 17, 2,  5, 20, 6,  13, 3,  11, 4,  14, 26, 22, 28, 12 } },
      { { 1,  12, 9,  4,  6,  19, 7,  10, 15, 2,  16, 5,  20, 13, 23, 30,
          31, 25, 21, 29, 28, 32, 22, 14, 26, 11, 27, 18, 24, 8,  17, 3 } },
      { { 1, 16, 13, 12, 17, 30, 28, 31, 20, 7,  2,  6,  14, 8,  24, 15,
          3, 4,  10, 22, 32, 25, 27, 23, 9,  18, 29, 21, 5,  19, 26, 11 } },
      { { 1,  7, 10, 21, 5,  9,  23, 12, 4,  13, 6,  2,  3,  16, 15, 25,
          11, 8, 24, 26, 31, 19, 17, 32, 27, 18, 30, 20, 28, 22, 29, 14 } },
      { { 1,  5,  6,  15, 28, 21, 10, 17, 14, 12, 2,  7,  22, 9,  8, 16,
          30, 32, 24, 27, 18, 4,  20, 31, 26, 11, 23, 29, 25, 19, 3, 13 } },
      { { 1, 4,  9,  5,  3,  17, 12, 20, 6,  19, 10, 2,  13, 15, 8,  23,
          7, 14, 26, 25, 29, 30, 18, 28, 22, 31, 16, 32, 21, 27, 24, 11 } },
      { { 1,  16, 27, 14, 23, 29, 26, 19, 32, 18, 28, 12, 13, 21, 6, 22,
          10, 8,  3,  2,  5,  17, 31, 25, 30, 20, 24, 15, 4,  11, 7, 9 } },
      { { 1,  4,  9,  2,  18, 7,  17, 3,  15, 5,  6,  8,  14, 22, 16, 30,
          21, 20, 31, 29, 25, 12, 19, 11, 26, 10, 23, 32, 27, 24, 28, 13 } },
      { { 1, 11, 23, 9,  16, 6,  19, 22, 14, 28, 12, 5,  21, 8, 10, 18,
          7, 13, 24, 20, 29, 26, 17, 32, 27, 31, 25, 30, 15, 3, 4,  2 } },
      { { 1, 6,  5,  21, 11, 24, 31, 28, 22, 14, 23, 29, 25, 16, 2, 10,
          8, 20, 30, 17, 32, 27, 12, 26, 19, 3,  4,  15, 18, 7,  9, 13 } },
      { { 1,  15, 31, 17, 25, 27, 24, 29, 28, 26, 22, 13, 6, 19, 3, 4,
          14, 20, 32, 21, 30, 18, 8,  23, 10, 2,  9,  12, 7, 11, 5, 16 } },
      { { 1,  6,  2,  12, 10, 19, 8,  21, 9,  13, 5,  11, 4,  20, 7, 14,
          26, 23, 24, 32, 31, 22, 16, 27, 29, 15, 30, 25, 28, 18, 3, 17 } } }
  };

constexpr static std::array<std::array<uint32_t, 64>, NUM_PERMUTATIONS>
  PERMUTATIONS_64 = {
    { { { 22, 42, 58, 48, 25, 56, 50, 35, 14, 37, 61, 49, 54, 47, 64, 32,
          57, 59, 55, 29, 38, 60, 51, 63, 41, 44, 15, 45, 21, 53, 39, 40,
          9,  36, 18, 28, 43, 62, 46, 19, 33, 16, 11, 8,  26, 34, 6,  27,
          2,  31, 20, 1,  12, 10, 23, 3,  7,  13, 5,  4,  30, 17, 24, 52 } },
      { { 24, 38, 43, 17, 49, 28, 46, 55, 48, 60, 29, 58, 33, 52, 42, 57,
          30, 61, 45, 34, 62, 53, 25, 50, 63, 51, 59, 56, 27, 54, 37, 18,
          40, 8,  6,  26, 4,  7,  11, 3,  5,  31, 1,  2,  19, 35, 21, 32,
          14, 10, 16, 23, 47, 41, 64, 44, 20, 15, 36, 13, 12, 22, 9,  39 } },
      { { 39, 56, 36, 64, 60, 35, 62, 47, 63, 40, 8,  32, 57, 27, 9,  11,
          2,  6,  14, 21, 4,  5,  28, 12, 25, 31, 46, 20, 13, 3,  15, 45,
          18, 7,  38, 26, 23, 17, 22, 44, 58, 50, 19, 29, 1,  10, 42, 61,
          37, 16, 34, 54, 41, 52, 55, 33, 59, 30, 51, 49, 48, 43, 24, 53 } },
      { { 34, 28, 35, 19, 43, 20, 42, 13, 27, 1,  30, 2,  29, 10, 38, 7,
          8,  24, 15, 41, 46, 56, 51, 47, 44, 63, 48, 60, 49, 17, 5,  36,
          61, 64, 62, 55, 59, 37, 54, 33, 16, 39, 50, 40, 58, 31, 23, 53,
          52, 22, 9,  18, 4,  25, 57, 32, 45, 21, 3,  11, 26, 6,  12, 14 } },
      { { 11, 20, 40, 15, 19, 6,  32, 12, 33, 43, 42, 26, 9,  41, 58, 55,
          28, 47, 62, 54, 60, 61, 31, 53, 25, 36, 52, 23, 4,  22, 30, 21,
          14, 45, 13, 38, 17, 2,  7,  5,  8,  10, 24, 1,  29, 3,  27, 16,
          39, 34, 63, 51, 37, 64, 46, 59, 49, 18, 48, 44, 56, 50, 57, 35 } },
      { { 14, 42, 50, 59, 48, 63, 61, 40, 47, 51, 29, 60, 31, 58, 27, 57,
          54, 36, 41, 52, 43, 38, 56, 37, 17, 34, 53, 26, 11, 32, 18, 44,
          64, 39, 13, 45, 15, 8,  33, 62, 49, 55, 23, 46, 22, 16, 6,  5,
          19, 35, 12, 24, 7,  10, 20, 4,  28, 30, 2,  3,  25, 21, 9,  1 } },
      { { 16, 46, 23, 40, 14, 2,  34, 6,  13, 11, 4,  35, 37, 18, 10, 32,
          7,  26, 20, 17, 3,  27, 47, 50, 19, 9,  5,  30, 12, 33, 1,  28,
          39, 15, 21, 25, 43, 22, 51, 24, 8,  36, 41, 56, 57, 42, 52, 64,
          53, 31, 44, 60, 59, 29, 55, 63, 54, 49, 58, 38, 61, 48, 62, 45 } },
      { { 29, 50, 23, 52, 21, 10, 28, 16, 22, 4,  3,  31, 14, 7,  32, 59,
          56, 46, 49, 35, 12, 8,  25, 6,  38, 54, 33, 41, 26, 39, 9,  20,
          42, 13, 17, 1,  2,  11, 5,  24, 55, 47, 15, 45, 19, 34, 48, 53,
          44, 64, 40, 27, 51, 61, 63, 43, 18, 30, 37, 60, 58, 36, 62, 57 } },
      { { 42, 55, 23, 48, 24, 56, 54, 45, 57, 59, 29, 51, 35, 63, 38, 62,
          33, 52, 32, 61, 44, 60, 64, 49, 27, 47, 58, 50, 53, 40, 37, 36,
          43, 16, 21, 11, 26, 5,  28, 2,  19, 46, 18, 39, 8,  34, 22, 4,
          14, 3,  12, 30, 7,  1,  31, 17, 25, 6,  20, 13, 9,  15, 10, 41 } },
      { { 38, 22, 17, 49, 56, 41, 15, 9,  10, 14, 7,  36, 25, 6,  21, 4,
          12, 3,  24, 35, 60, 28, 31, 13, 44, 23, 39, 11, 30, 16, 8,  32,
          2,  29, 57, 45, 20, 50, 19, 42, 47, 18, 5,  1,  27, 33, 55, 52,
          54, 34, 46, 64, 37, 51, 61, 59, 58, 48, 26, 43, 63, 40, 53, 62 } },
      { { 53, 26, 48, 64, 63, 37, 58, 34, 19, 9,  13, 2,  31, 14, 10, 1,
          32, 62, 49, 18, 6,  16, 24, 29, 43, 23, 41, 11, 22, 47, 44, 21,
          40, 12, 38, 50, 25, 42, 28, 56, 51, 57, 39, 7,  30, 8,  35, 55,
          36, 15, 17, 20, 33, 4,  5,  3,  27, 59, 52, 61, 45, 60, 54, 46 } },
      { { 63, 54, 41, 18, 48, 47, 35, 62, 45, 17, 28, 25, 50, 36, 4,  20,
          5,  10, 32, 56, 29, 44, 13, 34, 53, 42, 16, 19, 51, 59, 61, 55,
          37, 7,  33, 64, 60, 40, 58, 39, 43, 57, 52, 30, 9,  38, 22, 12,
          21, 49, 24, 31, 2,  15, 8,  14, 6,  26, 27, 3,  1,  11, 23, 46 } },
      { { 25, 9,  4,  29, 18, 10, 6,  27, 15, 16, 35, 3,  19, 51, 22, 2,
          26, 7,  5,  20, 24, 1,  31, 45, 14, 32, 8,  13, 42, 62, 34, 21,
          11, 39, 12, 43, 46, 54, 28, 41, 58, 33, 60, 38, 17, 23, 49, 40,
          52, 61, 63, 48, 30, 53, 36, 47, 57, 64, 50, 44, 37, 59, 56, 55 } },
      { { 15, 7,  27, 44, 25, 52, 24, 56, 29, 16, 23, 36, 10, 39, 8,  32,
          21, 35, 6,  37, 62, 42, 58, 57, 41, 45, 63, 51, 61, 43, 54, 31,
          17, 47, 55, 60, 38, 64, 49, 19, 40, 59, 34, 30, 53, 50, 33, 28,
          22, 1,  13, 3,  12, 18, 9,  11, 4,  26, 2,  5,  20, 48, 46, 14 } },
      { { 59, 35, 5,  13, 41, 23, 53, 44, 12, 1,  3,  15, 11, 40, 25, 57,
          29, 10, 20, 33, 32, 56, 51, 52, 36, 54, 31, 58, 61, 39, 64, 43,
          50, 30, 45, 38, 55, 49, 63, 60, 46, 17, 22, 48, 21, 42, 62, 37,
          24, 47, 16, 27, 19, 7,  26, 9,  18, 34, 8,  14, 4,  2,  6,  28 } },
      { { 26, 10, 42, 18, 47, 59, 52, 35, 12, 7,  29, 4,  17, 43, 11, 5,
          20, 21, 32, 53, 60, 39, 45, 27, 16, 3,  13, 15, 31, 2,  6,  34,
          22, 41, 50, 19, 46, 38, 24, 49, 40, 14, 37, 9,  23, 54, 57, 55,
          36, 56, 64, 44, 61, 51, 48, 33, 63, 62, 58, 28, 1,  25, 30, 8 } } }
  };
namespace octree {
struct NodeStructure;
}

/**
 * A reference to a cached point in a PointBuffer, together with the points
 * octree index
 */
template<unsigned int MaxLevels>
struct IndexedPoint
{
  PointBuffer::PointReference point_reference;
  MortonIndex<MaxLevels> morton_index;
};

/**
 * IndexedPoint using 64-bit MortonIndex
 */
using IndexedPoint64 = IndexedPoint<21>;

template<unsigned int MaxLevels>
bool
operator<(const IndexedPoint<MaxLevels>& l, const IndexedPoint<MaxLevels>& r)
{
  return l.morton_index.get() < r.morton_index.get();
}

/**
 * Different behaviours for the SamplingStrategies based on the number of points
 * that they process
 */
enum class SamplingBehaviour
{
  /**
   * If 'sample_points' is called with a range that has <= max_points points,
   * the sampling process will be skipped and all points are taken instead
   */
  TakeAllWhenCountBelowMaxPoints,
  /**
   * No matter the point count, always adhere to the minimum spacing
   */
  AlwaysAdhereToMinSpacing
};

/**
 * Sampling strategy that partitions a node into an even grid and takes the
 * first point to fall into each grid cell
 */
struct RandomSortedGridSampling
{
  explicit RandomSortedGridSampling(size_t max_points_per_node);

  template<typename Iter, unsigned int MaxLevels>
  Iter sample_points(Iter begin,
                     Iter end,
                     MortonIndex<MaxLevels> node_key,
                     int32_t node_level,
                     const AABB& root_bounds,
                     float spacing_at_root,
                     SamplingBehaviour sampling_behaviour =
                       SamplingBehaviour::TakeAllWhenCountBelowMaxPoints)
  {
    if (sampling_behaviour ==
        SamplingBehaviour::TakeAllWhenCountBelowMaxPoints) {
      const auto num_points_to_process =
        static_cast<size_t>(std::distance(begin, end));
      if (num_points_to_process <= _max_points_per_node) {
        return end;
      }
    }

    const auto spacing_at_this_node =
      spacing_at_root / std::pow(2, node_level + 1);
    // candidate_level_in_octree is the last level in the octree at which the
    // node sidelength is >= spacing we use floor() here because this guarantees
    // that we always get a node with sidelength
    // >= spacing. Sadly this means that there might be edge cases where the
    // spacing is just a bit too large to fit into a node (e.g. sidelength = 2,
    // spacing = 2.1) If we don't want that, we could use round() which resolves
    // these cases while at the same time dropping the guarantee for >= spacing.
    // Also, round(log(...)) might not split fair. If there are sidelengths 4
    // and 2, we would expect that spacing >= 3 rounds to node with sidelength
    // 4, while < 3 rounds to 2. However, log_2(4/2.9)~=0.464, which will still
    // round to the larger node
    const auto candidate_level_in_octree =
      std::max(-1,
               (int)std::floor(
                 std::log2f(root_bounds.extent().x / spacing_at_this_node)) -
                 1); // the root node (whole octree) is level '-1', so level
                     // 0 has a sidelength of half the max octree, hence we
                     // have to subtract one here
    auto partition_point = begin;

    const auto stable_partition_at_level = [&](int level) {
      // TODO stable_partition looks at all elements, but since the elements are
      // sorted, once we found the first point in a grid cell, we can skip to
      // the next grid cell. We know the index of the next grid cell (just
      // increment the level of the current cell by one)
      // std::unordered_set<uint64_t> taken_indices;
      // partition_point =
      //   std::stable_partition(begin, end, [this, &taken_indices, level](const
      //   auto& indexed_point) {
      //     if (taken_indices.size() >= _max_points_per_node)
      //       return false;

      //     const auto significant_bits =
      //     indexed_point.morton_index.truncate_to_level(level); if
      //     (taken_indices.find(significant_bits.get()) != taken_indices.end())
      //       return false;

      //     taken_indices.insert(significant_bits.get());
      //     return true;
      //   });

      size_t num_taken_indices = 0;
      partition_point = stable_partition_with_jumps(
        begin,
        end,
        [this, &num_taken_indices, level](const auto curBegin, const auto end) {
          // Take the first point, then look for the next point that falls
          // into a different cell up to the current level
          const auto taken_iter = curBegin;
          ++num_taken_indices;
          // if (num_taken_indices == _max_points_per_node) {
          //   return std::make_pair(taken_iter, end);
          // }

          const auto taken_cell_idx =
            taken_iter->morton_index.truncate_to_level(level);
          // const auto next_iter =
          //   std::find_if(taken_iter + 1, end, [taken_cell_idx, level](const
          //   auto& other_point) {
          //     return
          //     other_point.morton_index.truncate_to_level(level).get() >
          //            taken_cell_idx.get();
          //   });
          const auto next_iter = std::partition_point(
            taken_iter + 1,
            end,
            [taken_cell_idx, level](const auto& other_point) {
              return other_point.morton_index.truncate_to_level(level).get() <=
                     taken_cell_idx.get();
            });

          return std::make_pair(taken_iter, next_iter);
        });
    };

    // Dirty, but we need a special function to partition at root (level -1) if
    // this situation ever occurs. We just take the first point and are done
    // with it
    const auto partition_at_root = [&]() {
      if (begin == end)
        return;

      ++partition_point;
    };

    if (candidate_level_in_octree == -1) {
      partition_at_root();
    } else {
      stable_partition_at_level(candidate_level_in_octree);
    }

    return partition_point;
  }

private:
  size_t _max_points_per_node;
};

/**
 * Sampling strategy that partitions a node into an even grid and takes the
 * point closest to the center of each cell
 */
struct GridCenterSampling
{
  explicit GridCenterSampling(size_t max_points_per_node);

  template<typename Iter, unsigned int MaxLevels>
  Iter sample_points(Iter begin,
                     Iter end,
                     MortonIndex<MaxLevels> node_key,
                     int32_t node_level,
                     const AABB& root_bounds,
                     float spacing_at_root,
                     SamplingBehaviour sampling_behaviour =
                       SamplingBehaviour::TakeAllWhenCountBelowMaxPoints)
  {
    if (sampling_behaviour ==
        SamplingBehaviour::TakeAllWhenCountBelowMaxPoints) {
      const auto num_points_to_process =
        static_cast<size_t>(std::distance(begin, end));
      if (num_points_to_process <= _max_points_per_node) {
        return end;
      }
    }

    const auto spacing_at_this_node =
      spacing_at_root / std::pow(2, node_level + 1);
    const auto candidate_level_in_octree =
      std::max(-1,
               (int)std::floor(
                 std::log2f(root_bounds.extent().x / spacing_at_this_node)) -
                 1);
    auto partition_point = begin;

    if (candidate_level_in_octree == -1) {
      return ++partition_point;
    }
    size_t num_selected_points = 0;

    // TODO Can we write this method in a way that it prevents the out-of-bounds
    // errors?

    return stable_partition_with_jumps(
      begin,
      end,
      [this, candidate_level_in_octree, &num_selected_points, &root_bounds](
        const auto cur_begin, const auto cur_end) {
        /*
        HACK truncate_to_level shifts down but it should just mask away the
        lower levels. This causes bugs because the new key starts at
        'candidate_level_in_octree', but get_bounds_from_morton_index starts
        at level zero up to 'candidate_level_in_octree', which of course is
        all zeroes...
        */
        const auto current_cell_idx =
          cur_begin->morton_index.truncate_to_level(candidate_level_in_octree);
        // const auto points_in_same_cell_end = std::find_if(
        //   cur_begin + 1,
        //   cur_end,
        //   [current_cell_idx, candidate_level_in_octree](const auto&
        //   other_point) {
        //     return
        //     other_point.morton_index.truncate_to_level(candidate_level_in_octree)
        //              .get() > current_cell_idx.get();
        //   });
        const auto points_in_same_cell_end = std::partition_point(
          cur_begin + 1,
          cur_end,
          [current_cell_idx,
           candidate_level_in_octree](const auto& other_point) {
            return other_point.morton_index
                     .truncate_to_level(candidate_level_in_octree)
                     .get() <= current_cell_idx.get();
          });

        // Find the point closest to the center of the current cell bounds
        const auto current_cell_bounds = get_bounds_from_morton_index(
          cur_begin->morton_index, root_bounds, candidate_level_in_octree + 1);
        const auto current_cell_center = current_cell_bounds.getCenter();

        const auto min_point = std::min_element(
          cur_begin,
          points_in_same_cell_end,
          [&current_cell_center](const auto& l, const auto& r) {
            const auto l_dist_to_center =
              l.point_reference.position().squaredDistanceTo(
                current_cell_center);
            const auto r_dist_to_center =
              r.point_reference.position().squaredDistanceTo(
                current_cell_center);
            return l_dist_to_center < r_dist_to_center;
          });

        // ++num_selected_points;
        // if (num_selected_points == _max_points_per_node) {
        //   return std::make_pair(min_point, cur_end);
        // }

        return std::make_pair(min_point, points_in_same_cell_end);
      });
  }

private:
  size_t _max_points_per_node;
};

/**
 * Sampling strategy that takes points with a minimum distance
 */
struct PoissonDiskSampling
{
  explicit PoissonDiskSampling(size_t max_points_per_node);

  template<typename Iter, unsigned int MaxLevels>
  Iter sample_points(Iter begin,
                     Iter end,
                     MortonIndex<MaxLevels> node_key,
                     int32_t node_level,
                     const AABB& root_bounds,
                     float spacing_at_root,
                     SamplingBehaviour sampling_behaviour =
                       SamplingBehaviour::TakeAllWhenCountBelowMaxPoints)
  {
    if (sampling_behaviour ==
        SamplingBehaviour::TakeAllWhenCountBelowMaxPoints) {
      const auto num_points_to_process =
        static_cast<size_t>(std::distance(begin, end));
      if (num_points_to_process <= _max_points_per_node) {
        return end;
      }
    }

    const auto bounds_at_this_node =
      get_bounds_from_morton_index(node_key, root_bounds, node_level + 1);
    const auto spacing_at_this_node =
      spacing_at_root / std::pow(2, node_level + 1);
    SparseGrid sparse_grid{ bounds_at_this_node,
                            static_cast<float>(spacing_at_this_node) };
    size_t num_points_taken = 0;

    const auto ret = std::stable_partition(
      begin, end, [this, &num_points_taken, &sparse_grid](const auto& point) {
        // if (num_points_taken == _max_points_per_node)
        //   return false;

        const auto accepted = sparse_grid.add(point.point_reference.position());
        if (!accepted)
          return false;

        ++num_points_taken;
        return true;
      });

    std::cout << to_string(node_key) << " - "
              << sparse_grid.dbg_num_comparisons() << " comparisons\n";
    return ret;
  }

private:
  size_t _max_points_per_node;
  std::default_random_engine _rnd;
};

/**
 * Poisson disk sampling, but it can skip points entirely depending on a density
 * function based on the level of the sampled node
 */
struct AdaptivePoissonDiskSampling
{
  AdaptivePoissonDiskSampling(size_t max_points_per_node,
                              std::function<float(int32_t)> density_per_level);

  template<typename Iter, unsigned int MaxLevels>
  Iter sample_points(Iter begin,
                     Iter end,
                     MortonIndex<MaxLevels> node_key,
                     int32_t node_level,
                     const AABB& root_bounds,
                     float spacing_at_root,
                     SamplingBehaviour sampling_behaviour =
                       SamplingBehaviour::TakeAllWhenCountBelowMaxPoints)
  {
    if (sampling_behaviour ==
        SamplingBehaviour::TakeAllWhenCountBelowMaxPoints) {
      const auto num_points_to_process =
        static_cast<size_t>(std::distance(begin, end));
      if (num_points_to_process <= _max_points_per_node) {
        return end;
      }
    }

    const auto spacing_at_this_node =
      spacing_at_root / std::pow(2, node_level + 1);
    const auto candidate_level_in_octree =
      std::max(-1,
               (int)std::floor(
                 std::log2f(root_bounds.extent().x / spacing_at_this_node)) -
                 1);
    auto partition_point = begin;

    if (candidate_level_in_octree == -1) {
      return ++partition_point;
    }

    const auto bounds_at_this_node =
      get_bounds_from_morton_index(node_key, root_bounds, node_level + 1);
    SparseGrid sparse_grid{ bounds_at_this_node,
                            static_cast<float>(spacing_at_this_node) };

    // Density determines the ratio of points that will be analyzed, e.g. a
    // density of 0.1 means 10% of all points get analyzed, or in other words 9
    // out of 10 points are ignored
    const auto nth_point =
      static_cast<uint32_t>(std::round(1 / _density_per_level(node_level)));
    uint32_t point_counter =
      nth_point - 1; // Guarantees that at least one point is analyzed

    return std::stable_partition(
      begin,
      end,
      [this, &point_counter, nth_point, &sparse_grid](const auto& point) {
        if (++point_counter == nth_point) {
          point_counter = 0;
          return sparse_grid.add(point.point_reference.position());
        }
        return false;
      });
  }

private:
  size_t _max_points_per_node;
  std::function<float(int32_t)> _density_per_level;
};

/**
 * Sampling that takes the next point (in Z-order sorting) that is further away
 * than the minimum distance from the previously selected point
 */
struct ZOrderNextSampling
{
  explicit ZOrderNextSampling(size_t max_points_per_node)
    : _max_points_per_node(max_points_per_node)
  {}

  template<typename Iter, unsigned int MaxLevels>
  Iter sample_points(Iter begin,
                     Iter end,
                     MortonIndex<MaxLevels> node_key,
                     int32_t node_level,
                     const AABB& root_bounds,
                     float spacing_at_root,
                     SamplingBehaviour sampling_behaviour =
                       SamplingBehaviour::TakeAllWhenCountBelowMaxPoints)
  {
    if (sampling_behaviour ==
        SamplingBehaviour::TakeAllWhenCountBelowMaxPoints) {
      const auto num_points_to_process =
        static_cast<size_t>(std::distance(begin, end));
      if (num_points_to_process <= _max_points_per_node) {
        return end;
      }
    }

    const auto spacing_at_this_node =
      spacing_at_root / std::pow(2, node_level + 1);
    const auto sqr_spacing = spacing_at_this_node * spacing_at_this_node;

    // auto last_taken_point_iter = begin;

    return stable_partition_with_jumps(
      begin, end, [&](const auto cur_begin, const auto cur_end) {
        // Take the current point, search for next point that is outside of
        // min distance
        const auto current_position = cur_begin->point_reference.position();
        const auto next_begin =
          std::find_if(cur_begin + 1, cur_end, [&](const auto& other) {
            return other.point_reference.position().squaredDistanceTo(
                     current_position) >= sqr_spacing;
          });

        return std::make_pair(cur_begin, next_begin);
      });
  }

private:
  size_t _max_points_per_node;
};

struct JitteredSampling
{
  explicit JitteredSampling(size_t max_points_per_node);

  template<typename Iter, unsigned int MaxLevels>
  Iter sample_points(Iter begin,
                     Iter end,
                     MortonIndex<MaxLevels> node_key,
                     int32_t node_level,
                     const AABB& root_bounds,
                     float spacing_at_root,
                     SamplingBehaviour sampling_behaviour =
                       SamplingBehaviour::TakeAllWhenCountBelowMaxPoints)
  {
    const auto num_points_to_process =
      static_cast<size_t>(std::distance(begin, end));
    if (sampling_behaviour ==
        SamplingBehaviour::TakeAllWhenCountBelowMaxPoints) {
      if (num_points_to_process <= _max_points_per_node) {
        return end;
      }
    }

    // From the spacing and node level, figure out the size of the permutation
    // grid
    const AABB bounds_at_this_node =
      get_bounds_from_morton_index(node_key, root_bounds, node_level + 1);
    const auto spacing_at_this_node =
      spacing_at_root / std::pow(2, node_level + 1);
    const auto perfect_cell_count =
      bounds_at_this_node.extent().x / spacing_at_this_node;
    const auto actual_cell_count =
      get_prev_power_of_two(static_cast<uint32_t>(perfect_cell_count));

    if (actual_cell_count < 16) {
      throw std::runtime_error{ "Grids smaller than 16x16 "
                                "are not supported currently!" };
    }

    const uint32_t levels = static_cast<uint32_t>(std::log2(actual_cell_count));

    // grid_level is the total number of levels in the Morton index from the
    // root node to the cell size of the permutation grid
    const uint32_t grid_level = static_cast<uint32_t>(node_level + levels);
    if (grid_level >= MortonIndex64Levels) {
      const auto msg =
        (boost::format(
           "Node %1% at level %2% is too small to be sampled with "
           "ImprovedPoissonSampling!\n\tRoot spacing: %3%\n\tSpacing at this "
           "node: %4%\n\tGrid dimensions: %5%\n\tGrid level: %6%") %
         to_string(node_key, MortonIndexNamingConvention::Potree) % node_level %
         spacing_at_root % spacing_at_this_node % actual_cell_count %
         grid_level)
          .str();
      throw std::runtime_error{ msg };
    }

    // grid_mask is a bitmask that extracts the Morton index of the permutation
    // grid relative to the bounding box of the current node
    const uint64_t grid_mask = (1ull << (3 * levels)) - 1ull;
    const double grid_cell_size =
      bounds_at_this_node.extent().x / actual_cell_count;
    const double permutation_cell_size = grid_cell_size / actual_cell_count;

    // Select 3 permutations from the appropriate level at random
    const std::array<uint32_t const*, 3> permutations =
      [this,
       actual_cell_count,
       node_level]() -> std::array<uint32_t const*, 3> {
      const auto start_index =
        (3 * static_cast<uint32_t>(node_level + 1)) % NUM_PERMUTATIONS;
      if (actual_cell_count <= 16) {
        return { {
          PERMUTATIONS_16[start_index].data(),
          PERMUTATIONS_16[(start_index + 1) % 16].data(),
          PERMUTATIONS_16[(start_index + 2) % 16].data(),
        } };
      }
      if (actual_cell_count <= 32) {
        return { {
          PERMUTATIONS_32[start_index].data(),
          PERMUTATIONS_32[(start_index + 1) % 16].data(),
          PERMUTATIONS_32[(start_index + 2) % 16].data(),
        } };
      }

      // For all grids >= 64 cells, we use the permutation of size 64 and simply
      // repeat it
      return { {
        PERMUTATIONS_64[start_index].data(),
        PERMUTATIONS_64[(start_index + 1) % 16].data(),
        PERMUTATIONS_64[(start_index + 2) % 16].data(),
      } };
    }();

    const auto permutation_length = std::min<uint32_t>(actual_cell_count, 64);

    return stable_partition_with_jumps(
      begin,
      end,
      [grid_level,
       levels,
       &permutations,
       &bounds_at_this_node,
       grid_cell_size,
       permutation_cell_size,
       grid_mask,
       permutation_length](const auto cur_begin, const auto cur_end) {
        const IndexedPoint<MaxLevels>& cur_point = *cur_begin;
        const uint64_t cur_point_relative_morton_index =
          cur_point.morton_index.truncate_to_level(grid_level).get();
        const auto octree_node_index =
          OctreeNodeIndex64::unchecked_from_index_and_levels(
            cur_point_relative_morton_index & grid_mask, levels);
        const auto grid_index = octree_node_index.to_grid_index();

        const auto first_point_in_next_cell = std::partition_point(
          cur_begin + 1,
          cur_end,
          [cur_point_relative_morton_index,
           grid_level](const auto& other_point) {
            return other_point.morton_index.truncate_to_level(grid_level)
                     .get() <= cur_point_relative_morton_index;
          });

        // The three permutations define a point within the grid cell. Select
        // the closest point in the current range to that point
        const auto px =
          permutations[0][(grid_index.y + grid_index.z) % permutation_length] -
          1;
        const auto py =
          permutations[1][(grid_index.x + grid_index.z) % permutation_length] -
          1;
        const auto pz =
          permutations[2][(grid_index.x + grid_index.y) % permutation_length] -
          1;

        const auto target_position = bounds_at_this_node.min + Vector3<double>{
          grid_index.x * grid_cell_size + px * permutation_cell_size,
          grid_index.y * grid_cell_size + py * permutation_cell_size,
          grid_index.z * grid_cell_size + pz * permutation_cell_size
        };

        const auto min_point = std::min_element(
          cur_begin,
          first_point_in_next_cell,
          [&target_position](const auto& l, const auto& r) {
            const auto l_dist_to_center =
              l.point_reference.position().squaredDistanceTo(target_position);
            const auto r_dist_to_center =
              r.point_reference.position().squaredDistanceTo(target_position);
            return l_dist_to_center < r_dist_to_center;
          });

        return std::make_pair(min_point, first_point_in_next_cell);
      });
  }

private:
  size_t _max_points_per_node;
  std::default_random_engine _rnd;
};

using SamplingStrategy = std::variant<RandomSortedGridSampling,
                                      GridCenterSampling,
                                      PoissonDiskSampling,
                                      AdaptivePoissonDiskSampling,
                                      JitteredSampling>;

template<typename T, typename... Args>
SamplingStrategy
make_sampling_strategy(Args&&... args)
{
  return T{ std::forward<Args>(args)... };
}

template<typename... Args>
SamplingStrategy
make_sampling_strategy_from_name(const std::string& name, Args&&... args)
{
  if (name == "RANDOM_GRID")
    return RandomSortedGridSampling{ std::forward<Args>(args)... };
  if (name == "GRID_CENTER")
    return GridCenterSampling{ std::forward<Args>(args)... };
  if (name == "MIN_DISTANCE")
    return PoissonDiskSampling{ std::forward<Args>(args)... };
  if (name == "MIN_DISTANCE_FAST")
    return AdaptivePoissonDiskSampling{ std::forward<Args>(args)... };
  if (name == "JITTERED")
    return JitteredSampling{ std::forward<Args>(args)... };

  throw std::runtime_error{ "Unrecognized sampling strategy name \"" + name +
                            "\"" };
}

/**
 * Sample points for the given node from a range of points using the given
 * sampling strategy. Returns a partition point in the range of points where
 * [begin, partition_point) contains all sampled points and [partition_point,
 * end) all remaining points. Partitioning is stable
 */
template<typename Iter, unsigned int MaxLevels>
Iter
sample_points(SamplingStrategy& sampling_strategy,
              Iter begin,
              Iter end,
              MortonIndex<MaxLevels> node_key,
              int32_t node_level,
              const AABB& root_bounds,
              float spacing_at_root,
              SamplingBehaviour sampling_behaviour)
{
  return std::visit(
    [&](auto& strategy) {
      return strategy.sample_points(begin,
                                    end,
                                    node_key,
                                    node_level,
                                    root_bounds,
                                    spacing_at_root,
                                    sampling_behaviour);
    },
    sampling_strategy);
}

/**
 * Which level of Morton indices does the given sampling strategy require for
 * the given node level?
 *
 * This method is a bit of a hack. Some sampling strategies (like
 * RANDOM_SORTED_GRID and GRID_CENTER) use the Morton indices to perform their
 * sampling in a fast way, while others (MIN_DISTANCE) use separate data
 * structures for this task. The first type of sampling may use Morton indices
 * of a deeper level than the level of the node that is processed. Since Morton
 * indices have a fixed depth, knowing this value for each node is required to
 * decide how to proceed with a node (sample regularily, calculate deeper Morton
 * indices, treat as terminal node)
 */
int32_t
required_morton_index_depth(const SamplingStrategy& sampling_strategy,
                            int32_t node_level,
                            const octree::NodeStructure& root_node);