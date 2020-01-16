#pragma once

#include "AABB.h"
#include "MortonIndex.h"
#include "SparseGrid.h"
#include "util/Algorithm.h"

#include <random>
#include <unordered_set>
#include <variant>

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
                     float spacing_at_root)
  {
    const auto num_points_to_process = static_cast<size_t>(std::distance(begin, end));
    if (num_points_to_process <= _max_points_per_node) {
      return end;
    }

    const auto spacing_at_this_node = spacing_at_root / std::pow(2, node_level + 1);
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
               (int)std::floor(std::log2f(root_bounds.extent().x / spacing_at_this_node)) -
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
        begin, end, [this, &num_taken_indices, level](const auto curBegin, const auto end) {
          // Take the first point, then look for the next point that falls
          // into a different cell up to the current level
          const auto taken_iter = curBegin;
          ++num_taken_indices;
          // if (num_taken_indices == _max_points_per_node) {
          //   return std::make_pair(taken_iter, end);
          // }

          const auto taken_cell_idx = taken_iter->morton_index.truncate_to_level(level);
          // const auto next_iter =
          //   std::find_if(taken_iter + 1, end, [taken_cell_idx, level](const
          //   auto& other_point) {
          //     return
          //     other_point.morton_index.truncate_to_level(level).get() >
          //            taken_cell_idx.get();
          //   });
          const auto next_iter = std::partition_point(
            taken_iter + 1, end, [taken_cell_idx, level](const auto& other_point) {
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
                     float spacing_at_root)
  {
    const auto num_points_to_process = static_cast<size_t>(std::distance(begin, end));
    if (num_points_to_process <= _max_points_per_node) {
      return end;
    }

    const auto spacing_at_this_node = spacing_at_root / std::pow(2, node_level + 1);
    const auto candidate_level_in_octree =
      std::max(-1, (int)std::floor(std::log2f(root_bounds.extent().x / spacing_at_this_node)) - 1);
    auto partition_point = begin;

    if (candidate_level_in_octree == -1) {
      return ++partition_point;
    }
    size_t num_selected_points = 0;

    return stable_partition_with_jumps(
      begin,
      end,
      [this, candidate_level_in_octree, &num_selected_points, &root_bounds](const auto cur_begin,
                                                                            const auto cur_end) {
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
          [current_cell_idx, candidate_level_in_octree](const auto& other_point) {
            return other_point.morton_index.truncate_to_level(candidate_level_in_octree).get() <=
                   current_cell_idx.get();
          });

        // Find the point closest to the center of the current cell bounds
        const auto current_cell_bounds = get_bounds_from_morton_index(
          cur_begin->morton_index, root_bounds, candidate_level_in_octree + 1);
        const auto current_cell_center = current_cell_bounds.getCenter();

        const auto min_point = std::min_element(
          cur_begin, points_in_same_cell_end, [&current_cell_center](const auto& l, const auto& r) {
            const auto l_dist_to_center =
              l.point_reference.position().squaredDistanceTo(current_cell_center);
            const auto r_dist_to_center =
              r.point_reference.position().squaredDistanceTo(current_cell_center);
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
                     float spacing_at_root)
  {
    const auto num_points_to_process = static_cast<size_t>(std::distance(begin, end));
    if (num_points_to_process <= _max_points_per_node) {
      return end;
    }
    const auto bounds_at_this_node =
      get_bounds_from_morton_index(node_key, root_bounds, node_level + 1);
    const auto spacing_at_this_node = spacing_at_root / std::pow(2, node_level + 1);
    SparseGrid sparse_grid{ bounds_at_this_node, static_cast<float>(spacing_at_this_node) };
    size_t num_points_taken = 0;

    return std::stable_partition(
      begin, end, [this, &num_points_taken, &sparse_grid](const auto& point) {
        // if (num_points_taken == _max_points_per_node)
        //   return false;

        const auto accepted = sparse_grid.add(point.point_reference.position());
        if (!accepted)
          return false;

        ++num_points_taken;
        return true;
      });
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
                     float spacing_at_root)
  {
    const auto num_points_to_process = static_cast<size_t>(std::distance(begin, end));
    if (num_points_to_process <= _max_points_per_node) {
      return end;
    }

    const auto spacing_at_this_node = spacing_at_root / std::pow(2, node_level + 1);
    const auto candidate_level_in_octree =
      std::max(-1, (int)std::floor(std::log2f(root_bounds.extent().x / spacing_at_this_node)) - 1);
    auto partition_point = begin;

    if (candidate_level_in_octree == -1) {
      return ++partition_point;
    }

    const auto bounds_at_this_node =
      get_bounds_from_morton_index(node_key, root_bounds, node_level + 1);
    SparseGrid sparse_grid{ bounds_at_this_node, static_cast<float>(spacing_at_this_node) };

    // Density determines the ratio of points that will be analyzed, e.g. a
    // density of 0.1 means 10% of all points get analyzed, or in other words 9
    // out of 10 points are ignored
    const auto nth_point = static_cast<uint32_t>(std::round(1 / _density_per_level(node_level)));
    uint32_t point_counter = nth_point - 1; // Guarantees that at least one point is analyzed

    return std::stable_partition(
      begin, end, [this, &point_counter, nth_point, &sparse_grid](const auto& point) {
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
                     float spacing_at_root)
  {
    const auto num_points_to_process = static_cast<size_t>(std::distance(begin, end));
    if (num_points_to_process <= _max_points_per_node) {
      return end;
    }

    const auto spacing_at_this_node = spacing_at_root / std::pow(2, node_level + 1);
    const auto sqr_spacing = spacing_at_this_node * spacing_at_this_node;

    // auto last_taken_point_iter = begin;

    return stable_partition_with_jumps(begin, end, [&](const auto cur_begin, const auto cur_end) {
      // Take the current point, search for next point that is outside of
      // min distance
      const auto current_position = cur_begin->point_reference.position();
      const auto next_begin = std::find_if(cur_begin + 1, cur_end, [&](const auto& other) {
        return other.point_reference.position().squaredDistanceTo(current_position) >= sqr_spacing;
      });

      return std::make_pair(cur_begin, next_begin);
    });
  }

private:
  size_t _max_points_per_node;
};

using SamplingStrategy = std::variant<RandomSortedGridSampling,
                                      GridCenterSampling,
                                      PoissonDiskSampling,
                                      AdaptivePoissonDiskSampling>;

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

  throw std::runtime_error{ "Unrecognized sampling strategy name \"" + name + "\"" };
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
              float spacing_at_root)
{
  return std::visit(
    [&](auto& strategy) {
      return strategy.sample_points(begin, end, node_key, node_level, root_bounds, spacing_at_root);
    },
    sampling_strategy);
}