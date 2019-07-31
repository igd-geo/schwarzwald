#pragma once

#include "AABB.h"
#include "OctreeNodeKey.h"
#include "PointBuffer.h"

#include <unordered_set>
#include <vector>

/**
 * A reference to a cached point in a PointBuffer, together with the points octree index
 */
template<unsigned int MaxLevels>
struct IndexedPoint
{
  Potree::PointBuffer::PointReference point_reference;
  OctreeNodeKey<MaxLevels> octree_node_index;
};

/**
 * Returns a collection of IndexedPoints from the given set of OctreeNodeKeys and the given
 * PointBuffer
 */
template<unsigned int MaxLevels>
std::vector<IndexedPoint<MaxLevels>>
index_points(const std::vector<OctreeNodeKey<MaxLevels>>& octree_node_keys,
             Potree::PointBuffer& points)
{
  assert(octree_node_keys.size() == points.count());

  std::vector<IndexedPoint<MaxLevels>> indexed_points;
  indexed_points.reserve(points.count());
  for (size_t idx = 0; idx < points.count(); ++idx) {
    indexed_points.push_back(
      IndexedPoint<MaxLevels>{ points.get_point(idx), octree_node_keys[idx] });
  }

  return indexed_points;
}

/**
 * Returns the bounding box of the given octant of the parent bounding box
 */
Potree::AABB
get_octant_bounds(uint8_t octant, const Potree::AABB& parent_bounds);

/**
 * Returns the index of the octant of 'bounds' that the given position lies in
 */
uint8_t
get_octant(const Potree::Vector3<double>& position, const Potree::AABB& bounds);

/**
 * Calculates the OctreeNodeKey for a given position starting from 'node_bounds' as root node
 */
template<unsigned int MaxLevels>
OctreeNodeKey<MaxLevels>
calculate_octree_key(const Potree::Vector3<double>& position, const Potree::AABB& node_bounds)
{
  OctreeNodeKey<MaxLevels> key;
  auto cur_bounds = node_bounds;
  for (uint32_t level = 0; level < MaxLevels; ++level) {
    auto cur_octant = get_octant(position, cur_bounds);
    key.set_octant_at_level(level, cur_octant);
    cur_bounds = get_octant_bounds(cur_octant, cur_bounds);
  }
  return key;
}

template<unsigned int MaxLevels>
Potree::AABB
get_bounds_from_octree_key(const OctreeNodeKey<MaxLevels>& key,
                           const Potree::AABB& root_bounds,
                           uint32_t depth = MaxLevels)
{
  auto bounds = root_bounds;
  const auto max_level = std::min(depth, MaxLevels);
  for (uint32_t level = 0; level < max_level; ++level) {
    bounds = get_octant_bounds(key.get_octant_at_level(level), bounds);
  }
  return bounds;
}

/**
 * Calculates an index into the octree spanned by the given bounding box for each of the given
 * points. The index represents the octant at each level of the octree that a point belongs to.
 */
template<unsigned int MaxLevels, typename KeysIter, typename PointsIter>
void
calculate_octree_keys_for_points(PointsIter points_begin, //*points_begin is Vector3<double>
                                 PointsIter points_end,
                                 KeysIter keys_begin,
                                 const Potree::AABB& octree_bounds)
{
  std::transform(points_begin, points_end, keys_begin, [&](const auto& position) {
    OctreeNodeKey<MaxLevels> key;
    auto cur_bounds = octree_bounds;
    for (uint32_t level = 0; level < MaxLevels; ++level) {
      auto cur_octant = get_octant(position, cur_bounds);
      key.set_octant_at_level(level, cur_octant);
      cur_bounds = get_octant_bounds(cur_octant, cur_bounds);
    }
    return key;
  });
}

/**
 * Partitions the given range of points into two ranges where the first range [begin,center)
 * contains all points that belong to the given octree node and the second range [center, end)
 * contains all points that don't belong to this node. Returns 'center'. Partitioning is stable,
 * i.e. it preserves the relative order between elements within the two ranges.
 *
 * The octree root node has 'node_level' -1, the first children start at level 0
 */
template<typename Iter, unsigned int MaxLevels>
Iter
filter_points_for_octree_node(Iter points_begin, // *Iter is IndexedPoint<MaxLevels>
                              Iter points_end,
                              const Potree::AABB& octree_bounds,
                              OctreeNodeKey<MaxLevels> node_key,
                              int32_t node_level,
                              float spacing_at_root,
                              size_t max_points_per_node)
{
  assert(points_end >= points_begin);
  const auto num_points_to_process = static_cast<size_t>(std::distance(points_begin, points_end));
  // This is the optimization that Potree did: As long as the node can still accomodate points, we
  // defer the spacing check and just add all the points

  if (num_points_to_process <= max_points_per_node) {
    return points_end;
  }

  const auto spacing_at_this_node = spacing_at_root / std::pow(2, node_level + 1);
  // candidate_level_in_octree is the first level in the octree at which the nodes have a smaller
  // side length than the desired spacing
  const auto candidate_level_in_octree =
    std::max(-1,
             (int)std::round(std::log2f(octree_bounds.size.x / spacing_at_this_node)) -
               1); // the root node (whole octree) is level '-1', so level 0 has a sidelength of
                   // half the max octree, hence we have to subtract one here
  auto partition_point = points_begin;

  const auto stable_partition_at_level = [&](int level) {
    std::unordered_set<uint64_t> taken_indices;
    partition_point =
      std::stable_partition(points_begin, points_end, [&](const auto& indexed_point) {
        if (taken_indices.size() >= max_points_per_node)
          return false;

        const auto significant_bits = indexed_point.octree_node_index.truncate_to_level(level);
        if (taken_indices.find(significant_bits.get()) != taken_indices.end())
          return false;

        taken_indices.insert(significant_bits.get());
        return true;
      });
  };

  // Dirty, but we need a special function to partition at root (level -1) if this situation
  // ever occurs. We just take the first point and are done with it
  const auto partition_at_root = [&]() {
    if (points_begin == points_end)
      return;

    ++partition_point;
  };

  if (candidate_level_in_octree == -1) {
    partition_at_root();
  } else {
    // partition_at_level(candidate_level_in_octree);
    stable_partition_at_level(candidate_level_in_octree);
  }

  return partition_point;
}

/**
 * Given a range of indexed points, partitions the range into 8 ranges where each sub-range contains
 * only points that fall into the specific octant at 'level_to_partition_at'. This assumes that all
 * points in the input range are sorted in ascending order by their OctreeNodeKey and that the
 * parent node (at 'level_to_partition_at - 1') is equal for all points.
 *
 * 'Iter' has to dereference to IndexedPoint<N> for arbitrary N
 */
template<typename Iter>
std::array<std::pair<Iter, Iter>, 8>
partition_points_into_child_octants(Iter begin, Iter end, uint32_t level_to_partition_at)
{
  std::array<std::pair<Iter, Iter>, 8> partitions;
  auto current_begin = begin;

  for (uint8_t octant = 0; octant < 8; ++octant) {
    // End iterator is at the first element whose octant index on the current level is larger than
    // octant
    const auto current_end =
      std::find_if(current_begin, end, [octant, level_to_partition_at](const auto& indexed_point) {
        return indexed_point.octree_node_index.get_octant_at_level(level_to_partition_at) > octant;
      });

    partitions[octant] = std::make_pair(current_begin, current_end);
    current_begin = current_end;
  }

  return partitions;
}

/*
TODO filter_points_for_octree should work like an STL algorithm. It should partition the range of
potential points into a range of selected points followed by a range of not selected points.

Maybe we can even operate on std::pair<PointReference, OctreeNodeKey<>> so that we can easily swap
around elements in the range

I think that by doing so, we can get rid of that nasty 'invalid OctreeNodeKey' thing.
Also, since each node is processed in isolation from other nodes (at the same level), manipulating
the order of elements in the range for that particular node doesn't affect the other nodes

Futher question: How do we process nodes of different levels in parallel? Child nodes depend on
their parent nodes
*/