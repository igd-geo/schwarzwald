#pragma once

#include "datastructures/DynamicMortonIndex.h"
#include "datastructures/MortonIndex.h"
#include "datastructures/PointBuffer.h"
#include "datastructures/OctreeNodeIndex.h"
#include "math/AABB.h"
#include "tiling/Sampling.h"
#include "util/stuff.h"

#include <boost/format.hpp>
#include <unordered_set>
#include <vector>

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
 * What to do with outlier points (i.e. points that are read from the source
 * file but are not within the bounds of the source file)
 */
enum class OutlierPointsBehaviour
{
  /**
   * Clamp points to the bounding box
   */
  ClampToBounds,
  /**
   * Abort program when an outlier point is found
   */
  Abort
};

/**
 * Returns the bounding box of the given octant of the parent bounding box
 */
AABB
get_octant_bounds(uint8_t octant, const AABB& parent_bounds);

/**
 * Returns the bounding box of the octree node with the given name. Names must
 * follow the form 'r01234567'
 */
AABB
get_bounds_from_node_name(const std::string& node_name, const AABB& root_bounds);

/**
 * Returns the root bounds of the octree that the given node belongs to, based on the
 * bounds of the node
 */
AABB
get_root_bounds_from_node(OctreeNodeIndex64 node_index, const AABB& node_bounds);

/**
 * Returns the index of the octant of 'bounds' that the given position lies in
 */
uint8_t
get_octant(const Vector3<double>& position, const AABB& bounds);

/**
 * Calculates the MortonIndex for a given position starting from 'node_bounds'
 * as root node
 */
template<unsigned int MaxLevels>
MortonIndex<MaxLevels>
calculate_morton_index(const Vector3<double>& position, const AABB& node_bounds)
{
  using DataType_t = typename MortonIndex<MaxLevels>::Store_t;
  // Normalize bounds and position to [0;2^MaxLevels-1]
  const auto normalized_scale = (std::pow(2, MaxLevels) / node_bounds.extent());
  const auto normalized_point =
    (position - node_bounds.min).multiply_component_wise(normalized_scale);
  // Ensure that points right on the edge of the bounds don't overflow
  const auto bits_x = std::min(static_cast<DataType_t>(normalized_point.x),
                               static_cast<DataType_t>((1 << MaxLevels) - 1));
  const auto bits_y = std::min(static_cast<DataType_t>(normalized_point.y),
                               static_cast<DataType_t>((1 << MaxLevels) - 1));
  const auto bits_z = std::min(static_cast<DataType_t>(normalized_point.z),
                               static_cast<DataType_t>((1 << MaxLevels) - 1));
  // Interleave the bits and we have the key
  const auto expanded_bits_x = expand_bits_by_3(bits_x);
  const auto expanded_bits_y = expand_bits_by_3(bits_y);
  const auto expanded_bits_z = expand_bits_by_3(bits_z);
  const auto key_val = (expanded_bits_z) | (expanded_bits_y << 1) | (expanded_bits_x << 2);
  return MortonIndex<MaxLevels>{ static_cast<DataType_t>(key_val) };
}

template<unsigned int MaxLevels>
MortonIndex<MaxLevels>
calculate_morton_index_naive(const Vector3<double>& position, const AABB& node_bounds)
{
  MortonIndex<MaxLevels> key;
  auto cur_bounds = node_bounds;
  for (uint32_t level = 0; level < MaxLevels; ++level) {
    auto cur_octant = get_octant(position, cur_bounds);
    key.set_octant_at_level(level, cur_octant);
    cur_bounds = get_octant_bounds(cur_octant, cur_bounds);
  }
  return key;
}

template<unsigned int MaxLevels>
AABB
get_bounds_from_morton_index(const MortonIndex<MaxLevels>& key,
                             const AABB& root_bounds,
                             uint32_t depth = MaxLevels)
{
  auto bounds = root_bounds;
  const auto max_level = std::min(depth, MaxLevels);
  for (uint32_t level = 0; level < max_level; ++level) {
    bounds = get_octant_bounds(key.get_octant_at_level(level), bounds);
  }
  return bounds;
}

AABB
get_bounds_from_morton_index(const DynamicMortonIndex& morton_index, const AABB& root_bounds);

/**
 * Calculates an index into the octree spanned by the given bounding box for
 * each of the given points. The index represents the octant at each level of
 * the octree that a point belongs to.
 */
template<unsigned int MaxLevels, typename KeysIter, typename PointsIter>
void
calculate_morton_indices_for_points(PointsIter points_begin, //*points_begin is Vector3<double>
                                    PointsIter points_end,
                                    KeysIter keys_begin,
                                    const AABB& octree_bounds)
{
  std::transform(points_begin, points_end, keys_begin, [&](const auto& position) {
    return calculate_morton_index<MaxLevels>(position, octree_bounds);
  });
}

template<unsigned int MaxLevels>
IndexedPoint<MaxLevels>
index_point(PointBuffer::PointReference point,
            const AABB& bounds,
            OutlierPointsBehaviour outlier_points_behaviour)
{
  const auto is_outlier = [](const Vector3<double>& position, const AABB& bounds) {
    return !bounds.isInside(position);
  };

  auto& position = point.position();
  if (is_outlier(position, bounds)) {

    if (outlier_points_behaviour == OutlierPointsBehaviour::Abort) {
      throw std::runtime_error{
        (boost::format("Found outlier point:\n\tPoint: %1%\n\tBounds: %2%\n") % position % bounds)
          .str()
      };
    }

    position.x = std::min(bounds.max.x, std::max(bounds.min.x, position.x));
    position.y = std::min(bounds.max.y, std::max(bounds.min.y, position.y));
    position.z = std::min(bounds.max.z, std::max(bounds.min.z, position.z));
  }

  return IndexedPoint<MaxLevels>{ point, calculate_morton_index<MaxLevels>(position, bounds) };
}

/**
 * Generates IndexedPoint objects for each point in the given PointBuffer and
 * stores them in the given output iterator.
 */
template<unsigned int MaxLevels, typename OutIter>
void
index_points(PointBuffer::PointIterator points_begin,
             PointBuffer::PointIterator points_end,
             OutIter indexed_points_begin,
             const AABB& bounds,
             OutlierPointsBehaviour outlier_points_behaviour)
{
  std::transform(points_begin,
                 points_end,
                 indexed_points_begin,
                 [&bounds, outlier_points_behaviour](PointBuffer::PointReference point_reference) {
                   return index_point<MaxLevels>(point_reference, bounds, outlier_points_behaviour);
                 });
}

/**
 * Partitions the given range of points into two ranges where the first range
 * [begin,center) contains all points that belong to the given octree node and
 * the second range [center, end) contains all points that don't belong to this
 * node. Returns 'center'. Partitioning is stable, i.e. it preserves the
 * relative order between elements within the two ranges.
 *
 * The octree root node has 'node_level' -1, the first children start at level 0
 */
template<typename Iter, unsigned int MaxLevels>
Iter
filter_points_for_octree_node(Iter points_begin, // *Iter is IndexedPoint<MaxLevels>
                              Iter points_end,
                              MortonIndex<MaxLevels> node_key,
                              int32_t node_level,
                              const AABB& root_bounds,
                              float spacing_at_root,
                              SamplingStrategy& sampling_strategy)
{
  assert(points_end >= points_begin);
  return sample_points(sampling_strategy,
                       points_begin,
                       points_end,
                       node_key,
                       node_level,
                       root_bounds,
                       spacing_at_root);
}

/**
 * Given a range of indexed points, partitions the range into 8 ranges where
 * each sub-range contains only points that fall into the specific octant at
 * 'level_to_partition_at'. This assumes that all points in the input range are
 * sorted in ascending order by their MortonIndex and that the parent node (at
 * 'level_to_partition_at - 1') is equal for all points.
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
    // End iterator is at the first element whose octant index on the current
    // level is larger than octant
    const auto current_end =
      std::find_if(current_begin, end, [octant, level_to_partition_at](const auto& indexed_point) {
        return indexed_point.morton_index.get_octant_at_level(level_to_partition_at) > octant;
      });

    partitions[octant] = std::make_pair(current_begin, current_end);
    current_begin = current_end;
  }

  return partitions;
}