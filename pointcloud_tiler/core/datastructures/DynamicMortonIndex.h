#pragma once

#include <functional>
#include <stdint.h>
#include <string>
#include <vector>

#include <boost/functional/hash.hpp>
#include <expected.hpp>

#include "MortonIndex.h"

// Naming convention for parsing and printing Morton indices
enum class MortonIndexNamingConvention
{
  // Concatenation of the octant indices, starting with the node that is closest to the root (e.g.
  // 0475263). The root node itself is ignored, as it has no octant index. An empty Morton index
  // thus corresponds to the empty string.
  Simple,
  // Like the 'Simple' convention, but the resulting string is prefixed with 'r' for the root node.
  // An empty Morton index thus corresponds to the string 'r'. This is the naming convention that
  // Potree uses.
  Potree,
  // A concatenation of the depth of the Morton index with the X, Y and Z components of the Morton
  // index. The spatial components correspond to the X, Y and Z indices of the octree node within a
  // regular grid with 2^depth subdivisions. Strings then take the form of 'depth-X-Y-Z', e.g.
  // 2-7-3-1. This is the naming convention that Entwine uses.
  Entwine
};

/**
 * A Morton index implementation that supports runtime-dynamic depth. Similar to the implementation
 * in MortonIndex.h, but it can support an arbitrary depth.
 *
 * Octant indexing order is equal to that of MortonIndex.h (XYZ little endian)
 */
struct DynamicMortonIndex
{
  using Octant_t = uint8_t;

  DynamicMortonIndex();
  explicit DynamicMortonIndex(std::vector<Octant_t> octants);
  /**
   * Creates a DynamicMortonIndex from a fixed-size MortonIndex. The template parameter 'MaxLevels'
   * defines the maximum storage size of the static MortonIndex, whereas the 'levels' argument
   * defines how many of the levels of the static MortonIndex to use for the conversion. It defaults
   * to 'MaxLevels' but can for example be used to create a DynamicMortonIndex with 5 levels from a
   * static MortonIndex with 10 levels.
   */
  template<unsigned int MaxLevels>
  DynamicMortonIndex(MortonIndex<MaxLevels> static_morton_index, unsigned int levels = MaxLevels);

  DynamicMortonIndex(const DynamicMortonIndex&) = default;
  DynamicMortonIndex(DynamicMortonIndex&&) = default;

  DynamicMortonIndex& operator=(const DynamicMortonIndex&) = default;
  DynamicMortonIndex& operator=(DynamicMortonIndex&&) = default;

  size_t depth() const;

  Octant_t operator[](size_t level) const;
  Octant_t& operator[](size_t level);

  bool operator==(const DynamicMortonIndex& other) const;

  std::vector<Octant_t>::const_iterator begin() const;
  std::vector<Octant_t>::const_iterator end() const;

  DynamicMortonIndex truncate_to_depth(size_t new_depth) const;

  DynamicMortonIndex child(Octant_t octant) const;
  DynamicMortonIndex parent() const;

  template<unsigned int MaxLevels>
  MortonIndex<MaxLevels> to_static_morton_index() const;

  static tl::expected<DynamicMortonIndex, std::string> parse_string(
    const std::string& str,
    MortonIndexNamingConvention naming_convention = MortonIndexNamingConvention::Simple);

private:
  std::vector<Octant_t> _octants;
};

std::string
to_string(const DynamicMortonIndex& morton_index,
          MortonIndexNamingConvention naming_convention = MortonIndexNamingConvention::Simple);

namespace std {
template<>
struct hash<::DynamicMortonIndex>
{
  size_t operator()(const ::DynamicMortonIndex& obj) const
  {
    size_t hash = 17;
    for (auto octant : obj) {
      boost::hash_combine(hash, octant);
    }
    return hash;
  }
};
}

template<unsigned int MaxLevels>
DynamicMortonIndex::DynamicMortonIndex(MortonIndex<MaxLevels> static_morton_index,
                                       unsigned int max_levels)
{
  assert(max_levels <= MaxLevels);
  _octants.resize(max_levels);
  for (unsigned int level = 0; level < max_levels; ++level) {
    _octants[level] = static_morton_index.get_octant_at_level(level);
  }
}

template<unsigned int MaxLevels>
MortonIndex<MaxLevels>
DynamicMortonIndex::to_static_morton_index() const
{
  const auto max_depth = std::min(MaxLevels, static_cast<unsigned int>(_octants.size()));
  MortonIndex<MaxLevels> static_morton_index = {};
  for (unsigned int depth = 0; depth < max_depth; ++depth) {
    static_morton_index.set_octant_at_level(depth, _octants[depth]);
  }
  return static_morton_index;
}