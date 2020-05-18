#pragma once

#include "MortonIndex.h"

#include <algorithms/Bitmanip.h>
#include <containers/Range.h>

#include <expected.hpp>

#include <functional>
#include <sstream>

#include <boost/functional/hash.hpp>

/**
 * Morton index of a node inside an octree. This is different from a Morton index for
 * points WITHIN the octree! Morton indices for points are only defined for a fixed
 * number of levels (usually the maximum depth of the desired octree). For the tree
 * nodes, there is no fixed level, each node will have its own level and as such its
 * own unique Morton index.
 *
 * Here is an example (in 2D) that illustrates this:
 *  _______________________
 * |           |     |     |
 * |    1      |__ __|__ __|
 * |           |     |     |
 * |__ __ __ __|__ __|__ __|
 * |     | 2   |__|__|__|__|
 * |__ __|__ __|__|__|4_|__|
 * |     |   3 |__|__|__|__|
 * |__ __|__ __|__|__|__|__|
 *
 * The numbers represent points sorted into a quadtree. To calculate the Morton indices
 * for these points, a maximum tree depth has to be defined. This depth then defines the
 * resolution of a 2D grid which is used to calculate the Morton indices:
 *  _______________________
 * |__|__|__|__|__|__|__|__|
 * |__|_1|__|__|__|__|__|__|
 * |__|__|__|__|__|__|__|__|
 * |__|__|__|__|__|__|__|__|
 * |__|__|_2|__|__|__|__|__|
 * |__|__|__|__|__|__|4_|__|
 * |__|__|__|3_|__|__|__|__|
 * |__|__|__|__|__|__|__|__|
 *
 * Here, the maximum depth is 3, which results in a grid of size (2^3)*(2^3)=8*8. Based on
 * this, the Morton indices for the points are as follows (starting from top left quadrant
 * in clockwise direction):
 *
 * 1: [0,0,2]
 * 2: [3,1,0]
 * 3: [3,2,1]
 * 4: [2,1,3]
 *
 * Each of these points is part of MULTIPLE quadtree cells, as each non-root cell is always
 * fully contained in a cell with lower level (lower meaining 'closer to root'). As an example,
 * point 1 is part of cell [0,0,2], which is fully contained in cell [0,0], which in turn is
 * fully contained in cell [0], which ultimately is part of the root cell [].
 *
 * [0,0,2], [0,0], [0] and [] are the Morton indices for these nodes. This is what this
 * structure (OctreeNodeIndex) is representing!
 *
 * Here is the technical reason why we need a SEPARATE structure apart from 'MortonIndex' for
 * this:
 *
 * There is a fundamental difference between a Morton index for a point (which always has a fixed
 * size) and a Morton index for a node (which has a varying size). Numerically, the first one
 * has a fixed number of bits and the second one a varying number of bits. Let's take the example
 * of point 1 again, which has a Morton index of [0,0,2]. We need exactly 6 bits to represent this
 * Morton index (9 bits in the 3D case):
 *
 *   0b00'00'10
 *
 * Imagine for a moment that we were to represent the Morton indices for nodes with the same 6 bits,
 * so for the node [0] this would be:
 *
 *   0b00'00'00
 *
 * Clearly, [0,0,2] is part of [0], however purely from the numerical representation, there is no
 * way of inferring this. 0b00'00'00 could represent the node [0], or [0,0], or [0,0,0], which
 * depends fully on context. Instead, what we need is an additional piece of information telling
 * us how many levels this Morton index represents. In the case of node [0], we have one level and
 * as such need exactly two bits to represent this Morton index:
 *
 *   0b00
 *
 * In order to determine whether [0,0,2] is contained in [0], we have to convert [0,0,2] to the same
 * number of levels as [0]. This is easy - we just truncate the bits:
 *
 *   0b00'00'02 ---> 0b00
 *
 * Now we can compare the two bitwise representations, find that they are equal and thus know that
 * [0,0,2] is contained in [0]. Note that the same conversion DOES NOT work in the other direction
 * (going from a Morton index with lower level to one with a higher level), as we do not have enough
 * information to fill in the missing levels. This is why there is a conversion from MortonIndex to
 * OctreeNodeIndex, but not the other way around!
 *
 * Lastly, the existence of the MortonIndex structure (which does not store the number of levels at
 * runtime) is justified from a performance standpoint, as MortonIndex takes less memory and has a
 * simpler comparison operation.
 */
template<
  /*
    This parameter is ONLY for determining the storage size for the index. This has nothing to do
    with the actual number of levels that OctreeNodeIndex represents, which is a runtime parameter
  */
  unsigned MaxLevels>
struct OctreeNodeIndex
{
  /**
   * Creates a default OctreeNodeIndex that represents a root node (level 0)
   */
  constexpr OctreeNodeIndex()
    : _index(0)
    , _levels(0)
  {}

  /**
   * Creates an OctreeNodeIndex from the given range of octants, starting from the root node (i.e.
   * the first entry in 'octants' is the child octant of the root node)
   */
  constexpr explicit OctreeNodeIndex(std::initializer_list<uint8_t> octants)
    : OctreeNodeIndex(std::begin(octants), std::end(octants))
  {}

  template<typename Iter>
  constexpr OctreeNodeIndex(Iter begin, Iter end)
  {
    _levels = static_cast<uint32_t>(std::distance(begin, end));
    _index = 0;
    for (; begin != end; ++begin) {
      _index <<= 3;
      _index |= *begin;
    }
  }

  /**
   * Creates an OctreeNodeIndex from the first 'levels' levels of the given static MortonIndex
   */
  template<unsigned OtherMaxLevels>
  constexpr explicit OctreeNodeIndex(MortonIndex<OtherMaxLevels> static_morton_index,
                                     uint32_t levels = OtherMaxLevels)
  {
    static_assert(OtherMaxLevels <= MaxLevels,
                  "Can't construct OctreeNodeIndex from a MortonIndex that stores more bits than "
                  "the OctreeNodeIndex is capable of storing!");
    assert(levels <= MaxLevels);
    const auto bit_shift = (MaxLevels - levels) * 3;
    _levels = levels;
    _index = static_morton_index.get() >> bit_shift;
  }

  /**
   * Copy constructor
   */
  constexpr OctreeNodeIndex(const OctreeNodeIndex& other) = default;

  /**
   * Copy assignment operator
   */
  OctreeNodeIndex& operator=(const OctreeNodeIndex& other)
  {
    _levels = other._levels;
    _index = other._index;
    return *this;
  }

  /**
   * Constructs an OctreeNodeIndexed from a raw index and levels. This does no checks for the
   * validity of the index and levels!
   */
  static OctreeNodeIndex unchecked_from_index_and_levels(UnsignedBits<MaxLevels * 3> index,
                                                         uint32_t levels)
  {
    return OctreeNodeIndex(index, levels);
  }

  // Comparisons

  friend constexpr bool operator==(const OctreeNodeIndex& l, const OctreeNodeIndex& r)
  {
    return l.index() == r.index() && l.levels() == r.levels();
  }
  friend constexpr bool operator!=(const OctreeNodeIndex& l, const OctreeNodeIndex& r)
  {
    return !(l == r);
  }
  friend constexpr bool operator<(const OctreeNodeIndex& l, const OctreeNodeIndex& r)
  {
    const auto lowest_level = std::min(l.levels(), r.levels());
    return l.parent_at_level(lowest_level).index() < r.parent_at_level(lowest_level).index();
  }
  friend constexpr bool operator<=(const OctreeNodeIndex& l, const OctreeNodeIndex& r)
  {
    const auto lowest_level = std::min(l.levels(), r.levels());
    return l.parent_at_level(lowest_level).index() <= r.parent_at_level(lowest_level).index();
  }
  friend constexpr bool operator>(const OctreeNodeIndex& l, const OctreeNodeIndex& r)
  {
    const auto lowest_level = std::min(l.levels(), r.levels());
    return l.parent_at_level(lowest_level).index() > r.parent_at_level(lowest_level).index();
  }
  friend constexpr bool operator>=(const OctreeNodeIndex& l, const OctreeNodeIndex& r)
  {
    const auto lowest_level = std::min(l.levels(), r.levels());
    return l.parent_at_level(lowest_level).index() >= r.parent_at_level(lowest_level).index();
  }

  // Accessors

  /**
   * Returns the number of levels that this OctreeNodeIndex represents. Zero levels corresponds
   * to the root node
   */
  constexpr uint32_t levels() const { return _levels; }
  /**
   * Returns the Morton index of this OctreeNodeIndex
   */
  constexpr auto index() const { return _index; }
  /**
   * Returns the octant at the given level. The level is NON-ZERO-INDEXED, as level zero is equal
   * to the root node. So for an OctreeNodeIndex that represents 5 levels, level 5 corresponds to
   * the level 5 levels below the root node. For example:
   *
   * Given the OctreeNodeIndex [2,7,3,5,1]:
   *
   * Octant : / 2 7 3 5 1
   * Index  : 0 1 2 3 4 5
   * Level  : 0 1 2 3 4 5
   *
   * octant_at_level(5) == 1
   * octant_at_level(4) == 5
   * octant_at_level(3) == 3
   *   etc.
   *
   * With the exception:
   *
   * octant_at_level(0) == 0, as there is only a single node (the root node) at level zero!
   */
  constexpr uint8_t octant_at_level(const uint32_t level) const
  {
    assert(level <= _levels);
    const auto shift = (_levels - level) * 3;
    return static_cast<uint8_t>((_index >> shift) & 0b111);
  }

  // Other methods

  /**
   * Returns the parent OctreeNodeIndex to this index. Calling this method is only allowed when
   * this index does not represent the root node!
   */
  constexpr OctreeNodeIndex parent() const
  {
    if (!_levels)
      throw std::runtime_error{
        "Calling parent() is undefined on OctreeNodeIndex with levels() == 0!"
      };
    return OctreeNodeIndex(static_cast<UnsignedBits<MaxLevels * 3>>(_index >> 3), _levels - 1);
  }

  /**
   * Returns the OctreeNodeIndex for the parent at the given 'level' to this node. 'level' must be
   * <= levels()
   */
  constexpr OctreeNodeIndex parent_at_level(const uint32_t level) const
  {
    if (level > _levels)
      throw std::runtime_error{ "level parameter must be <= levels() in call to parent_at_level!" };
    const auto shift = (_levels - level) * 3;
    return OctreeNodeIndex(static_cast<UnsignedBits<MaxLevels * 3>>(_index >> shift), level);
  }

  /**
   * Tries to parse the given string into an OctreeNodeIndex. Returns the parsed index on success
   * and the reason for failure otherwise.
   */
  static tl::expected<OctreeNodeIndex, std::string> from_string(std::string_view str)
  {
    if (str.size() > MaxLevels) {
      return tl::make_unexpected("String size exceeds MaxLevels");
    }

    std::vector<uint8_t> octants;
    octants.reserve(str.size());
    for (auto c : str) {
      auto octant = static_cast<uint8_t>(c - '0');
      if (octant > 7) {
        return tl::make_unexpected(
          "Unexpected character in string. String must contain only ['0';'7']");
      }
      octants.push_back(octant);
    }

    return { OctreeNodeIndex{ std::begin(octants), std::end(octants) } };
  }

  /**
   * Converts the given OctreeNodeIndex into a string
   */
  static std::string to_string(const OctreeNodeIndex& index)
  {
    std::stringstream ss;
    for (uint32_t level = 1; level <= index._levels; ++level) {
      const auto cur_char =
        static_cast<char>('0' + static_cast<char>(index.octant_at_level(level)));
      ss << cur_char;
    }
    return ss.str();
  }

private:
  /**
   * Creates an OctreeNodeIndex with the given index and levels. This constructor does no validity
   * checks and no bit-shifting, it expected that 'index' and 'levels' match! Because this is easy
   * to get wrong, this constructor is private and instead the 'unchecked_from_index_and_levels'
   * method is exposed to make the intent clear.
   */
  constexpr OctreeNodeIndex(UnsignedBits<MaxLevels * 3> index, uint32_t levels)
    : _index(index)
    , _levels(levels)
  {
    assert(levels <= MaxLevels);
  }

  UnsignedBits<MaxLevels * 3> _index;
  uint32_t _levels;
};

namespace std {
template<unsigned int MaxBits>
struct hash<::OctreeNodeIndex<MaxBits>>
{
  size_t operator()(const ::OctreeNodeIndex<MaxBits>& obj) const
  {
    size_t hash = 17;
    boost::hash_combine(hash, obj.levels());
    boost::hash_combine(hash, obj.index());
    return hash;
  }
};
}

/**
 * OctreeNodeIndex that can hold 64 bits and, as such, 21 levels
 */
using OctreeNodeIndex64 = OctreeNodeIndex<21>;
/**
 * OctreeNodeIndex that can hold 32 bits and, as such, 10 levels
 */
using OctreeNodeIndex32 = OctreeNodeIndex<10>;