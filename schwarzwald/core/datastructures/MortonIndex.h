#pragma once

#include <array>
#include <cassert>
#include <sstream>
#include <stdint.h>
#include <type_traits>

namespace detail {
template<unsigned int MaxBits>
using KeyDataType_t = std::conditional_t<
  MaxBits <= 8,
  uint8_t,
  std::conditional_t<
    MaxBits <= 16,
    uint16_t,
    std::conditional_t<
      MaxBits <= 32,
      uint32_t,
      std::conditional_t<MaxBits <= 64, uint64_t, __uint128_t>>>>;

/**
 * Returns a bitmask with the first 'Bits' set bits, e.g. calling
 * 'all_set_bits<5>()' returns '0b11111' etc.
 */
template<unsigned int Bits>
constexpr auto
all_set_bits()
{
  using IntType_t = KeyDataType_t<Bits>;
  return static_cast<IntType_t>(1ull << Bits) - static_cast<IntType_t>(1);
}
} // namespace detail

// Naming convention for parsing and printing Morton indices
enum class MortonIndexNamingConvention
{
  // Concatenation of the octant indices, starting with the node that is closest
  // to the root (e.g. 0475263). The root node itself is ignored, as it has no
  // octant index. An empty Morton index thus corresponds to the empty string.
  Simple,
  // Like the 'Simple' convention, but the resulting string is prefixed with 'r'
  // for the root node. An empty Morton index thus corresponds to the string
  // 'r'. This is the naming convention that Potree uses.
  Potree,
  // A concatenation of the depth of the Morton index with the X, Y and Z
  // components of the Morton index. The spatial components correspond to the X,
  // Y and Z indices of the octree node within a regular grid with 2^depth
  // subdivisions. Strings then take the form of 'depth-X-Y-Z', e.g. 2-7-3-1.
  // This is the naming convention that Entwine uses.
  Entwine
};

/**
 * A key that stores the coordinates (indices) of a specific node in an octree
 * inside a bitmask. Each level in the octree is represented as an index from
 * [0;7] through three bits. The indices are packed from topmost level down in
 * big-endian order, i.e. the root level index is stored in the most significant
 * bits of the key while lower levels are stored in the lesser significant bits.
 * The lowest level is always stored in the 3 least significant bits.
 *
 * Octants in the octree are indexed with this pattern:
 *   y
 *   |-z
 *   |/
 *   O----x
 *
 *   3----7
 *  /|   /|
 * 2----6 |
 * | 1--|-5
 * |/   |/
 * 0----4
 *
 * An example: Consider a node which lies in the octants [1,4,3,7] (from top to
 * bottom). The key for this node requires at least 12 bits to represent. In
 * binary notation, the octant indices are encoded as [0b001, 0b100, 0b011,
 * 0b111] an packed together into the value 0b001'100'011'111
 */
template<unsigned int _MaxLevels>
struct MortonIndex
{
  static constexpr auto MaxLevels = _MaxLevels;

  static constexpr auto BitsRequired = MaxLevels * 3;
  static_assert(BitsRequired <= 128,
                "MaxLevels is too large, only 42 levels are supported!");
  using Store_t = detail::KeyDataType_t<BitsRequired>;

  static constexpr MortonIndex MAX = { ~Store_t{ 0 } };

  constexpr MortonIndex()
    : _store(0)
  {}
  constexpr MortonIndex(Store_t val)
    : _store(val & detail::all_set_bits<BitsRequired>())
  {}
  constexpr MortonIndex(const std::array<uint8_t, MaxLevels>& levels)
  {
    _store = init_from_levels(levels);
  }

  MortonIndex& operator=(Store_t val)
  {
    _store = val;
    return *this;
  }
  MortonIndex& operator=(const std::array<uint8_t, MaxLevels>& levels)
  {
    _store = init_from_levels(levels);
    return *this;
  }

  bool operator==(const MortonIndex<_MaxLevels>& other) const
  {
    return get() == other.get();
  }
  bool operator<(const MortonIndex<_MaxLevels>& other) const
  {
    return get() < other.get();
  }

  constexpr MortonIndex truncate_to_level(const uint32_t level) const
  {
    assert(level < MaxLevels);

    const auto shift = (MaxLevels - level - 1) * 3;
    return { static_cast<Store_t>(_store >> shift) };
  }

  constexpr Store_t get() const { return _store; }

  uint8_t get_octant_at_level(uint32_t level) const
  {
    assert(level < MaxLevels);
    const auto shift = (MaxLevels - level - 1) * 3;
    return static_cast<uint8_t>((_store >> shift) & 0b111);
  }

  void set_octant_at_level(uint32_t level, uint8_t octant)
  {
    assert(level < MaxLevels);
    const auto shift = (MaxLevels - level - 1) * 3;
    _store |= static_cast<Store_t>((octant & 0b111)) << shift;
  }

  std::string str() const
  {
    std::stringstream ss;
    for (uint32_t level = 0; level < MaxLevels; ++level) {
      ss << static_cast<uint32_t>(get_octant_at_level(level));
    }
    return ss.str();
  }

private:
  Store_t _store;

  constexpr Store_t init_from_levels(
    const std::array<uint8_t, MaxLevels>& levels)
  {
    Store_t key{ 0 };
    for (uint32_t level = 0; level < MaxLevels; ++level) {
      const auto shift = static_cast<Store_t>((MaxLevels - level - 1) * 3);
      key |= (static_cast<Store_t>(levels.at(level) & 0b111) << shift);
    }
    return key;
  }
};

/**
 * Converts a MortonIndex into a string representation of the node index. This
 * is a concatenation of all octree indices from top to bottom.
 *
 * An example: A key that represents the node [1,4,3,7] gets converted into the
 * string "1437"
 */
template<unsigned int MaxLevels>
std::string
to_string(const MortonIndex<MaxLevels>& key, uint32_t levels = MaxLevels)
{
  const auto threshold = std::min(MaxLevels, levels);
  std::stringstream ss;
  for (uint32_t level = 0; level < threshold; ++level) {
    ss << static_cast<uint32_t>(key.get_octant_at_level(level));
  }
  return ss.str();
}

/**
 * Takes strings of the form '03465273' and converts them into MortonIndex
 * objects. The strings may start with 'r', which is ignored.
 */
template<unsigned int MaxLevels>
MortonIndex<MaxLevels>
from_string(const std::string& str)
{
  MortonIndex<MaxLevels> key;
  const auto cleaned_str = (str[0] == 'r') ? str.substr(1) : str;
  const auto bound = std::min(static_cast<size_t>(MaxLevels), str.size());
  for (size_t idx = 0; idx < bound; ++idx) {
    const auto level = static_cast<uint8_t>(cleaned_str[idx] - '0');
    key.set_octant_at_level(static_cast<uint32_t>(idx), level);
  }
  return key;
}

namespace std {
template<unsigned int MaxLevels>
struct hash<MortonIndex<MaxLevels>>
{
  size_t operator()(const MortonIndex<MaxLevels>& idx) const
  {
    return idx.get();
  }
};
}

/**
 * A 64-bit MortonIndex that can represent 21 levels (21*3 == 63)
 */
using MortonIndex64 = MortonIndex<21>;

/**
 * The maximum number of levels representable by a 64-bit Morton index
 */
static constexpr unsigned int MortonIndex64Levels = 21;