#include "catch.hpp"

#include "octree/OctreeNodeKey.h"

TEST_CASE("Default constructed key is zero", "[OctreeNodeKey]")
{
  constexpr uint32_t Levels = 20;
  using Key = OctreeNodeKey<Levels>;

  Key k;
  REQUIRE(k.get() == 0);
  REQUIRE(std::is_same_v<Key::Store_t, uint64_t>);
}

TEST_CASE("Value constructor works", "[OctreeNodeKey]")
{
  constexpr uint32_t Levels = 20;
  using Key = OctreeNodeKey<Levels>;

  const size_t expected = 12345;
  Key k{ expected };
  REQUIRE(k.get() == expected);
}

TEST_CASE("Value constructor discards bits outside of range", "[OctreeNodeKey]")
{
  constexpr uint32_t Levels = 2;
  using Key = OctreeNodeKey<Levels>;

  const uint8_t too_many_bits = 0b11111111; // 8 bits, but Key can only store 6 bits!
  const uint8_t expected = (too_many_bits & 0b111111);

  Key k{ too_many_bits };
  REQUIRE(k.get() == expected);
}

TEST_CASE("Level constructor works correctly", "[OctreeNodeKey]")
{
  constexpr uint32_t Levels = 4;
  using Key = OctreeNodeKey<Levels>;

  std::array<uint8_t, Levels> expected_levels = { 7, 6, 0, 3 };
  const uint16_t expected_value =
    static_cast<uint16_t>((expected_levels[0] << 9) | (expected_levels[1] << 6) |
                          (expected_levels[2] << 3) | (expected_levels[3]));

  Key k{ expected_levels };
  REQUIRE(k.get() == expected_value);
  for (uint32_t level = 0; level < Levels; ++level) {
    REQUIRE(k.get_octant_at_level(level) == expected_levels[level]);
  }
}

TEST_CASE("truncate_to_level works correctly", "[OctreeNodeKey]")
{
  constexpr uint32_t Levels = 4;
  using Key = OctreeNodeKey<Levels>;

  std::array<uint8_t, Levels> expected_levels = { 7, 6, 0, 3 };
  Key k{ expected_levels };

  auto l0_key = k.truncate_to_level(0);
  REQUIRE(l0_key.get() == expected_levels[0]);

  auto l1_key = k.truncate_to_level(1);
  REQUIRE(l1_key.get() == (expected_levels[0] << 3 | expected_levels[1]));
}

TEST_CASE("to_string on empty key returns empty string", "[OctreeNodeKey]")
{
  OctreeNodeKey64 empty_key;
  const auto str = to_string(empty_key, 0);
  REQUIRE(str == "");
}

TEST_CASE("to_string returns correct octants", "[OctreeNodeKey]")
{
  OctreeNodeKey64 key;
  key.set_octant_at_level(0, uint8_t(0));
  key.set_octant_at_level(1, uint8_t(1));
  key.set_octant_at_level(2, uint8_t(2));
  key.set_octant_at_level(3, uint8_t(3));
  key.set_octant_at_level(4, uint8_t(4));
  key.set_octant_at_level(5, uint8_t(5));
  key.set_octant_at_level(6, uint8_t(6));
  key.set_octant_at_level(7, uint8_t(7));

  const auto str = to_string(key, 8);
  REQUIRE(str == "01234567");
}

TEST_CASE("from_string from empty string returns empty key", "[OctreeNodeKey]")
{
  std::string str = "";
  const auto key = from_string<21>(str);
  REQUIRE(key.get() == 0ull);
}

TEST_CASE("from_string parses octants correctly", "[OctreeNodeKey]")
{
  std::string str = "01234567";
  const auto key = from_string<21>(str);
  REQUIRE(key.get_octant_at_level(0) == uint8_t(0));
  REQUIRE(key.get_octant_at_level(1) == uint8_t(1));
  REQUIRE(key.get_octant_at_level(2) == uint8_t(2));
  REQUIRE(key.get_octant_at_level(3) == uint8_t(3));
  REQUIRE(key.get_octant_at_level(4) == uint8_t(4));
  REQUIRE(key.get_octant_at_level(5) == uint8_t(5));
  REQUIRE(key.get_octant_at_level(6) == uint8_t(6));
  REQUIRE(key.get_octant_at_level(7) == uint8_t(7));
}

TEST_CASE("from_string ignores first 'r' character", "[OctreeNodeKey]")
{
  std::string str = "r01234567";
  const auto key = from_string<21>(str);
  REQUIRE(key.get_octant_at_level(0) == uint8_t(0));
  REQUIRE(key.get_octant_at_level(1) == uint8_t(1));
  REQUIRE(key.get_octant_at_level(2) == uint8_t(2));
  REQUIRE(key.get_octant_at_level(3) == uint8_t(3));
  REQUIRE(key.get_octant_at_level(4) == uint8_t(4));
  REQUIRE(key.get_octant_at_level(5) == uint8_t(5));
  REQUIRE(key.get_octant_at_level(6) == uint8_t(6));
  REQUIRE(key.get_octant_at_level(7) == uint8_t(7));
}