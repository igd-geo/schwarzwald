#include <catch2/catch_all.hpp>

#include "datastructures/MortonIndex.h"

TEST_CASE("Default constructed key is zero", "[MortonIndex]")
{
  constexpr uint32_t Levels = 20;
  using Key = MortonIndex<Levels>;

  Key k;
  REQUIRE(k.get() == 0);
  REQUIRE(std::is_same_v<Key::Store_t, uint64_t>);
}

TEST_CASE("Value constructor works", "[MortonIndex]")
{
  constexpr uint32_t Levels = 20;
  using Key = MortonIndex<Levels>;

  const size_t expected = 12345;
  Key k{ expected };
  REQUIRE(k.get() == expected);
}

TEST_CASE("Value constructor discards bits outside of range", "[MortonIndex]")
{
  constexpr uint32_t Levels = 2;
  using Key = MortonIndex<Levels>;

  const uint8_t too_many_bits = 0b11111111; // 8 bits, but Key can only store 6 bits!
  const uint8_t expected = (too_many_bits & 0b111111);

  Key k{ too_many_bits };
  REQUIRE(k.get() == expected);
}

TEST_CASE("Level constructor works correctly", "[MortonIndex]")
{
  constexpr uint32_t Levels = 4;
  using Key = MortonIndex<Levels>;

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

TEST_CASE("Level constructor with full number of levels works", "[MortonIndex]")
{
  constexpr uint32_t Levels = 21;
  using Key = MortonIndex<Levels>;
  std::array<uint8_t, Levels> expected_octants{ 5, 3, 7, 4, 0, 1, 6, 4, 3, 5, 3,
                                                6, 7, 3, 2, 1, 4, 0, 2, 5, 6 };
  Key k{ expected_octants };
  for (uint32_t level = 0; level < Levels; ++level) {
    REQUIRE(k.get_octant_at_level(level) == expected_octants[level]);
  }
}

TEST_CASE("truncate_to_level works correctly", "[MortonIndex]")
{
  constexpr uint32_t Levels = 4;
  using Key = MortonIndex<Levels>;

  std::array<uint8_t, Levels> expected_levels = { 7, 6, 0, 3 };
  Key k{ expected_levels };

  auto l0_key = k.truncate_to_level(0);
  REQUIRE(l0_key.get() == expected_levels[0]);

  auto l1_key = k.truncate_to_level(1);
  REQUIRE(l1_key.get() == (expected_levels[0] << 3 | expected_levels[1]));
}

TEST_CASE("to_string on empty key returns empty string", "[MortonIndex]")
{
  MortonIndex64 empty_key;
  const auto str = to_string(empty_key, 0);
  REQUIRE(str == "");
}

TEST_CASE("to_string returns correct octants", "[MortonIndex]")
{
  MortonIndex64 key;
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

  const auto truncated_str = to_string(key, 4);
  REQUIRE(truncated_str == "0123");
}

TEST_CASE("from_string from empty string returns empty key", "[MortonIndex]")
{
  std::string str = "";
  const auto key = from_string<21>(str);
  REQUIRE(key.get() == 0ull);
}

TEST_CASE("from_string parses octants correctly", "[MortonIndex]")
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

TEST_CASE("from_string ignores first 'r' character", "[MortonIndex]")
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