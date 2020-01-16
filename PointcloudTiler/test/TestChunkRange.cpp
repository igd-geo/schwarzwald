#include "catch.hpp"

#include "util/Algorithm.h"

#include <vector>

TEST_CASE("Chunk range generates even chunks when chunk count is a divider of range size",
          "[split_range_into_chunks]")
{
  constexpr size_t ChunkSize = 16;
  constexpr size_t NumChunks = 4;
  constexpr size_t RangeSize = ChunkSize * NumChunks;

  using Iter_t = std::vector<int>::iterator;

  std::vector<int> range;
  range.resize(RangeSize);

  const auto range_begin = range.begin();
  std::vector<std::pair<Iter_t, Iter_t>> expected_chunks = {
    std::make_pair(range_begin, range_begin + ChunkSize),
    std::make_pair(range_begin + ChunkSize, range_begin + 2 * ChunkSize),
    std::make_pair(range_begin + 2 * ChunkSize, range_begin + 3 * ChunkSize),
    std::make_pair(range_begin + 3 * ChunkSize, range.end())
  };

  const auto actual_chunks = split_range_into_chunks(NumChunks, range.begin(), range.end());

  REQUIRE(actual_chunks == expected_chunks);
}

TEST_CASE("Chunk range covers full range if chunk count is not a divier of range size",
          "[split_range_into_chunks]")
{
  constexpr size_t NumChunks = 4;
  constexpr size_t RangeSize = 17;

  using Iter_t = std::vector<int>::iterator;

  std::vector<int> range;
  range.resize(RangeSize);

  const auto range_begin = range.begin();
  std::vector<std::pair<Iter_t, Iter_t>> expected_chunks = {
    std::make_pair(range_begin, range_begin + 4),
    std::make_pair(range_begin + 4, range_begin + 8),
    std::make_pair(range_begin + 8, range_begin + 12),
    std::make_pair(range_begin + 12, range.end())
  };

  const auto actual_chunks = split_range_into_chunks(NumChunks, range.begin(), range.end());

  REQUIRE(actual_chunks == expected_chunks);
}