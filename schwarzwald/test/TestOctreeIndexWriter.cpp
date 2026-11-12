#include <catch2/catch_all.hpp>

#include "tiling/OctreeIndexWriter.h"

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif
#include <random>

template<unsigned int MaxLevels, typename Rand>
static MortonIndex<MaxLevels>
generate_random_octree_index(Rand& rnd)
{
#ifdef _WIN32
  std::uniform_int_distribution<unsigned short> dist{ 0, 8 };
#else
  std::uniform_int_distribution<uint8_t> dist{ 0, 8 };
#endif
  std::array<uint8_t, MaxLevels> indices;
  for (uint32_t idx = 0; idx < MaxLevels; ++idx) {
    indices[idx] = dist(rnd);
  }
  return { indices };
}

TEST_CASE("Octree indices are preserved when writing to file and reading from it",
          "[octree_index_writer]")
{
#ifdef _WIN32
  static constexpr uint32_t Levels = 10;
  static constexpr uint32_t IdxCount = 32;
#else
  constexpr uint32_t Levels = 10;
  constexpr uint32_t IdxCount = 32;
#endif
  const std::string file_path = "__test_octree_idx_write.idx";
  using OctreeIdx_t = MortonIndex<Levels>;

  std::default_random_engine rnd;

  std::vector<OctreeIdx_t> expected_indices;
  expected_indices.reserve(IdxCount);
  std::generate_n(std::back_inserter(expected_indices), IdxCount, [&]() {
    return generate_random_octree_index<Levels>(rnd);
  });

  write_octree_indices_to_file(file_path, gsl::make_span(expected_indices));

  const auto actual_indices = read_octree_indices_from_file<Levels>(file_path);

  #ifdef _WIN32
    std::filesystem::remove(file_path);
  #else
    std::experimental::filesystem::remove(file_path);
  #endif

  REQUIRE(actual_indices.size() == expected_indices.size());
  for (size_t idx = 0; idx < actual_indices.size(); ++idx) {
    REQUIRE(actual_indices[idx].get() == expected_indices[idx].get());
  }
}