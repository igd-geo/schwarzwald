#include "catch.hpp"

#include "util/Algorithm.h"

#include <algorithm>
#include <random>
#include <vector>

std::vector<int>
generate_random_numbers(size_t count, int min, int max)
{
  std::mt19937 mt;
  mt.seed((int)time(nullptr));
  std::uniform_int_distribution<int> dist{ min, max };

  std::vector<int> numbers;
  numbers.reserve(count);

  std::generate_n(std::back_inserter(numbers), count, [&]() { return dist(mt); });

  return numbers;
}

TEST_CASE("stable_partition_with_jumps is stable on even count input",
          "[stable_partition_with_jumps]")
{
  constexpr size_t Count = 1024;
  auto numbers = generate_random_numbers(Count, 0, 999);

  std::sort(std::begin(numbers), std::end(numbers));

  const auto predicate = [](int number) { return (number % 7) == 0; };
  const auto matches_count = std::count_if(std::begin(numbers), std::end(numbers), predicate);

  const auto pivot = stable_partition_with_jumps(
    std::begin(numbers), std::end(numbers), [&](auto current, auto end) {
      if (!predicate(*current)) {
        const auto match = std::find_if(current + 1, end, predicate);
        if (match == end)
          return std::make_pair(end, end);
        return std::make_pair(match, match + 1);
      }
      return std::make_pair(current, current + 1);
    });

  const auto selected_count = std::distance(std::begin(numbers), pivot);
  REQUIRE(selected_count == matches_count);

  REQUIRE(std::is_sorted(std::begin(numbers), pivot));
  REQUIRE(std::is_sorted(pivot, std::end(numbers)));
}

TEST_CASE("stable_partition_with_jumps is stable on odd count input",
          "[stable_partition_with_jumps]")
{
  constexpr size_t Count = 1025;
  auto numbers = generate_random_numbers(Count, 0, 999);

  std::sort(std::begin(numbers), std::end(numbers));

  const auto predicate = [](int number) { return (number % 7) == 0; };
  const auto matches_count = std::count_if(std::begin(numbers), std::end(numbers), predicate);

  const auto pivot = stable_partition_with_jumps(
    std::begin(numbers), std::end(numbers), [&](auto current, auto end) {
      if (!predicate(*current)) {
        const auto match = std::find_if(current + 1, end, predicate);
        if (match == end)
          return std::make_pair(end, end);
        return std::make_pair(match, match + 1);
      }
      return std::make_pair(current, current + 1);
    });

  const auto selected_count = std::distance(std::begin(numbers), pivot);
  REQUIRE(selected_count == matches_count);

  REQUIRE(std::is_sorted(std::begin(numbers), pivot));
  REQUIRE(std::is_sorted(pivot, std::end(numbers)));
}