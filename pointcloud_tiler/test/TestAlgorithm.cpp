#include "catch.hpp"

#include "algorithms/Algorithm.h"

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

SCENARIO("Algorithm - merge - single range", "[Algorithm]")
{
  using Range_t = util::Range<std::vector<int>::iterator>;
  GIVEN("A single range")
  {
    std::vector<int> numbers = { 1, 2, 3, 4 };
    std::vector<Range_t> ranges = { util::range(numbers) };

    WHEN("This range is merged")
    {
      std::vector<int> out;
      out.resize(4);
      merge_ranges(util::range(ranges), util::range(out), std::less<int>{});

      THEN("The order of elements is preserved") { REQUIRE(out == numbers); }
    }
  }

  GIVEN("A single empty range")
  {
    std::vector<int> numbers;
    std::vector<Range_t> ranges = { util::range(numbers) };

    WHEN("This range is merged")
    {
      std::vector<int> out;
      merge_ranges(util::range(ranges), util::range(out), std::less<int>{});

      THEN("The output range is empty") { REQUIRE(out == numbers); }
    }
  }
}

SCENARIO("Algorithm - merge - two ranges", "[Algorithm]")
{
  using Range_t = util::Range<std::vector<int>::iterator>;
  GIVEN("Two ranges")
  {
    std::vector<int> odds = { 1, 3, 7, 9 };
    std::vector<int> evens = { 2, 4, 6, 8 };

    std::vector<Range_t> ranges = { util::range(odds), util::range(evens) };

    WHEN("This range is merged")
    {
      std::vector<int> out;
      out.resize(8);
      merge_ranges(util::range(ranges), util::range(out), std::less<int>{});

      THEN("The order of elements is preserved")
      {
        std::vector<int> expected = { 1, 2, 3, 4, 6, 7, 8, 9 };
        REQUIRE(out == expected);
      }
    }
  }

  GIVEN("Two ranges of different sizes")
  {
    std::vector<int> odds = { 1, 3, 7, 9, 11, 13 };
    std::vector<int> evens = { 2, 4, 24 };

    std::vector<Range_t> ranges = { util::range(odds), util::range(evens) };

    WHEN("This range is merged")
    {
      std::vector<int> out;
      out.resize(9);
      merge_ranges(util::range(ranges), util::range(out), std::less<int>{});

      THEN("The order of elements is preserved")
      {
        std::vector<int> expected = { 1, 2, 3, 4, 7, 9, 11, 13, 24 };
        REQUIRE(out == expected);
      }
    }
  }

  GIVEN("Two ranges where one range is empty")
  {
    std::vector<int> odds = { 1, 3, 7, 9 };
    std::vector<int> empty;

    std::vector<Range_t> ranges = { util::range(odds), util::range(empty) };

    WHEN("This range is merged")
    {
      std::vector<int> out;
      out.resize(4);
      merge_ranges(util::range(ranges), util::range(out), std::less<int>{});

      THEN("The order of elements is preserved") { REQUIRE(out == odds); }
    }
  }
}

SCENARIO("Algorithm - merge - many ranges", "[Algorithm]")
{
  using Range_t = util::Range<std::vector<int>::iterator>;

  GIVEN("Many ranges of different sizes")
  {
    std::vector<int> v1 = { 1, 3, 7, 9, 11, 13 };
    std::vector<int> v2 = { 2, 4, 24 };
    std::vector<int> v3 = { 5, 6, 13, 16, 99 };
    std::vector<int> v4 = { 1, 3, 4, 5, 6, 8, 9, 44, 55, 66, 77, 88, 99 };

    std::vector<Range_t> ranges = {
      util::range(v1), util::range(v2), util::range(v3), util::range(v4)
    };

    WHEN("This range is merged")
    {
      std::vector<int> out;
      out.resize(v1.size() + v2.size() + v3.size() + v4.size());
      merge_ranges(util::range(ranges), util::range(out), std::less<int>{});

      THEN("The order of elements is preserved")
      {
        std::vector<int> expected = { 1, 1,  2,  3,  3,  4,  4,  5,  5,  6,  6,  7,  8, 9,
                                      9, 11, 13, 13, 16, 24, 44, 55, 66, 77, 88, 99, 99 };
        REQUIRE(out == expected);
      }
    }
  }
}