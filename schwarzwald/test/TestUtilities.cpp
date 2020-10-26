#include "catch.hpp"

#include "util/stuff.h"

TEST_CASE("concat with one argument returns argument", "[concat]") {
  std::string expected{"concat"};
  const auto actual = concat(expected);
  REQUIRE(actual == expected);
}

TEST_CASE("concat with two arguments concatinates correctly", "[concat]") {
  const auto actual = concat("first", "second");
  REQUIRE(actual == "firstsecond");
}

TEST_CASE("concat with multiple arguments concatinates correctly", "[concat]") {
  const auto actual = concat("..", "/some", "_test");
  REQUIRE(actual == "../some_test");
}

TEST_CASE("concat with mixed type arguments concatinates correctly",
          "[concat]") {
  const char *arg1 = "arg1";
  std::string arg2 = "arg2";
  const auto actual = concat(arg1, arg2);
  REQUIRE(actual == "arg1arg2");
}