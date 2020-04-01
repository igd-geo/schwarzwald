#include "catch.hpp"

#include "types/Units.h"

TEST_CASE("Test small metric units") {
  // Zero is a special case, it has no precision. Zero is always zero
  REQUIRE(unit::format_with_metric_prefix(0) == "0");

  REQUIRE(unit::format_with_metric_prefix(1, 0) == "1");
  REQUIRE(unit::format_with_metric_prefix(0.1, 0) ==
          "100m"); /*TODO Not sure if this is really the expected behaviour.
                      Both '100m' and '0.1' seem valid*/
  REQUIRE(unit::format_with_metric_prefix(0.001, 0) == "1m");
  REQUIRE(unit::format_with_metric_prefix(1e-4, 0) == "100u");
  REQUIRE(unit::format_with_metric_prefix(1e-6, 0) == "1u");
  REQUIRE(unit::format_with_metric_prefix(1e-8, 0) == "10n");
  REQUIRE(unit::format_with_metric_prefix(1e-9, 0) == "1n");
  REQUIRE(unit::format_with_metric_prefix(1e-12, 0) == "1p");
  REQUIRE(unit::format_with_metric_prefix(1e-15, 0) == "1f");
  REQUIRE(unit::format_with_metric_prefix(1e-18, 0) == "1a");
  REQUIRE(unit::format_with_metric_prefix(1e-21, 0) == "1z");
  REQUIRE(unit::format_with_metric_prefix(1e-24, 0) == "1y");
  // Smaller than the smallest SI prefix and we start using decimals
  REQUIRE(unit::format_with_metric_prefix(1e-25, 0) == "0y");
  REQUIRE(unit::format_with_metric_prefix(1e-25, 1) == "0.1y");
}

TEST_CASE("Test large metric units") {
  REQUIRE(unit::format_with_metric_prefix(1, 0) == "1");
  REQUIRE(unit::format_with_metric_prefix(10, 0) == "10");
  REQUIRE(unit::format_with_metric_prefix(1e3, 0) == "1k");
  REQUIRE(unit::format_with_metric_prefix(1e4, 0) == "10k");
  REQUIRE(unit::format_with_metric_prefix(1e6, 0) == "1M");
  REQUIRE(unit::format_with_metric_prefix(1e8, 0) == "100M");
  REQUIRE(unit::format_with_metric_prefix(1e9, 0) == "1G");
  REQUIRE(unit::format_with_metric_prefix(1e12, 0) == "1T");
  REQUIRE(unit::format_with_metric_prefix(1e15, 0) == "1P");
  REQUIRE(unit::format_with_metric_prefix(1e18, 0) == "1E");
  REQUIRE(unit::format_with_metric_prefix(1e21, 0) == "1Z");
  REQUIRE(unit::format_with_metric_prefix(1e24, 0) == "1Y");
}

TEST_CASE("Test large binary units") {
  REQUIRE(unit::format_with_binary_prefix(0, 0) == "0");
  REQUIRE(unit::format_with_binary_prefix(1, 0) == "1");
  REQUIRE(unit::format_with_binary_prefix(10, 0) == "10");
  REQUIRE(unit::format_with_binary_prefix(1ull << 10, 0) == "1Ki");
  REQUIRE(unit::format_with_binary_prefix(1ull << 12, 0) == "4Ki");
  REQUIRE(unit::format_with_binary_prefix(1ull << 20, 0) == "1Mi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 24, 0) == "16Mi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 30, 0) == "1Gi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 40, 0) == "1Ti");
  REQUIRE(unit::format_with_binary_prefix(1ull << 50, 0) == "1Pi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 60, 0) == "1Ei");
  REQUIRE(unit::format_with_binary_prefix(1.180591621e21, 0) == "1Zi");
  REQUIRE(unit::format_with_binary_prefix(1.20892582e24, 0) == "1Yi");
}

TEST_CASE("Test precision") {
  REQUIRE(unit::format_with_metric_prefix(1.01) == "1.010");
  REQUIRE(unit::format_with_metric_prefix(1.01, 1) == "1.0");
  REQUIRE(unit::format_with_metric_prefix(1.123, 2) == "1.12");
  REQUIRE(unit::format_with_metric_prefix(10.123) == "10.123");
  REQUIRE(unit::format_with_metric_prefix(10.123, 2) == "10.12");
  REQUIRE(unit::format_with_metric_prefix(10.123, 1) == "10.1");
  REQUIRE(unit::format_with_metric_prefix(10.123, 0) == "10");
}