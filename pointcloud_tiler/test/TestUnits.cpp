#include "catch.hpp"

#include "types/Units.h"

TEST_CASE("Test small metric units")
{
  REQUIRE(unit::format_with_metric_prefix(1) == "1");
  REQUIRE(unit::format_with_metric_prefix(0.1) ==
          "100m"); /*TODO Not sure if this is really the expected behaviour.
                      Both '100m' and '0.1' seem valid*/
  REQUIRE(unit::format_with_metric_prefix(0.001) == "1m");
  REQUIRE(unit::format_with_metric_prefix(1e-4) == "100u");
  REQUIRE(unit::format_with_metric_prefix(1e-6) == "1u");
  REQUIRE(unit::format_with_metric_prefix(1e-8) == "10n");
  REQUIRE(unit::format_with_metric_prefix(1e-9) == "1n");
  REQUIRE(unit::format_with_metric_prefix(1e-12) == "1p");
  REQUIRE(unit::format_with_metric_prefix(1e-15) == "1f");
  REQUIRE(unit::format_with_metric_prefix(1e-18) == "1a");
  REQUIRE(unit::format_with_metric_prefix(1e-21) == "1z");
  REQUIRE(unit::format_with_metric_prefix(1e-24) == "1y");
  // Smaller than the smallest SI prefix and we start using decimals
  REQUIRE(unit::format_with_metric_prefix(1e-25) == "0.1y");
}

TEST_CASE("Test large metric units")
{
  REQUIRE(unit::format_with_metric_prefix(1) == "1");
  REQUIRE(unit::format_with_metric_prefix(10) == "10");
  REQUIRE(unit::format_with_metric_prefix(1e3) == "1k");
  REQUIRE(unit::format_with_metric_prefix(1e4) == "10k");
  REQUIRE(unit::format_with_metric_prefix(1e6) == "1M");
  REQUIRE(unit::format_with_metric_prefix(1e8) == "100M");
  REQUIRE(unit::format_with_metric_prefix(1e9) == "1G");
  REQUIRE(unit::format_with_metric_prefix(1e12) == "1T");
  REQUIRE(unit::format_with_metric_prefix(1e15) == "1P");
  REQUIRE(unit::format_with_metric_prefix(1e18) == "1E");
  REQUIRE(unit::format_with_metric_prefix(1e21) == "1Z");
  REQUIRE(unit::format_with_metric_prefix(1e24) == "1Y");
}

TEST_CASE("Test large binary units")
{
  REQUIRE(unit::format_with_binary_prefix(1) == "1");
  REQUIRE(unit::format_with_binary_prefix(10) == "10");
  REQUIRE(unit::format_with_binary_prefix(1ull << 10) == "1Ki");
  REQUIRE(unit::format_with_binary_prefix(1ull << 12) == "4Ki");
  REQUIRE(unit::format_with_binary_prefix(1ull << 20) == "1Mi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 24) == "16Mi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 30) == "1Gi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 40) == "1Ti");
  REQUIRE(unit::format_with_binary_prefix(1ull << 50) == "1Pi");
  REQUIRE(unit::format_with_binary_prefix(1ull << 60) == "1Ei");
  REQUIRE(unit::format_with_binary_prefix(1.180591621e21) == "1Zi");
  REQUIRE(unit::format_with_binary_prefix(1.20892582e24) == "1Yi");
}

TEST_CASE("Test precision")
{
  REQUIRE(unit::format_with_metric_prefix(1.01) == "1.010");
  REQUIRE(unit::format_with_metric_prefix(1.01, 1) == "1.0");
  REQUIRE(unit::format_with_metric_prefix(1.123, 2) == "1.12");
  REQUIRE(unit::format_with_metric_prefix(10.123) == "10.123");
  REQUIRE(unit::format_with_metric_prefix(10.123, 2) == "10.12");
  REQUIRE(unit::format_with_metric_prefix(10.123, 1) == "10.1");
  REQUIRE(unit::format_with_metric_prefix(10.123, 0) == "10");
}