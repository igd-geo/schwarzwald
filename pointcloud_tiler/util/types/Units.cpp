#include "types/Units.h"

#include <array>
#include <iomanip>
#include <sstream>

struct UnitPrefix
{
  constexpr UnitPrefix(const char* label, double multiplier)
    : label(label)
    , multiplier(multiplier)
  {}

  const char* label;
  double multiplier;
};

constexpr static std::array<UnitPrefix, 17> MetricPrefixes = {
  UnitPrefix{ "y", 1e-24 }, UnitPrefix{ "z", 1e-21 }, UnitPrefix{ "a", 1e-18 },
  UnitPrefix{ "f", 1e-15 }, UnitPrefix{ "p", 1e-12 }, UnitPrefix{ "n", 1e-9 },
  UnitPrefix{ "u", 1e-6 },  UnitPrefix{ "m", 1e-3 },  UnitPrefix{ "", 1 },
  UnitPrefix{ "k", 1e3 },   UnitPrefix{ "M", 1e6 },   UnitPrefix{ "G", 1e9 },
  UnitPrefix{ "T", 1e12 },  UnitPrefix{ "P", 1e15 },  UnitPrefix{ "E", 1e18 },
  UnitPrefix{ "Z", 1e21 },  UnitPrefix{ "Y", 1e24 }
};
constexpr static std::array<UnitPrefix, 9> BinaryPrefixes = { UnitPrefix{ "", 1 },
                                                              UnitPrefix{ "Ki", 1ull << 10 },
                                                              UnitPrefix{ "Mi", 1ull << 20 },
                                                              UnitPrefix{ "Gi", 1ull << 30 },
                                                              UnitPrefix{ "Ti", 1ull << 40 },
                                                              UnitPrefix{ "Pi", 1ull << 50 },
                                                              UnitPrefix{ "Ei", 1ull << 60 },
                                                              UnitPrefix{ "Zi", 1.180591621e21 },
                                                              UnitPrefix{ "Yi", 1.20892582e24 } };

template<size_t N>
static std::string
format_quantity(double quantity, uint32_t precision, const std::array<UnitPrefix, N>& prefixes)
{

  // Search for the next UnitPrefix that is larger than the quantity. Go back
  // one step and we have the last prefix that was smaller, which is the one we
  // want
  const auto prefix_iter = std::upper_bound(
    std::begin(prefixes), std::end(prefixes), quantity, [](double value, const UnitPrefix& prefix) {
      return value < prefix.multiplier;
    });

  const auto& unit_prefix = [&]() -> const UnitPrefix& {
    if (prefix_iter == std::begin(prefixes))
      return *prefix_iter;
    return *std::prev(prefix_iter);
  }();

  std::stringstream ss;
  ss.precision(precision);
  ss << std::fixed << (quantity / unit_prefix.multiplier) << unit_prefix.label;
  return ss.str();
}

std::string
unit::format_with_metric_prefix(double quantity, uint32_t precision)
{
  return format_quantity(quantity, precision, MetricPrefixes);
}

std::string
unit::format_with_binary_prefix(double quantity, uint32_t precision)
{
  return format_quantity(quantity, precision, BinaryPrefixes);
}