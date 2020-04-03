#pragma once

#include "algorithms/Enums.h"
#include "algorithms/Pairs.h"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <expected.hpp>

enum class PointAttribute
{
  Position,
  RGB, // TODO This should maybe be named COLOR, but this would be a breaking
       // change for the command line arguments
  RGBFromIntensity,
  Intensity,
  Classification,
  Normal,
};

namespace util {
namespace {
static const std::unordered_set<std::pair<PointAttribute, std::string>, util::PairHash>
  POINT_ATTRIBUTE_STRING_MAPPING = {
    { PointAttribute::Position, "POSITION" },
    { PointAttribute::RGB, "RGB" },
    { PointAttribute::RGBFromIntensity, "RGB_FROM_INTENSITY" },
    { PointAttribute::Intensity, "INTENSITY" },
    { PointAttribute::Classification, "CLASSIFICATION" },
    { PointAttribute::Normal, "NORMAL" },
  };
}

template<>
inline tl::expected<PointAttribute, std::string>
try_parse(const std::string& token)
{
  const auto iter = std::find_if(std::begin(POINT_ATTRIBUTE_STRING_MAPPING),
                                 std::end(POINT_ATTRIBUTE_STRING_MAPPING),
                                 [&token](const auto& pair) { return pair.second == token; });
  if (iter == std::end(POINT_ATTRIBUTE_STRING_MAPPING)) {
    return tl::make_unexpected(
      (boost::format("Could not parse token \"%1%\" as PointAttribute") % token).str());
  }

  return { iter->first };
}

template<>
inline const std::string&
to_string(PointAttribute attribute)
{
  const auto iter = std::find_if(std::begin(POINT_ATTRIBUTE_STRING_MAPPING),
                                 std::end(POINT_ATTRIBUTE_STRING_MAPPING),
                                 [attribute](const auto& pair) { return pair.first == attribute; });
  if (iter == std::end(POINT_ATTRIBUTE_STRING_MAPPING)) {
    throw std::invalid_argument{
      (boost::format("Invalid PointAttribute %1%") % static_cast<int>(attribute)).str()
    };
  }

  return iter->second;
}
} // namespace util

using PointAttributes = std::unordered_set<PointAttribute>;

void
validate(boost::any& v, const std::vector<std::string>& values, PointAttributes*, int);

bool
has_attribute(PointAttributes const& attributes, PointAttribute attribute_to_find);
std::string
print_attributes(PointAttributes const& attributes);

tl::expected<PointAttributes, std::string>
point_attributes_from_strings(const std::vector<std::string>& attribute_names);