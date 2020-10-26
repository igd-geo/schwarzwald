
#include "pointcloud/PointAttributes.h"

#include <cstring>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

namespace bpo = boost::program_options;

bool
has_attribute(PointAttributes const& attributes, PointAttribute attribute_to_find)
{
  return attributes.find(attribute_to_find) != std::end(attributes);
}

bool
attributes_are_subset(PointAttributes const& maybe_subset, PointAttributes const& maybe_superset)
{
  return std::all_of(
    std::begin(maybe_subset), std::end(maybe_subset), [&maybe_superset](PointAttribute attribute) {
      return maybe_superset.find(attribute) != std::end(maybe_superset);
    });
}

std::string
print_attributes(PointAttributes const& attributes)
{
  std::stringstream ss;
  ss << "[ ";
  for (auto attribute : attributes) {
    ss << util::to_string(attribute) << " ";
  }
  ss << "]";
  return ss.str();
}

tl::expected<PointAttributes, std::string>
point_attributes_from_strings(const std::vector<std::string>& attribute_names)
{
  PointAttributes point_attributes;
  for (const auto& attribute : attribute_names) {
    const auto parse_result = util::try_parse<PointAttribute>(attribute).map(
      [&point_attributes](PointAttribute attribute) { point_attributes.insert(attribute); });
    if (!parse_result.has_value()) {
      return tl::make_unexpected(parse_result.error());
    }
  }
  return { point_attributes };
}

void
validate(boost::any& v, const std::vector<std::string>& values, PointAttributes*, int)
{
  bpo::validators::check_first_occurrence(v);
  std::vector<std::string> tokens;
  boost::algorithm::split(
    tokens, bpo::validators::get_single_string(values), boost::is_any_of(" "));

  PointAttributes attributes;

  for (const auto& token : tokens) {
    util::try_parse<PointAttribute>(token)
      .map([&attributes](const PointAttribute& attribute) { attributes.insert(attribute); })
      .or_else([](const std::string& err) {
        throw bpo::validation_error{ bpo::validation_error::kind_t::invalid_option_value };
      });
  }

  v = boost::any(attributes);
}