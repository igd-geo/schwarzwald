
#include "PointAttributes.hpp"

#include <cstring>
#include <sstream>

PointAttribute
PointAttribute::fromString(const std::string& name)
{
  if (name == "POSITION_CARTESIAN") {
    return attributes::POSITION_CARTESIAN;
  } else if (name == "COLOR_PACKED") {
    return attributes::COLOR_PACKED;
  } else if (name == "INTENSITY") {
    return attributes::INTENSITY;
  } else if (name == "CLASSIFICATION") {
    return attributes::CLASSIFICATION;
  } else if (name == "NORMAL_SPHEREMAPPED") {
    return attributes::NORMAL_SPHEREMAPPED;
  } else if (name == "NORMAL_OCT16") {
    return attributes::NORMAL_OCT16;
  } else if (name == "NORMAL") {
    return attributes::NORMAL;
  }

  throw std::runtime_error("Invalid PointAttribute name: '" + name + "'");
}

PointAttribute
PointAttribute::fromStringLiteral(const char* name)
{
  if (std::strcmp(name, "POSITION_CARTESIAN") == 0) {
    return attributes::POSITION_CARTESIAN;
  } else if (std::strcmp(name, "COLOR_PACKED") == 0) {
    return attributes::COLOR_PACKED;
  } else if (std::strcmp(name, "INTENSITY") == 0) {
    return attributes::INTENSITY;
  } else if (std::strcmp(name, "CLASSIFICATION") == 0) {
    return attributes::CLASSIFICATION;
  } else if (std::strcmp(name, "NORMAL_SPHEREMAPPED") == 0) {
    return attributes::NORMAL_SPHEREMAPPED;
  } else if (std::strcmp(name, "NORMAL_OCT16") == 0) {
    return attributes::NORMAL_OCT16;
  } else if (std::strcmp(name, "NORMAL") == 0) {
    return attributes::NORMAL;
  }

  throw std::runtime_error{ "Unrecognized PointAttribute name!" };
}

bool
operator==(const PointAttribute& lhs, const PointAttribute& rhs)
{
  return lhs.ordinal == rhs.ordinal;
}

std::string
PointAttributes::toString() const
{
  std::stringstream ss;
  ss << "[";
  for (size_t idx = 0; idx < attributes.size(); ++idx) {
    ss << attributes[idx].name;
    if (idx < (attributes.size() - 1))
      ss << ";";
  }
  ss << "]";
  return ss.str();
}

PointAttributes
point_attributes_from_strings(const std::vector<std::string>& attribute_names)
{
  PointAttributes point_attributes;
  point_attributes.add(attributes::POSITION_CARTESIAN);
  for (const auto& attribute : attribute_names) {
    if (attribute == "RGB") {
      point_attributes.add(attributes::COLOR_PACKED);
    } else if (attribute == "RGB_FROM_INTENSITY") {
      point_attributes.add(attributes::COLOR_FROM_INTENSITY);
    } else if (attribute == "INTENSITY") {
      point_attributes.add(attributes::INTENSITY);
    } else if (attribute == "CLASSIFICATION") {
      point_attributes.add(attributes::CLASSIFICATION);
    } else if (attribute == "NORMAL") {
      point_attributes.add(attributes::NORMAL_OCT16);
    }
  }
  return point_attributes;
}