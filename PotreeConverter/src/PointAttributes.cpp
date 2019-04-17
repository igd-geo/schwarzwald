
#include "PointAttributes.hpp"
#include "PotreeException.h"

#include <sstream>

namespace Potree {

PointAttribute PointAttribute::fromString(const std::string& name) {
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

  throw PotreeException("Invalid PointAttribute name: '" + name + "'");
}

constexpr PointAttribute PointAttribute::fromStringLiteral(const char* name) {
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

//BUG: The throw does not compile with gcc (7.x.x), but the C++ standard says this is conformant?
#ifdef _WIN32
  throw std::runtime_error{"Unrecognized PointAttribute name!"};
#endif  
}

bool operator==(const PointAttribute& lhs, const PointAttribute& rhs) {
  return lhs.ordinal == rhs.ordinal;
}

std::string PointAttributes::toString() const {
  std::stringstream ss;
  ss << "[";
  for (size_t idx = 0; idx < attributes.size(); ++idx) {
    ss << attributes[idx].name;
    if (idx < (attributes.size() - 1)) ss << ";";
  }
  ss << "]";
  return ss.str();
}

}  // namespace Potree
