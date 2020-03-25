#pragma once

#include <sstream>

namespace util {

template <typename... Args> std::string concat(const Args &... args) {
  std::stringstream ss;
  (ss << ... << args);
  return ss.str();
}

} // namespace util