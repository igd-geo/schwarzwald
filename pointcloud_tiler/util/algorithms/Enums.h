#pragma once

#include <expected.hpp>
#include <string>

#include "types/type_util.h"

namespace util {

/**
 * Try to parse an object of type T from the given string. Returns the parsed
 * object or the reason why parsing failed
 */
template <typename T>
tl::expected<T, std::string> try_parse(const std::string &str) {
  static_assert(AlwaysFalse<T>::value,
                "Please provide a specialization of try_parse for type T");
}

/**
 * Convert an object of type T to a string
 */
template <typename T> const std::string &to_string(T literal) {
  static_assert(AlwaysFalse<T>::value,
                "Please provide a specialization of to_string for type T");
  throw std::runtime_error{"unimplemented"};
}

} // namespace util