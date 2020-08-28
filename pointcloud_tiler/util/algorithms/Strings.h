#pragma once

#include "types/type_util.h"

#include <expected.hpp>
#include <gsl/gsl>
#include <sstream>
#include <variant>

namespace util {

template<typename... Args>
std::string
concat(const Args&... args)
{
  std::stringstream ss;
  (ss << ... << args);
  return ss.str();
}

namespace detail {
template<typename T>
T
str_to(const std::string& str, std::size_t* pos, int base)
{
  static_assert(AlwaysFalse<T>::value, "try_parse_number not defined for type T!");
}

#define GEN_STR_TO_SPECIALIZATION(type, func)                                                      \
  template<>                                                                                       \
  inline type str_to(const std::string& str, std::size_t* pos, int base)                           \
  {                                                                                                \
    const auto parsed = func(str, pos, base);                                                      \
    if (parsed > std::numeric_limits<type>::max()) {                                               \
      throw std::out_of_range{ "Value is out of range for type " #type };                          \
    }                                                                                              \
    return gsl::narrow_cast<type>(parsed);                                                         \
  }

GEN_STR_TO_SPECIALIZATION(uint8_t, std::stoul)
GEN_STR_TO_SPECIALIZATION(int8_t, std::stol)
GEN_STR_TO_SPECIALIZATION(uint16_t, std::stoul)
GEN_STR_TO_SPECIALIZATION(int16_t, std::stol)
GEN_STR_TO_SPECIALIZATION(uint32_t, std::stoul)
GEN_STR_TO_SPECIALIZATION(int32_t, std::stol)
GEN_STR_TO_SPECIALIZATION(uint64_t, std::stoull)
GEN_STR_TO_SPECIALIZATION(int64_t, std::stoll)

template<>
inline float
str_to(const std::string& str, std::size_t* pos, int)
{
  return std::stof(str, pos);
}

template<>
inline double
str_to(const std::string& str, std::size_t* pos, int)
{
  return std::stod(str, pos);
}

template<>
inline long double
str_to(const std::string& str, std::size_t* pos, int)
{
  return std::stold(str, pos);
}

}

/**
 * Wrapper around the std::stoX functions. Tries to parse the given string as a number
 * of the given type and returns a tl::expected with the resulting number of the reason
 * for failure. The arguments are the same as for the std functions, with the exception that
 * base is only valid if T is an integer type
 */
template<typename T>
tl::expected<T, std::variant<std::invalid_argument, std::out_of_range>>
try_parse_number(const std::string& str, std::size_t* pos = nullptr, int base = 10)
{
  try {
    const auto number = detail::str_to<T>(str, pos, base);
    return { number };
  } catch (const std::invalid_argument& ex) {
    return tl::make_unexpected(std::variant<std::invalid_argument, std::out_of_range>{ ex });
  } catch (const std::out_of_range& ex) {
    return tl::make_unexpected(std::variant<std::invalid_argument, std::out_of_range>{ ex });
  }
}

} // namespace util