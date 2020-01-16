#pragma once

#include "util/Units.h"

#include <boost/concept_check.hpp>
#include <numeric>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

/**
 * ################## MemoryIntrospectable Concept #####################
 *
 * Pre C++20 implementation of a concept for types that provide memory
 * introspection, i.e. they know their exact size in memory at runtime
 */

// Basic signature:
// template<typename T> Byte size_in_memory(T const&);

namespace concepts {

#pragma region has_constant_size

namespace detail {
template<typename T>
constexpr bool has_constant_size_impl = true;

template<>
constexpr bool has_constant_size_impl<std::string> = false;

template<typename T>
constexpr bool has_constant_size_impl<std::vector<T>> = false;

template<typename First, typename Second>
constexpr bool has_constant_size_impl<std::pair<First, Second>> =
  has_constant_size_impl<std::decay_t<First>>&& has_constant_size_impl<std::decay_t<Second>>;
}

template<typename T>
constexpr bool has_constant_size = detail::has_constant_size_impl<std::decay_t<T>>;

#pragma endregion

#pragma region size_in_memory implementations

namespace {
template<typename T>
struct AlwaysFalse : std::false_type
{};
}

template<typename T>
unit::byte
size_in_memory(T const&)
{
  if constexpr (has_constant_size<T>) {
    return sizeof(T) * boost::units::information::byte;
  } else {
    static_assert(AlwaysFalse<T>::value,
                  "Please provide a specialization of size_in_memory for type T");
    return {};
  }
}

template<>
inline unit::byte
size_in_memory(std::string const& str)
{
  // Strings use small-string-optimization, so we have to figure out if this string points
  // to dynamically allocated memory or not. We can do that by inspecting the pointer returned
  // from data()
  const auto data_ptr = reinterpret_cast<char const*>(str.data());
  const auto str_ptr = reinterpret_cast<char const*>(&str);
  if (data_ptr < str_ptr || data_ptr >= (str_ptr + sizeof(std::string))) {
    return (sizeof(std::string) + str.capacity() + 1) * boost::units::information::byte;
  }
  return sizeof(std::string) * boost::units::information::byte;
}

template<typename First, typename Second>
unit::byte
size_in_memory(std::pair<First, Second> const& pair)
{
  return concepts::size_in_memory(pair.first) + concepts::size_in_memory(pair.second);
}

template<typename... Types>
unit::byte
size_in_memory(std::tuple<Types...> const& tuple)
{
  return (concepts::size_in_memory(std::get<Types>(tuple)) + ...);
}

template<typename T>
unit::byte
size_in_memory(std::vector<T> const& vector)
{
  if constexpr (has_constant_size<T>) {
    return (sizeof(std::vector<T>) + vector.capacity() * sizeof(T)) *
           boost::units::information::byte;
  } else {
    const auto unused_elements = vector.capacity() - vector.size();
    const auto unused_elements_size = unused_elements * sizeof(T) * boost::units::information::byte;
    const auto used_elements_size =
      std::accumulate(std::begin(vector),
                      std::end(vector),
                      0 * boost::units::information::byte,
                      [](auto accum, auto const& elem) { return concepts::size_in_memory(elem); });
    return (sizeof(std::vector<T>) * boost::units::information::byte) + used_elements_size +
           unused_elements_size;
  }
}

#pragma endregion

template<typename T>
struct MemoryIntrospectable
{
  BOOST_CONCEPT_USAGE(MemoryIntrospectable) { size = concepts::size_in_memory(introspectable); }

private:
  T introspectable;
  unit::byte size;
};
}