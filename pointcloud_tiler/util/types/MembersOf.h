#pragma once

#include "types/Typelist.h"

#include <cista/reflection/arity.h>
#include <type_traits>

namespace refl {

namespace detail {

template<typename T>
inline constexpr decltype(auto)
members_of_aggregate()
{
  static_assert(std::is_aggregate_v<T>, "T must be aggregate type");
  static_assert(std::is_default_constructible_v<T>, "T must be default constructible");

  constexpr auto const arity = cista::arity<T>();
  if constexpr (arity == 0) {
    return meta::Typelist<>{};
  } else if constexpr (arity == 1) {
    /**
     * This horrible piece of code exists because we need an actual instance of the
     * type T so that we can use the structured bindings trick to get all the (public)
     * members. We can't use std::declval because this function is actually executed
     * (at compile-time). It doesn't matter that 'dummy' contains garbage, we only need
     * a valid instance that we can assign to the structured binding
     */
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>>{};
  } else if constexpr (arity == 2) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>>{};
  } else if constexpr (arity == 3) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2, m3] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>,
                          std::remove_reference_t<decltype(m3)>>{};
  } else if constexpr (arity == 4) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2, m3, m4] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>,
                          std::remove_reference_t<decltype(m3)>,
                          std::remove_reference_t<decltype(m4)>>{};
  } else if constexpr (arity == 5) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2, m3, m4, m5] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>,
                          std::remove_reference_t<decltype(m3)>,
                          std::remove_reference_t<decltype(m4)>,
                          std::remove_reference_t<decltype(m5)>>{};
  } else if constexpr (arity == 6) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2, m3, m4, m5, m6] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>,
                          std::remove_reference_t<decltype(m3)>,
                          std::remove_reference_t<decltype(m4)>,
                          std::remove_reference_t<decltype(m5)>,
                          std::remove_reference_t<decltype(m6)>>{};
  } else if constexpr (arity == 7) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2, m3, m4, m5, m6, m7] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>,
                          std::remove_reference_t<decltype(m3)>,
                          std::remove_reference_t<decltype(m4)>,
                          std::remove_reference_t<decltype(m5)>,
                          std::remove_reference_t<decltype(m6)>,
                          std::remove_reference_t<decltype(m7)>>{};
  } else if constexpr (arity == 8) {
    int mem = 0;
    T* dummy = reinterpret_cast<T*>(&mem);
    auto& [m1, m2, m3, m4, m5, m6, m7, m8] = *dummy;
    return meta::Typelist<std::remove_reference_t<decltype(m1)>,
                          std::remove_reference_t<decltype(m2)>,
                          std::remove_reference_t<decltype(m3)>,
                          std::remove_reference_t<decltype(m4)>,
                          std::remove_reference_t<decltype(m5)>,
                          std::remove_reference_t<decltype(m6)>,
                          std::remove_reference_t<decltype(m7)>,
                          std::remove_reference_t<decltype(m8)>>{};
  } else {
    // TODO More members
    static_assert(AlwaysFalse<T>::value,
                  "StaticReflection not supported for aggregate type with more than 8 members");
  }
}
}

/**
 * Returns a Typelist containing the types of the members of type T in the order that they are
 * declared
 */
template<typename T>
inline constexpr decltype(auto)
members_of()
{
  if constexpr (std::is_scalar_v<T>) {
    return meta::Typelist<>{};
  } else if constexpr (std::is_aggregate_v<T>) {
    return detail::members_of_aggregate<T>();
  } else {
    static_assert(AlwaysFalse<T>::value,
                  "StaticReflection only supports scalar and aggregate types at the moment");
  }
}

/**
 * Is the given type reflectable using this simple static reflection library?
 */
template<typename T>
inline constexpr bool
is_reflectable()
{
  return std::is_scalar_v<T> || std::is_aggregate_v<T>;
}

}