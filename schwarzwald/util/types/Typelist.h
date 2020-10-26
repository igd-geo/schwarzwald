#pragma once

#include "types/type_util.h"

#include <stdexcept>

namespace meta {

/**
 * Simple typelist for storing types at compile time
 */
template<typename... Types>
struct Typelist
{};

template<typename T>
inline constexpr int size(T = {})
{
  static_assert(AlwaysFalse<T>::value, "size only defined for Typelist argument");
  return 0;
}

/**
 * Returns the size of the given Typelist, i.e. the number of types it stores
 */
template<typename... Types>
inline constexpr int size(Typelist<Types...> = {})
{
  return sizeof...(Types);
}

template<int Index, typename T>
inline constexpr void type_at(T = {})
{
  static_assert(AlwaysFalse<T>::value, "type_at only defined for Typelist argument");
}

/**
 * Get the type at the given index
 */
template<int Index, typename First, typename... Rest>
inline constexpr decltype(auto) type_at(Typelist<First, Rest...> = {})
{
  if constexpr (Index > sizeof...(Rest)) {
    throw std::invalid_argument{ "Index is out of bounds!" };
  }

  if constexpr (Index == 0) {
    return First{};
  }

  return type_at<Index, Rest...>();
}

template<template<typename T> typename Func>
inline constexpr bool all_of(Typelist<> = {})
{
  return true;
}

template<template<typename T> typename Func, typename First, typename... Rest>
inline constexpr bool all_of(Typelist<First, Rest...> = {})
{
  if constexpr (sizeof...(Rest) == 0) {
    return Func<First>{}();
  } else {
    return Func<First>{}() && all_of<Func, Rest...>();
  }
}

}