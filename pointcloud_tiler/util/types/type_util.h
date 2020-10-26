#pragma once

#include <type_traits>

template<class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template<class... Ts>
overloaded(Ts...)->overloaded<Ts...>;

template<typename T>
struct AlwaysFalse : std::bool_constant<false>
{};

template<unsigned T>
struct AlwaysFalseForIntegralTypes : std::bool_constant<false>
{};

/**
 * Predicate that always returns true
 */
struct AlwaysTruePredicate
{
  template<typename T>
  bool operator()(T&& arg) const
  {
    return true;
  }
};