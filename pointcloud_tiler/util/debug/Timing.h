#pragma once

#include <chrono>
#include <tuple>

namespace util {

template<typename Func>
auto
time(Func f) -> std::pair<decltype(f()), std::chrono::nanoseconds>
{
  const auto t_start = std::chrono::high_resolution_clock::now();
  auto res = f();
  const auto dt = std::chrono::high_resolution_clock::now() - t_start;
  return { std::move(res), dt };
}

}