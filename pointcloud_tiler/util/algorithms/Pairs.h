#pragma once

#include <functional>

namespace util {

struct PairHash
{
  template<typename First, typename Second>
  std::size_t operator()(const std::pair<First, Second>& pair) const
  {
    return std::hash<First>()(pair.first) ^ std::hash<Second>()(pair.second);
  }
};

}