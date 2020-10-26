#pragma once

#include <experimental/filesystem>

namespace util {

/**
 * A hash functor that can has filesystem::path
 */
struct PathHash
{
  size_t operator()(const std::experimental::filesystem::path& path) const
  {
    return std::experimental::filesystem::hash_value(path);
  }
};

}