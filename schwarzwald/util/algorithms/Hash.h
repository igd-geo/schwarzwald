#pragma once

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

namespace util {

/**
 * A hash functor that can has filesystem::path
 */
struct PathHash
{
#ifdef _WIN32
  size_t operator()(const std::filesystem::path& path) const
  {
    return std::filesystem::hash_value(path);
  }
#else
  size_t operator()(const std::experimental::filesystem::path& path) const
  {
    return std::experimental::filesystem::hash_value(path);
  }
#endif
};

}