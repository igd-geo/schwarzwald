#pragma once

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

/**
 * Global configuration options of the tiler. This is stuff that is useful to have
 * access to from some places in the code, e.g. for debugging, so it is stored in a
 * global struct
 */
struct TilerConfig
{
#ifdef _WIN32
  std::filesystem::path root_directory;
#else
  std::experimental::filesystem::path root_directory;
#endif
  bool is_journaling_enabled;
#ifdef _WIN32
  std::filesystem::path journal_directory;
#else
  std::experimental::filesystem::path journal_directory;
#endif
};

/**
 * Access the global TilerConfig object
 */
TilerConfig&
global_config();