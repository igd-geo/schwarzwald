#pragma once

#include <experimental/filesystem>

/**
 * Global configuration options of the tiler. This is stuff that is useful to have
 * access to from some places in the code, e.g. for debugging, so it is stored in a
 * global struct
 */
struct TilerConfig
{
  std::experimental::filesystem::path root_directory;
  bool is_journaling_enabled;
  std::experimental::filesystem::path journal_directory;
};

/**
 * Access the global TilerConfig object
 */
TilerConfig&
global_config();