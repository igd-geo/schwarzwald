#pragma once

#include "LASFile.h"
#include "util/Error.h"

#include <expected.hpp>
#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif
#include <system_error>
#include <variant>

using PointFile = std::variant<LASFile>; // and more files

/**
 * Open the given pointcloud file
 */
#ifdef _WIN32
tl::expected<PointFile, util::ErrorChain>
open_point_file(const std::filesystem::path &path);
#else
tl::expected<PointFile, util::ErrorChain>
open_point_file(const std::experimental::filesystem::path &path);
#endif

/**
 * Returns true if the given file format (as returned by
 * std::filesystem::path::extension()) is valid or not
 */
bool file_format_is_supported(const std::string &format);