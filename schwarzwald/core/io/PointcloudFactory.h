#pragma once

#include "LASFile.h"
#include "util/Error.h"

#include <expected.hpp>
#include <experimental/filesystem>
#include <system_error>
#include <variant>

using PointFile = std::variant<LASFile>; // and more files

/**
 * Open the given pointcloud file
 */
tl::expected<PointFile, util::ErrorChain>
open_point_file(const std::experimental::filesystem::path &path);

/**
 * Returns true if the given file format (as returned by
 * std::filesystem::path::extension()) is valid or not
 */
bool file_format_is_supported(const std::string &format);