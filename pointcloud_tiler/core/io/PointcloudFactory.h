#pragma once

#include "LASFile.h"

#include <expected.hpp>
#include <experimental/filesystem>
#include <system_error>
#include <variant>

using PointFile = std::variant<LASFile>; // and more files

/**
 * Open the given pointcloud file
 */
tl::expected<PointFile, std::error_code>
open_point_file(const std::experimental::filesystem::path& path);