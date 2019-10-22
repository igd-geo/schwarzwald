#pragma once

#include <optional>
#include <string>

#include "PointAttributes.hpp"
#include "definitions.hpp"

/**
 * Run conversion process
 */
void
run_conversion(const std::string& source_folder,
               const std::string& target_folder,
               OutputFormat target_format,
               const PointAttributes& attributes,
               std::optional<std::string> projection, std::optional<uint32_t> max_depth);