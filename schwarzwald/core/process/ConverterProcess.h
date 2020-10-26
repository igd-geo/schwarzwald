#pragma once

#include <optional>
#include <string>

#include "pointcloud/PointAttributes.h"
#include "util/Definitions.h"

struct ConverterArguments
{
  std::string source_folder;
  std::string output_folder;
  OutputFormat output_format;
  PointAttributes output_attributes;
  std::optional<std::string> source_projection;
  std::optional<uint32_t> max_depth;
  bool delete_source_files;
};

/**
 * Run conversion process
 */
void
run_conversion(const ConverterArguments& args);