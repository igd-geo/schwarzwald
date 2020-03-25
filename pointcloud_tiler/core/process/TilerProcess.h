#pragma once

#include "io/PointReader.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "util/Definitions.h"
#include <terminal/TerminalUI.h>

#include <cstdint>
#include <experimental/filesystem>
#include <optional>
#include <string>
#include <vector>

struct SRSTransformHelper;

struct TilerProcess
{
  struct Arguments
  {
    std::vector<fs::path> sources;
    fs::path output_directory;
    float spacing;
    int levels;
    int diagonal_fraction;
    uint32_t max_depth;
    size_t max_points_per_node;
    size_t internal_cache_size;
    size_t max_batch_read_size;
    std::vector<std::string> output_attributes;
    OutputFormat output_format;
    std::string sampling_strategy;
    std::string executable_path;
    std::optional<std::string> source_projection;
    std::optional<unit::byte> cache_size;
    bool use_compression;
    uint32_t max_memory_usage_MiB;
  };

  explicit TilerProcess(Arguments const& args);

  void run();

private:
  Arguments _args;
  AABB _bounds;
  PointAttributes _point_attributes;

  UIState _ui_state;
  TerminalUI _ui;

  void prepare();
  void cleanUp();
  AABB calculateAABB(SRSTransformHelper const* transform);
  size_t get_total_points_count() const;
};
