#pragma once

#include "io/PointReader.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "process/Tiler.h"
#include "util/Definitions.h"
#include "util/Error.h"
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
    OutputFormat output_format;
    RGBMapping rgb_mapping;
    std::string sampling_strategy;
    std::string executable_path;
    std::optional<std::string> source_projection;
    std::optional<unit::byte> cache_size;
    bool use_compression;
    uint32_t max_memory_usage_MiB;
    util::IgnoreErrors errors_to_ignore;
    TilingStrategy tiling_strategy;
  };

  explicit TilerProcess(Arguments const& args);

  void run();

private:
  struct CalculateBoundsResult
  {
    AABB tight_bounds;
    AABB cubic_bounds;
    AABB cubic_bounds_at_origin;
  };

  Arguments _args;
  AABB _bounds;

  // Attributes read from the source files
  PointAttributes _input_attributes;
  // Attributes written to the output files
  PointAttributes _output_attributes;

  UIState _ui_state;
  TerminalUI _ui;

  void prepare();
  void cleanUp();
  CalculateBoundsResult calculate_bounds(SRSTransformHelper const* transform);
  size_t get_total_points_count() const;
  void check_for_missing_point_attributes(const PointAttributes& required_attributes) const;
  void determine_input_and_output_attributes();
  SamplingStrategy make_sampling_strategy() const;
  Tiler make_tiler(bool shift_points_to_center,
                   uint32_t max_depth,
                   SRSTransformHelper const* srs_transform,
                   AABB const& cubic_bounds,
                   AABB const& cubic_bounds_at_origin,
                   SamplingStrategy sampling_strategy,
                   ProgressReporter* progress_reporter,
                   PointsPersistence& persistence) const;
};
