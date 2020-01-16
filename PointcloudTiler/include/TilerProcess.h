#pragma once

#include "AABB.h"
#include "Definitions.h"
#include "PointAttributes.hpp"
#include "PointReader.h"
#include "ui/TerminalUI.hpp"

#include <cstdint>
#include <experimental/filesystem>
#include <optional>
#include <string>
#include <vector>

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
    std::string sampling_strategy;
    std::string executable_path;
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
  AABB calculateAABB();
  size_t get_total_points_count() const;
};
