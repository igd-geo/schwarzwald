#pragma once

#include "AABB.h"
#include "PointAttributes.hpp"
#include "PointReader.h"
#include "definitions.hpp"
#include "ui/TerminalUI.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

class SparseGrid;

class TilerProcess
{
private:
  AABB aabb;
  std::vector<std::string> sources;
  std::string workDir;
  PointAttributes pointAttributes;

  UIState _ui_state;
  TerminalUI _ui;

  PointReader* createPointReader(const std::string& source,
                                 const PointAttributes& pointAttributes) const;
  void prepare();
  void cleanUp();
  AABB calculateAABB();
  size_t get_total_points_count() const;

public:
  float spacing;
  int maxDepth;
  size_t max_points_per_node;
  OutputFormat outputFormat;
  std::vector<std::string> outputAttributes;
  int diagonalFraction = 250;
  std::string samplingStrategy;
  uint32_t max_memory_usage_MiB;

  TilerProcess(const std::string& workDir, std::vector<std::string> sources);

  void run();
};
