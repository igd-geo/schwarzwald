

#ifndef POTREE_CONVERTER_H
#define POTREE_CONVERTER_H

#include "AABB.h"
#include "PointAttributes.hpp"
#include "PointReader.h"
#include "definitions.hpp"
#include "ui/TerminalUI.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

using std::string;
using std::vector;

namespace Potree {

class SparseGrid;

class PotreeConverter
{
private:
  AABB aabb;
  vector<string> sources;
  string workDir;
  PointAttributes pointAttributes;

  UIState _ui_state;
  TerminalUI _ui;

  PointReader* createPointReader(const string& source,
                                 const PointAttributes& pointAttributes) const;
  void prepare();
  void cleanUp();
  AABB calculateAABB();
  size_t get_total_points_count() const;

public:
  float spacing;
  int maxDepth;
  OutputFormat outputFormat;
  vector<string> outputAttributes;
  int diagonalFraction = 250;
  ConversionQuality quality = ConversionQuality::DEFAULT;
  std::optional<string> sourceProjection;
  uint32_t max_memory_usage_MiB;

  PotreeConverter(const string& workDir, vector<string> sources);

  void convert();
};

} // namespace Potree

#endif
