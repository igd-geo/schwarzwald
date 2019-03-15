

#ifndef POTREE_CONVERTER_H
#define POTREE_CONVERTER_H

#include "AABB.h"
#include "PointAttributes.hpp"
#include "PointReader.h"
#include "definitions.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

using std::string;
using std::vector;

namespace Potree {

class SparseGrid;

class PotreeConverter {
 private:
  AABB aabb;
  vector<string> sources;
  string workDir;
  PointAttributes pointAttributes;

  PointReader* createPointReader(string source,
                                 const PointAttributes& pointAttributes);
  void prepare();
  void cleanUp();
  AABB calculateAABB();
  void generatePage(string name);

 public:
  float spacing;
  int maxDepth;
  string format;
  OutputFormat outputFormat;
  vector<string> outputAttributes;
  vector<double> colorRange;
  vector<double> intensityRange;
  double scale = 0.01;
  int diagonalFraction = 250;
  vector<double> aabbValues;
  string pageName = "";
  string pageTemplatePath = "";
  StoreOption storeOption = StoreOption::ABORT_IF_EXISTS;
  bool sourceListingOnly = false;
  ConversionQuality quality = ConversionQuality::DEFAULT;
  string title = "PotreeViewer";
  string description = "";
  bool edlEnabled = false;
  bool showSkybox = false;
  string material = "RGB";
  string executablePath;
  std::optional<string> sourceProjection;

  PotreeConverter(string executablePath, string workDir,
                  vector<string> sources);

  void convert();
};

}  // namespace Potree

#endif
