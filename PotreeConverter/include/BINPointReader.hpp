

#ifndef BINPOINTREADER_H
#define BINPOINTREADER_H

#include <iostream>
#include <string>
#include <vector>

#include "Point.h"
#include "PointAttributes.hpp"
#include "PointReader.h"

using std::string;

using std::cout;
using std::endl;
using std::ifstream;
using std::vector;

namespace Potree {

class BINPointReader : public PointReader {
 private:
  AABB aabb;
  double scale;
  string path;
  vector<string> files;
  vector<string>::iterator currentFile;
  ifstream *reader;
  PointAttributes attributes;
  Point point;

  bool readNextPoint();

 public:
  BINPointReader(string path, AABB aabb, double scale,
                 PointAttributes pointAttributes);

  ~BINPointReader();

  PointBuffer readPointBatch(size_t maxBatchCount) override;

  AABB getAABB() override;

  long long numPoints() override;

  void close() override;

  Vector3<double> getScale();
};

}  // namespace Potree

#endif
