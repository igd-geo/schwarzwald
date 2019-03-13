
#ifndef POINTWRITER_H
#define POINTWRITER_H

#include <iostream>
#include <string>

#include "PointBuffer.h"

using std::string;
class Tileset;

namespace Potree {

class PointWriter {
 public:
  int numPoints = 0;

  virtual ~PointWriter(){};

  virtual void writePoints(const PointBuffer& points) = 0;

  virtual bool writeTileset(const std::string& workDir,
                            const Tileset& tileset) {
    return false;
  }

  virtual void close() = 0;
};

}  // namespace Potree

#endif
