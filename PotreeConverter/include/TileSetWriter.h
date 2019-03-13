#ifndef TILESETWRITER_H
#define TILESETWRITER_H

#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "PointAttributes.hpp"
#include "PointWriter.hpp"

#include "PNTWriter.h"
#include "Point.h"
#include "Tileset.h"

namespace Potree {

class TileSetWriter : public PointWriter {
 public:
  TileSetWriter(const std::string& filePath, const AABB& aabb, double scale,
                const PointAttributes& pointAttributes);
  ~TileSetWriter();

  void writePoints(const PointBuffer& points) override;
  bool writeTileset(const string& workDir, const Tileset& tileset) override;

  void close() override;

 private:
  std::string _filePath;
  PointAttributes _pointAttributes;
  AABB _aabb;
  double _scale;
  std::unique_ptr<PNTWriter> _pntWriter;

  bool writeTilesetJSON(const string& workDir, const Tileset& tileset);
};

}  // namespace Potree

#endif
