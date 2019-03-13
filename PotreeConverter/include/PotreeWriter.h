#ifndef POTREEWRITER_H
#define POTREEWRITER_H

#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "AABB.h"
#include "CloudJS.hpp"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "SparseGrid.h"
#include "Tileset.h"
#include "Transformation.h"

using std::string;
using std::thread;
using std::vector;

namespace Potree {

class PotreeWriter;
class PointReader;
class PointWriter;

class PWNode {
 public:
  int index = -1;
  AABB aabb;
  AABB acceptedAABB;
  int level = 0;
  SparseGrid *grid;
  unsigned int numAccepted = 0;
  PWNode *parent = NULL;
  vector<PWNode *> children;
  bool addedSinceLastFlush = true;
  bool addCalledSinceLastFlush = false;
  PotreeWriter *potreeWriter;
  PointBuffer cache;
  int storeLimit = 20'000;
  PointBuffer store;
  bool isInMemory = true;

  Tileset tileset;

  PWNode(PotreeWriter *potreeWriter, AABB aabb, const SRSTransformHelper &);

  PWNode(PotreeWriter *potreeWriter, int index, AABB aabb, int level,
         const SRSTransformHelper &);

  ~PWNode();

  string name() const;

  float spacing();

  bool isLeafNode() { return children.size() == 0; }

  bool isInnerNode() { return children.size() > 0; }

  void loadFromDisk();

  PWNode *add(PointBuffer::PointReference point);

  PWNode *createChild(int childIndex);

  void split();

  string workDir();

  string hierarchyPath();

  string path();

  void flush();

  void traverse(std::function<void(PWNode *)> callback);

  void traverseBreadthFirst(std::function<void(PWNode *)> callback);

  vector<PWNode *> getHierarchy(int levels);

  PWNode *findNode(string name);

 private:
  PointReader *createReader(string path);
  PointWriter *createWriter(string path);

  const SRSTransformHelper &_transformHelper;
};

class PotreeWriter {
 public:
  AABB aabb;
  AABB tightAABB;
  string workDir;
  float spacing;
  double scale = 0;
  int maxDepth = -1;
  PWNode *root;
  long long numAdded = 0;
  long long numAccepted = 0;
  CloudJS cloudjs;
  OutputFormat outputFormat;
  PointAttributes pointAttributes;
  int hierarchyStepSize = 5;
  PointBuffer store;
  thread storeThread;
  int pointsInMemory = 0;
  string projection = "";
  ConversionQuality quality = ConversionQuality::DEFAULT;

  PotreeWriter(string workDir, ConversionQuality quality,
               const SRSTransformHelper &transform);

  PotreeWriter(string workDir, AABB aabb, float spacing, int maxDepth,
               double scale, OutputFormat outputFormat,
               PointAttributes pointAttributes, ConversionQuality quality,
               const SRSTransformHelper &transform);

  ~PotreeWriter() {
    close();

    delete root;
  }

  string getExtension();

  void processStore();

  void waitUntilProcessed();

  void add(const PointBuffer &points);

  void flush();

  void close() { flush(); }

  void setProjection(string projection);

  void loadStateFromDisk();

 private:
  const SRSTransformHelper &_transform;
};

}  // namespace Potree

#endif
