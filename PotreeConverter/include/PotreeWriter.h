#ifndef POTREEWRITER_H
#define POTREEWRITER_H

#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "AABB.h"
#include "IWriter.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"
#include "SparseGrid.h"
#include "Tileset.h"
#include "Transformation.h"
#include "definitions.hpp"

using std::string;
using std::thread;
using std::vector;

namespace Potree {

class PotreeWriter;
class PointReader;
class PointWriter;

class PWNode
{
public:
  int index = -1;
  AABB aabb;
  AABB acceptedAABB;
  int level = 0;
  SparseGrid* grid;
  unsigned int numAccepted = 0;
  PWNode* parent = NULL;
  vector<PWNode*> children;
  bool addedSinceLastFlush = true;
  bool addCalledSinceLastFlush = false;
  PotreeWriter* potreeWriter;
  PointBuffer cache;
  int storeLimit = 20'000;
  PointBuffer store;
  bool isInMemory = true;

  Tileset tileset;

  PWNode(PotreeWriter* potreeWriter, AABB aabb, const SRSTransformHelper&);

  PWNode(PotreeWriter* potreeWriter, int index, AABB aabb, int level, const SRSTransformHelper&);

  ~PWNode();

  string name() const;

  float spacing();

  bool isLeafNode() { return children.size() == 0; }

  bool isInnerNode() { return children.size() > 0; }

  void loadFromDisk();

  PWNode* add(PointBuffer::PointConstReference point);

  PWNode* createChild(int childIndex);

  void split();

  std::string workDir() const;
  std::string jsonPathAbsolute() const;
  std::string pntsPathAbsolute() const;
  std::string jsonPathRelative() const;
  std::string pntsPathRelative() const;

  void flush();

  void traverse(std::function<void(PWNode*)> callback);

  void traverseBreadthFirst(std::function<void(PWNode*)> callback);

  vector<PWNode*> getHierarchy(int levels);

  PWNode* findNode(string name);

  /// <summary>
  /// Returns an estimate for the binary size of this PWNode in memory. This includes the
  /// dynamically allocated memory for the store, cache, grid as well as the memory for all child
  /// nodes!
  /// </summary>
  size_t estimate_binary_size() const;

private:
  PointReader* createReader(string path);

  const SRSTransformHelper& _transformHelper;
};

class PotreeWriter : IWriter
{
public:
  AABB aabb;
  AABB tightAABB;
  string workDir;
  float spacing;
  double scale = 0;
  int maxDepth = -1;
  PWNode* root;
  long long numAccepted = 0;
  OutputFormat outputFormat;
  PointAttributes pointAttributes;
  int hierarchyStepSize = 5;
  PointBuffer store;
  thread storeThread;
  int pointsInMemory = 0;
  ConversionQuality quality = ConversionQuality::DEFAULT;

  PotreeWriter(string workDir,
               ConversionQuality quality,
               const SRSTransformHelper& transform,
               uint32_t max_memory_usage_MiB);

  PotreeWriter(string workDir,
               AABB aabb,
               float spacing,
               int maxDepth,
               double scale,
               OutputFormat outputFormat,
               PointAttributes pointAttributes,
               ConversionQuality quality,
               const SRSTransformHelper& transform,
               uint32_t max_memory_usage_MiB);

  ~PotreeWriter()
  {
    flush();

    delete root;
  }

  string getExtension();

  void index() override;

  void wait_until_indexed() override;

  bool needs_indexing() const override;

  void cache(const PointBuffer& points) override;

  void flush() override;

  bool needs_flush() const override;

  void close() override;

private:
  const SRSTransformHelper& _transform;
  uint32_t _max_memory_usage_MiB;
  bool _needs_flush;
};

} // namespace Potree

#endif
