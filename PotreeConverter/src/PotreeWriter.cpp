#include "PotreeWriter.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stack>

#include <experimental/filesystem>
#include <glm/gtc/matrix_transform.hpp>

#include "AABB.h"
#include "BINPointReader.hpp"
#include "CloudJS.hpp"
#include "LASPointReader.h"
#include "PNTSReader.h"
#include "PointAttributes.hpp"
#include "PointReader.h"
#include "PotreeException.h"
#include "SparseGrid.h"
#include "stuff.h"

#include "PNTSWriter.h"
#include "TileSetWriter.h"

using std::ifstream;
using std::stack;
using std::stringstream;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace fs = std::experimental::filesystem;

namespace Potree {

constexpr double GEOMETRIC_ERROR_CORRECTION_FACTOR = 2.0;

/// <summary>
/// Flush the given PointBuffer into a .pnts file. If desired, the points are
/// appended to a potentially existing file
/// </summary>
static void flushPoints(PointBuffer &points, const std::string &filepath,
                        const PointAttributes &pointAttributes,
                        const SRSTransformHelper &transformHelper,
                        bool appendToExistingFile, bool transformInPlace) {
  const auto path = fs::path{filepath};
  const auto fileParentDirectory = path.parent_path();
  // TODO Make sure the root folder exists (this is the one specified as command
  // line argument)
  // if (!fs::exists(workDir() + "/data/" + hierarchyPath())) {
  //  fs::create_directories(workDir() + "/data/" + hierarchyPath());
  //}

  PNTSWriter writer{filepath, pointAttributes};

  std::optional<Vector3<double>> existingLocalOffsetToWorld;

  if (appendToExistingFile) {
    // Move current PNTS file into temporary path (only if we don't
    // support in-place append), read from it, write it to the writer and
    // delete the temporary file
    const auto temppath = fileParentDirectory.string() + "/temp/prepend.bin";
    if (fs::exists(filepath)) {
      fs::create_directories(fileParentDirectory.string() + "/temp");
      fs::rename(fs::path(filepath), fs::path(temppath));
    }

    if (fs::exists(temppath)) {
      // TODO Create PNTS reader or appender
      // The PNTS reader should return the LOCAL positions and return the
      // RTC_CENTER set for it so that we can use this as the center for the new
      // points that are passed into this method

      // PointReader *reader = nullptr;
      // while (true) {
      //  auto pointBatch = reader->readPointBatch();
      //  if (pointBatch.empty()) break;
      //  writer.writePoints(pointBatch);
      //}
      // reader->close();
      // delete reader;
      // fs::remove(temppath);

      const auto pntsFileContents = readPNTSFile(temppath);
      if (!pntsFileContents) {
        std::cerr << "Could not append to \"" << filepath
                  << "\", replacing .pnts file instead..." << std::endl;
      } else {
        existingLocalOffsetToWorld =
            std::make_optional(pntsFileContents->rtc_center);
        if (pntsFileContents->points.count()) {
          writer.writePoints(pntsFileContents->points);
        }
      }

      fs::remove(temppath);
    }
  } else {
    if (fs::exists(filepath)) {
      fs::remove(filepath);
    }
    // writer = createWriter(filepath);
  }

  // Transform the point positions on WRITE. This way, the creation of the
  // octree structure is not affected, but we get the correct positions
  // for Cesium. Additionally, we transform all point coordinates into a
  // local coordinate system centered around the smallest point to prevent
  // jitter during rendering

  Vector3<double> localOffsetToWorld;
  if (transformInPlace) {
    transformHelper.transformPositionsTo(TargetSRS::CesiumWorld,
                                         gsl::make_span(points.positions()));
    if (existingLocalOffsetToWorld) {
      // Transform into the existing local coordinate system based on the points
      // that we append to
      localOffsetToWorld = *existingLocalOffsetToWorld;
      for (auto &position : points.positions()) {
        position -= localOffsetToWorld;
      }
    } else {
      localOffsetToWorld = setOriginToSmallestPoint(points.positions());
    }
    writer.writePoints(points);
  } else {
    // TODO Figure out a way to get rid of this copy
    auto pointsCopy = points;
    transformHelper.transformPositionsTo(
        TargetSRS::CesiumWorld, gsl::make_span(pointsCopy.positions()));
    if (existingLocalOffsetToWorld) {
      localOffsetToWorld = *existingLocalOffsetToWorld;
      for (auto &position : pointsCopy.positions()) {
        position -= localOffsetToWorld;
      }
    } else {
      localOffsetToWorld = setOriginToSmallestPoint(pointsCopy.positions());
    }
    writer.writePoints(pointsCopy);
  }

  writer.flush(localOffsetToWorld);

  // this->tileset.localCenter = localOffsetToWorld;
  // writer->writeTileset(filepath, this->tileset);

  /*
  if(append && (writer->numPoints != this->numAccepted)){ // !!
          cout << "writeToDisk " << writer->numPoints  << " != " <<
  this->numAccepted << endl; exit(1);
  }
  */

  // writer->close();
}

PWNode::PWNode(PotreeWriter *potreeWriter, AABB aabb,
               const SRSTransformHelper &transformHelper)
    : tileset(""), _transformHelper(transformHelper) {
  this->potreeWriter = potreeWriter;
  this->aabb = aabb;
  this->grid = new SparseGrid(aabb, spacing());

  // this->tileset.content_url = hierarchyPath() + name() + ".pnt";
  tileset.content_url = pntsPathRelative();
  tileset.boundingVolume = boundingVolumeFromAABB(aabb, transformHelper);
  tileset.geometricError = spacing() * GEOMETRIC_ERROR_CORRECTION_FACTOR;
}

PWNode::PWNode(PotreeWriter *potreeWriter, int index, AABB aabb, int level,
               const SRSTransformHelper &transformHelper)
    : tileset(""), _transformHelper(transformHelper) {
  this->index = index;
  this->aabb = aabb;
  this->level = level;
  this->potreeWriter = potreeWriter;
  this->grid = new SparseGrid(aabb, spacing());

  // this->tileset.content_url = hierarchyPath() + name() + ".pnt";
  tileset.content_url = pntsPathRelative();
  tileset.boundingVolume = boundingVolumeFromAABB(aabb, transformHelper);
  tileset.geometricError = spacing() * GEOMETRIC_ERROR_CORRECTION_FACTOR;
}

PWNode::~PWNode() {
  for (PWNode *child : children) {
    if (child != NULL) {
      delete child;
    }
  }
  delete grid;
}

string PWNode::name() const {
  if (parent == NULL) {
    return "r";
  } else {
    return parent->name() + std::to_string(index);
  }
}

float PWNode::spacing() {
  return float(potreeWriter->spacing / pow(2.0, float(level)));
}

string PWNode::workDir() const { return potreeWriter->workDir; }

std::string PWNode::jsonPathAbsolute() const {
  return workDir() + "/" + name() + ".json";
}
std::string PWNode::pntsPathAbsolute() const {
  return workDir() + "/" + name() + ".pnts";
}

std::string PWNode::jsonPathRelative() const { return name() + ".json"; }
std::string PWNode::pntsPathRelative() const { return name() + ".pnts"; }

PointReader *PWNode::createReader(string path) {
  PointReader *reader = NULL;
  OutputFormat outputFormat = this->potreeWriter->outputFormat;
  if (outputFormat == OutputFormat::LAS || outputFormat == OutputFormat::LAZ) {
    reader = new LASPointReader(path);
  } else if (outputFormat == OutputFormat::BINARY) {
    reader = new BINPointReader(path, aabb, potreeWriter->scale,
                                this->potreeWriter->pointAttributes);
  }

  return reader;
}

void PWNode::loadFromDisk() {
  // This loads the contents of a node from disk. Nodes are evicted from memory
  // if they are flushed and no points have been added since the last flush
  auto pntsFileContents = readPNTSFile(pntsPathAbsolute());
  if (!pntsFileContents) {
    std::cerr << "Could not retrieve points of \"" << pntsPathAbsolute()
              << "\" from disk, they will be lost..." << std::endl;
    return;
  }

  // Transform point positions back to world
  auto &points = pntsFileContents->points;
  for (auto &position : points.positions()) {
    position += pntsFileContents->rtc_center;
  }

  if (isLeafNode()) {
    store.append_buffer(points);
  } else {
    for (auto &position : points.positions()) {
      grid->addWithoutCheck(position);
    }
  }

  grid->numAccepted = numAccepted;

  isInMemory = true;
}

PWNode *PWNode::createChild(int childIndex) {
  AABB cAABB = childAABB(aabb, childIndex);
  PWNode *child =
      new PWNode(potreeWriter, childIndex, cAABB, level + 1, _transformHelper);
  child->parent = this;
  children[childIndex] = child;

  // Write childs properties
  // TODO: correct geometric errors

  child->tileset.boundingVolume =
      boundingVolumeFromAABB(aabb, _transformHelper);

  child->tileset.url = child->jsonPathRelative();

  child->tileset.geometricError = this->tileset.geometricError / 2;

  child->tileset.content_url = child->pntsPathRelative();

  if (child->tileset.geometricError < 0) {
    child->tileset.geometricError = 0;
  }

  this->tileset.children.push_back(&child->tileset);

  return child;
}

void PWNode::split() {  // Update tileset.json children ??
  children.resize(8, nullptr);

  const auto jsonFilepath = jsonPathAbsolute();
  const auto pntsFilepath = pntsPathAbsolute();
  if (fs::exists(jsonFilepath)) {
    fs::remove(jsonFilepath);
  }
  if (fs::exists(pntsFilepath)) {
    fs::remove(pntsFilepath);
  }

  for (auto point : store) {
    add(point);
  }

  store.clear();
}

PWNode *PWNode::add(PointBuffer::PointReference point) {
  addCalledSinceLastFlush = true;

  if (!isInMemory) {
    loadFromDisk();
  }

  if (isLeafNode()) {  // If the node is a leaf node just append the point
                       // (split)
    store.push_point(point);
    if (int(store.count()) >= storeLimit) {
      split();
    }

    return this;
  } else {
    bool accepted = false;
    accepted = grid->add(point.position());

    // TODO_LG If the point fits in this cell, it is added, otherwise check
    // cells with a higher level (i.e. smaller cells)
    if (accepted) {
      cache.push_point(point);
      acceptedAABB.update(point.position());
      numAccepted++;

      return this;
    } else {
      // try adding point to higher level

      if (potreeWriter->maxDepth != -1 && level >= potreeWriter->maxDepth) {
        return NULL;
      }

      int childIndex = nodeIndex(aabb, point.position());
      if (childIndex >= 0) {
        if (isLeafNode()) {  // TODO Redundant check because of line 205?
          children.resize(8, NULL);
        }
        PWNode *child = children[childIndex];

        // create child node if not existent
        if (child == NULL) {
          child = createChild(childIndex);
        }

        return child->add(point);
        // child->add(point, targetLevel);
      } else {
        return NULL;
      }
    }
    return NULL;
  }
}

void PWNode::flush() {
  if (isLeafNode()) {
    if (addCalledSinceLastFlush) {
      // Leaf node, can't transform inplace
      flushPoints(store, pntsPathAbsolute(), potreeWriter->pointAttributes,
                  _transformHelper, false, false);
      writeTilesetJSON(jsonPathAbsolute(), tileset);

      // if(store.size() != this->numAccepted){
      //	cout << "store " << store.size() << " != " << this->numAccepted
      //<< " - " << this->name() << endl;
      //}
    } else if (!addCalledSinceLastFlush && isInMemory) {
      store.clear();

      isInMemory = false;
    }
  } else {
    if (addCalledSinceLastFlush) {
      // Interior node, cache is cleared after write so we can transform inplace
      flushPoints(cache, pntsPathAbsolute(), potreeWriter->pointAttributes,
                  _transformHelper, true, true);
      writeTilesetJSON(jsonPathAbsolute(), tileset);
      // if(cache.size() != this->numAccepted){
      //	cout << "cache " << cache.size() << " != " << this->numAccepted
      //<< " - " << this->name() << endl;
      //
      //	exit(1);
      //}
      cache.clear();
    } else if (!addCalledSinceLastFlush && isInMemory) {
      delete grid;
      grid = new SparseGrid(aabb, spacing());
      isInMemory = false;
    }
  }

  addCalledSinceLastFlush = false;

  for (PWNode *child : children) {
    if (child != NULL) {
      child->flush();
    }
  }
}

vector<PWNode *> PWNode::getHierarchy(int levels) {
  vector<PWNode *> hierarchy;

  list<PWNode *> stack;
  stack.push_back(this);
  while (!stack.empty()) {
    PWNode *node = stack.front();
    stack.pop_front();

    if (node->level >= this->level + levels) {
      break;
    }
    hierarchy.push_back(node);

    for (PWNode *child : node->children) {
      if (child != NULL) {
        stack.push_back(child);
      }
    }
  }

  return hierarchy;
}

void PWNode::traverse(std::function<void(PWNode *)> callback) {
  callback(this);

  for (PWNode *child : this->children) {
    if (child != NULL) {
      child->traverse(callback);
    }
  }
}

void PWNode::traverseBreadthFirst(std::function<void(PWNode *)> callback) {
  // https://en.wikipedia.org/wiki/Iterative_deepening_depth-first_search

  int currentLevel = 0;
  int visitedAtLevel = 0;

  do {
    // doing depth first search until node->level = curentLevel
    stack<PWNode *> st;
    st.push(this);
    while (!st.empty()) {
      PWNode *node = st.top();
      st.pop();

      if (node->level == currentLevel) {
        callback(node);
        visitedAtLevel++;
      } else if (node->level < currentLevel) {
        for (PWNode *child : node->children) {
          if (child != NULL) {
            st.push(child);
          }
        }
      }
    }

    currentLevel++;

  } while (visitedAtLevel > 0);
}

PWNode *PWNode::findNode(string name) {
  string thisName = this->name();

  if (name.size() == thisName.size()) {
    return (name == thisName) ? this : NULL;
  } else if (name.size() > thisName.size()) {
    int childIndex = stoi(string(1, name[thisName.size()]));
    if (!isLeafNode() && children[childIndex] != NULL) {
      return children[childIndex]->findNode(name);
    } else {
      return NULL;
    }
  } else {
    return NULL;
  }
}

PotreeWriter::PotreeWriter(string workDir, ConversionQuality quality,
                           const SRSTransformHelper &transform)
    : _transform(transform) {
  this->workDir = workDir;
  this->quality = quality;
}

PotreeWriter::PotreeWriter(string workDir, AABB aabb, float spacing,
                           int maxDepth, double scale,
                           OutputFormat outputFormat,
                           PointAttributes pointAttributes,
                           ConversionQuality quality,
                           const SRSTransformHelper &transform)
    : _transform(transform) {
  this->workDir = workDir;
  this->aabb = aabb;
  this->spacing = spacing;
  this->scale = scale;
  this->maxDepth = maxDepth;
  this->outputFormat = outputFormat;
  this->quality = quality;

  this->pointAttributes = pointAttributes;

  if (this->scale == 0) {
    if (aabb.size.length() > 1'000'000) {
      this->scale = 0.01;
    } else if (aabb.size.length() > 100'000) {
      this->scale = 0.001;
    } else if (aabb.size.length() > 1) {
      this->scale = 0.001;
    } else {
      this->scale = 0.0001;
    }
  }

  root = new PWNode(this, aabb, transform);
}

string PotreeWriter::getExtension() {
  if (outputFormat == OutputFormat::LAS) {
    return ".las";
  } else if (outputFormat == OutputFormat::LAZ) {
    return ".laz";
  } else if (outputFormat == OutputFormat::BINARY) {
    return ".bin";
  }

  return "";
}

void PotreeWriter::waitUntilProcessed() {
  if (storeThread.joinable()) {
    storeThread.join();
  }
}

void PotreeWriter::add(const PointBuffer &points) {
  store.append_buffer(points);

  if (store.count() > 10'000) {
    processStore();
  }
}

/*

*/
void PotreeWriter::processStore() {
  auto movedStore = std::move(store);
  store = {};

  waitUntilProcessed();

  storeThread = thread([this, pointsStore = std::move(movedStore)] {
    for (auto point : pointsStore) {
      PWNode *acceptedBy = root->add(point);
      if (acceptedBy) {
        tightAABB.update(point.position());

        pointsInMemory++;
        numAccepted++;
      }
    }
  });
}

void PotreeWriter::flush() {
  processStore();

  if (storeThread.joinable()) {
    storeThread.join();
  }

  root->flush();

  root->traverse([](PWNode *node) { node->addedSinceLastFlush = false; });
}

}  // namespace Potree
