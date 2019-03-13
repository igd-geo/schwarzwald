

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
#include "PointAttributes.hpp"
#include "PointReader.h"
#include "PointWriter.hpp"
#include "PotreeException.h"
#include "SparseGrid.h"
#include "stuff.h"

#include "PotreeWriter.h"
#include "TileSetWriter.h"

using std::ifstream;
using std::stack;
using std::stringstream;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace fs = std::experimental::filesystem;

namespace Potree {

PWNode::PWNode(PotreeWriter *potreeWriter, AABB aabb,
               const SRSTransformHelper &transformHelper)
    : tileset(""), _transformHelper(transformHelper) {
  this->potreeWriter = potreeWriter;
  this->aabb = aabb;
  this->grid = new SparseGrid(aabb, spacing());

  // this->tileset.content_url = hierarchyPath() + name() + ".pnt";
  tileset.content_url = name() + ".pnt";
  tileset.boundingVolume = boundingVolumeFromAABB(aabb, transformHelper);
  tileset.geometricError = spacing() * 50;
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
  tileset.content_url = name() + ".pnt";
  tileset.boundingVolume = boundingVolumeFromAABB(aabb, transformHelper);
  tileset.geometricError = spacing() * 50;
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

string PWNode::workDir() { return potreeWriter->workDir; }

string PWNode::hierarchyPath() {
  string path = "r/";
  /*
  int hierarchyStepSize = potreeWriter->hierarchyStepSize;
  string indices = name().substr(1);

  int numParts = (int)floor((float)indices.size() / (float)hierarchyStepSize);
  for(int i = 0; i < numParts; i++){
          path += indices.substr(i * hierarchyStepSize, hierarchyStepSize) +
  "/";
  }
  */

  return path;
}

string PWNode::path() {
  // string path = hierarchyPath() + name() + potreeWriter->getExtension();
  string path = hierarchyPath() + name() + ".json";
  return path;
}

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

PointWriter *PWNode::createWriter(string path) {
  return new TileSetWriter(path, aabb, potreeWriter->scale,
                           this->potreeWriter->pointAttributes);
}

void PWNode::loadFromDisk() {
  PointReader *reader = createReader(workDir() + "/data/" + path());

  while (true) {
    const auto pointBatch = reader->readPointBatch();
    if (pointBatch.empty()) break;

    if (isLeafNode()) {
      store.append_buffer(pointBatch);
    } else {
      for (auto &position : pointBatch.positions()) {
        grid->addWithoutCheck(position);
      }
    }
  }

  grid->numAccepted = numAccepted;
  reader->close();
  delete reader;

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
  // child->tileset.url = hierarchyPath() + name() + std::to_string(childIndex)
  // + ".json";
  child->tileset.url = name() + std::to_string(childIndex) + ".json";

  child->tileset.geometricError = this->tileset.geometricError / 2;

  // child->tileset.content_url = hierarchyPath() + name() +
  // std::to_string(childIndex) + ".pnt"; //TODO: set content url + set it up
  // for root
  child->tileset.content_url = name() + std::to_string(childIndex) + ".pnt";

  if (child->tileset.geometricError < 0) {
    child->tileset.geometricError = 0;
  }

  this->tileset.children.push_back(&child->tileset);

  return child;
}

void PWNode::split() {  // Update tileset.json children ??
  children.resize(8, NULL);

  string filepath = workDir() + "/data/" + path();
  if (fs::exists(filepath)) {
    fs::remove(filepath);
  }

  for (auto point : store) {
    add(point);
  }

  store.clear();
}

PWNode *PWNode::add(PointBuffer::PointReference point) {
  // TODO_LG This adds a single point to a node in the octree
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
  // TODO Refactor this lambda into a separate function
  std::function<void(PointBuffer & points, bool append, bool transformInplace)>
      writeToDisk = [&](PointBuffer &points, bool append,
                        bool transformInplace) {
        string filepath = workDir() + "/data/" + path();
        PointWriter *writer = NULL;

        if (!fs::exists(workDir() + "/data/" + hierarchyPath())) {
          fs::create_directories(workDir() + "/data/" + hierarchyPath());
        }

        if (append) {
          string temppath = workDir() + "/temp/prepend" +
                            potreeWriter->getExtension();  // change ??
          if (fs::exists(filepath)) {
            fs::rename(fs::path(filepath), fs::path(temppath));
          }

          writer = createWriter(filepath);
          if (fs::exists(temppath)) {
            PointReader *reader = createReader(temppath);
            while (true) {
              auto pointBatch = reader->readPointBatch();
              if (pointBatch.empty()) break;
              writer->writePoints(pointBatch);
            }
            reader->close();
            delete reader;
            fs::remove(temppath);
          }
        } else {
          if (fs::exists(filepath)) {
            fs::remove(filepath);
          }
          writer = createWriter(filepath);
        }

        Vector3<double> localOffsetToWorld;
        if (transformInplace) {
          _transformHelper.transformPositionsTo(
              TargetSRS::CesiumWorld, gsl::make_span(points.positions()));
          localOffsetToWorld = setOriginToSmallestPoint(points.positions());
          writer->writePoints(points);
        } else {
          // TODO Figure out a way to get rid of this copy
          auto pointsCopy = points;
          _transformHelper.transformPositionsTo(
              TargetSRS::CesiumWorld, gsl::make_span(pointsCopy.positions()));
          localOffsetToWorld = setOriginToSmallestPoint(pointsCopy.positions());
          writer->writePoints(pointsCopy);
        }

        // TODO Create the Tileset description for this node. This includes the
        // tile transform, based on 'localOffsetToWorld' as well as the parents´
        // offsets. 3d-tiles will chain transformations, so we have to take the
        // parent transformation into account

        this->tileset.localToWorldOffset = localOffsetToWorld;
        if (this->parent) {
          localOffsetToWorld -= this->parent->tileset.localToWorldOffset;
        }

        this->tileset.tileTransform = glm::translate(
            glm::identity<glm::dmat4>(),
            {localOffsetToWorld.x, localOffsetToWorld.y, localOffsetToWorld.z});

        writer->writeTileset(filepath, this->tileset);

        /*
        if(append && (writer->numPoints != this->numAccepted)){ // !!
                cout << "writeToDisk " << writer->numPoints  << " != " <<
        this->numAccepted << endl; exit(1);
        }
        */

        writer->close();
      };

  if (isLeafNode()) {
    if (addCalledSinceLastFlush) {
      // Leaf node, can't transform inplace
      writeToDisk(store, false, false);

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
      writeToDisk(cache, true, true);
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

  cloudjs.outputFormat = outputFormat;
  cloudjs.boundingBox = aabb;
  cloudjs.octreeDir = "data";
  cloudjs.spacing = spacing;
  cloudjs.version = "1.7";
  cloudjs.scale = this->scale;
  cloudjs.pointAttributes = pointAttributes;

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
  if (numAdded == 0) {
    fs::path dataDir(workDir + "/data");
    fs::path tempDir(workDir + "/temp");

    fs::create_directories(dataDir);
    fs::create_directories(tempDir);
  }

  store.append_buffer(points);
  numAdded++;

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

  // auto start = high_resolution_clock::now();

  root->flush();

  // auto end = high_resolution_clock::now();
  // long long duration = duration_cast<milliseconds>(end-start).count();
  // float seconds = duration / 1'000.0f;
  // cout << "flush nodes: " << seconds << "s" << endl;

  {  // update cloud.js
    cloudjs.hierarchy = vector<CloudJS::Node>();
    cloudjs.hierarchyStepSize = hierarchyStepSize;
    cloudjs.tightBoundingBox = tightAABB;
    cloudjs.numAccepted = numAccepted;
    cloudjs.projection = projection;

    ofstream cloudOut(workDir + "/cloud.js", ios::out);
    cloudOut << cloudjs.getString();
    cloudOut.close();
  }

  {  // write hierarchy
    // auto start = high_resolution_clock::now();

    int hrcTotal = 0;
    int hrcFlushed = 0;

    list<PWNode *> stack;
    stack.push_back(root);
    while (!stack.empty()) {
      PWNode *node = stack.front();
      stack.pop_front();

      hrcTotal++;

      vector<PWNode *> hierarchy = node->getHierarchy(hierarchyStepSize + 1);
      bool needsFlush = false;
      for (const auto &descendant : hierarchy) {
        if (descendant->level == node->level + hierarchyStepSize) {
          stack.push_back(descendant);
        }

        needsFlush = needsFlush || descendant->addedSinceLastFlush;
      }

      if (needsFlush) {
        string dest = workDir + "/data/" + node->hierarchyPath() + "/" +
                      node->name() + ".hrc";
        ofstream fout;
        fout.open(dest, ios::out | ios::binary);

        for (const auto &descendant : hierarchy) {
          char children = 0;
          for (int j = 0; j < (int)descendant->children.size(); j++) {
            if (descendant->children[j] != NULL) {
              children = children | (1 << j);
            }
          }

          fout.write(reinterpret_cast<const char *>(&children), 1);
          fout.write(reinterpret_cast<const char *>(&(descendant->numAccepted)),
                     4);
        }

        fout.close();
        hrcFlushed++;
      }
    }

    root->traverse([](PWNode *node) { node->addedSinceLastFlush = false; });

    // cout << "hrcTotal: " << hrcTotal << "; " << "hrcFlushed: " << hrcFlushed
    // << endl;

    // auto end = high_resolution_clock::now();
    // long long duration = duration_cast<milliseconds>(end-start).count();
    // float seconds = duration / 1'000.0f;
    // cout << "writing hierarchy: " << seconds << "s" << endl;
  }
}

void PotreeWriter::setProjection(string projection) {
  this->projection = projection;
}

void PotreeWriter::loadStateFromDisk() {
  {  // cloudjs
    string cloudJSPath = workDir + "/cloud.js";
    ifstream file(cloudJSPath);
    string line;
    string content;
    while (std::getline(file, line)) {
      content += line + "\n";
    }
    cloudjs = CloudJS(content);
  }

  {
    this->outputFormat = cloudjs.outputFormat;
    this->pointAttributes = cloudjs.pointAttributes;
    this->hierarchyStepSize = cloudjs.hierarchyStepSize;
    this->spacing = cloudjs.spacing;
    this->scale = cloudjs.scale;
    this->aabb = cloudjs.boundingBox;
    this->numAccepted = cloudjs.numAccepted;
  }

  {  // tree
    vector<string> hrcPaths;
    fs::path rootDir(workDir + "/data/r");
    for (fs::recursive_directory_iterator iter(rootDir), end; iter != end;
         ++iter) {
      fs::path path = iter->path();
      if (fs::is_regular_file(path)) {
        if (iEndsWith(path.extension().string(), ".hrc")) {
          hrcPaths.push_back(path.string());
        } else {
        }
      } else if (fs::is_directory(path)) {
      }
    }
    std::sort(hrcPaths.begin(), hrcPaths.end(),
              [](string &a, string &b) { return a.size() < b.size(); });

    PWNode *root = new PWNode(this, cloudjs.boundingBox, _transform);
    for (string hrcPath : hrcPaths) {
      fs::path pHrcPath(hrcPath);
      string hrcName = pHrcPath.stem().string();
      PWNode *hrcRoot = root->findNode(hrcName);

      PWNode *current = hrcRoot;
      current->addedSinceLastFlush = false;
      current->isInMemory = false;
      vector<PWNode *> nodes;
      nodes.push_back(hrcRoot);

      ifstream fin(hrcPath, ios::in | ios::binary);
      std::vector<char> buffer((std::istreambuf_iterator<char>(fin)),
                               (std::istreambuf_iterator<char>()));

      for (int i = 0; 5 * i < (int)buffer.size(); i++) {
        PWNode *current = nodes[i];

        char children = buffer[i * 5];
        char *p = &buffer[i * 5 + 1];
        unsigned int *ip = reinterpret_cast<unsigned int *>(p);
        unsigned int numPoints = *ip;

        // std::bitset<8> bs(children);
        // cout << i << "\t: " << "children: " << bs << "; " << "numPoints: " <<
        // numPoints << endl;

        current->numAccepted = numPoints;

        if (children != 0) {
          current->children.resize(8, NULL);
          for (int j = 0; j < 8; j++) {
            if ((children & (1 << j)) != 0) {
              AABB cAABB = childAABB(current->aabb, j);
              PWNode *child =
                  new PWNode(this, j, cAABB, current->level + 1, _transform);
              child->parent = current;
              child->addedSinceLastFlush = false;
              child->isInMemory = false;
              current->children[j] = child;
              nodes.push_back(child);
            }
          }
        }
      }
    }

    this->root = root;

    // TODO set it to actual number
    this->numAdded = 1;

    // int numNodes = 0;
    // root->traverse([&](PWNode *node){
    //	if(numNodes < 50){
    //		cout << std::left << std::setw(10) << node->name();
    //		cout << std::right << std::setw(10) << node->numAccepted << ";
    //"; 		cout << node->aabb.min << " - " << node->aabb.max <<
    // endl;
    //	}
    //
    //	numNodes++;
    //
    //});
  }
}

}  // namespace Potree
