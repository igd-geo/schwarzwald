#ifndef TILESET_H
#define TILESET_H

#include <functional>
#include <list>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <glm/matrix.hpp>

#include "PointAttributes.hpp"
#include "SparseGrid.h"
#include "Transformation.h"
#include "Vector3.h"

#include "AABB.h"

using Potree::AABB;
using Potree::Vector3;
using std::string;
using std::stringstream;
using std::vector;

enum GltAxis { X, Y, Z };
enum Refine { ADD, REFINE };

struct BoundingRegion {
  double west, south, east, north, minHeight, maxHeight;
};

struct BoundingBox {
    double cx, cy, cz;
    double xx, xy, xz;
    double yx, yy, yz;
    double zx, zy, zz;
};
// TODO Other bounding regions (box, sphere)

using BoundingVolume_t = std::variant<BoundingRegion, BoundingBox>;

/// <summary>
/// Creates a 3D tiles bounding volume from the given AABB. For coordinate
/// system transformations, the given SRSTransformHelper is used. The returned
/// bounding volume will always be in WGS84 coordinates as defined by
/// https://github.com/AnalyticalGraphicsInc/3d-tiles/tree/master/specification#bounding-volumes
/// </summary>
BoundingVolume_t boundingVolumeFromAABB(
    const AABB& aabb, const Potree::SRSTransformHelper& transformHelper);

/// <summary>
/// Converts the bounding volume to an array representation which contains all
/// relevant parameters of the bounding volume
/// </summary>
std::vector<double> boundingVolumeToArray(
    const BoundingVolume_t& boundingVolume);

class Tileset {
 public:
  string url = "";   // url of this Tileset e.g r/tileset.json
  string name = "";  // e.g tileset.json

  string version = "0.0";
  string tilesetVersion = "";  // not required
  GltAxis gltfUpAxis = Y;

  // properties
  double height_min = 0;
  double height_max = 0;

  // geometricError - required
  double geometricError =
      500;  // This should be set up and be less or eq in the child tilesets

  // root - required
  BoundingVolume_t boundingVolume;

  Refine refine = ADD;
  bool writeRefine = true;

  string content_url;  // Refers to the pnt-file

  vector<Tileset*> children;  // Get aabb geometricError and url

  string child_url;  // Points to another Tileset File

  Vector3<double> localCenter;

  Tileset(string name) : name(std::move(name)) {}

  void setRequestVolume(const BoundingVolume_t& rv) {
    _viewerRequestVolume = rv;
    _requestVolumeSet = true;
  }

  const auto& getRequestVolume() const { return _viewerRequestVolume; }
  bool requestVolumeIsSet() const { return _requestVolumeSet; }

 private:
  BoundingVolume_t
      _viewerRequestVolume;  // Viewer must be inside of it before the tiles
                             // content will be refined based on geometric error
  bool _requestVolumeSet = false;
};

#endif
