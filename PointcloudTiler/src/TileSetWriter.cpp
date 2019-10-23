#include "TileSetWriter.h"
#include "type_util.h"

#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

#include <experimental/filesystem>
#include <fstream>
namespace fs = std::experimental::filesystem;

using namespace rapidjson;

template <typename Alloc>
static Value writeBoundingVolume(const BoundingVolume_t &boundingVolume,
                                 Alloc &alloc) {
  Value boundingVolumeObj{kObjectType};
  Value boundingVolumeArr{kArrayType};

  const auto boundingVolumeAsArray = boundingVolumeToArray(boundingVolume);
  for (auto entry : boundingVolumeAsArray) {
    boundingVolumeArr.PushBack(entry, alloc);
  }

  std::visit(overloaded{[&](const BoundingRegion &br) {
                          boundingVolumeObj.AddMember("region",
                                                      boundingVolumeArr, alloc);
                        },
                        [&](const BoundingBox &bb) {
                          boundingVolumeObj.AddMember("box", boundingVolumeArr,
                                                      alloc);
                        }},
             boundingVolume);

  return boundingVolumeObj;
}

/**
 * Converts the given Tileset into JSON tree structure
 */
template <typename Alloc>
static Value write_tileset(const Tileset &tileset, Alloc &alloc) {
  Value root{kObjectType};

  auto boundingVolumeObj = writeBoundingVolume(tileset.boundingVolume, alloc);
  root.AddMember("boundingVolume", boundingVolumeObj, alloc);
  root.AddMember("geometricError", tileset.geometricError, alloc);
  root.AddMember("refine", "ADD", alloc);

  Value content{kObjectType};
  // optional: add bounding box for that object

  // HACK If the child_url is not set, the Tileset refers to an external
  // Tileset, in which case we have to write the content_url instead. We should
  // use the type system to guarantee this instead of this hack :)
  auto content_uri = [&]() {
    if (tileset.content_url.empty()) {
      return Value{tileset.child_url.c_str(),
                   static_cast<rapidjson::SizeType>(tileset.child_url.size())};
    }
    return Value{tileset.content_url.c_str(),
                 static_cast<rapidjson::SizeType>(tileset.content_url.size())};
  }();
  content.AddMember("uri", content_uri, alloc);
  root.AddMember("content", content, alloc);

  // Write children, if there are any
  if (tileset.children.empty())
    return root;

  Value children{kArrayType};
  for (auto &child : tileset.children) {
    children.PushBack(write_tileset(child, alloc), alloc);
  }
  root.AddMember("children", children, alloc);

  return root;
}

bool writeTilesetJSON(const std::string &filepath, const Tileset &ts) {
  // std::cout << "Writing tileset JSON \"" << filepath << "\"..." << std::endl;

  Document document;
  document.SetObject();

  auto &alloc = document.GetAllocator();

  // Tileset
  // https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/tileset.schema.json
  /*
  -asset required
  -properties
  -geometricError required
  -root required
  */

  // asset
  // https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/asset.schema.json
  /*
  Metadata about the entire Tileset
  -version required
  -tilsetVersion
  -gltUpAxis
  */
  Value assetobj(kObjectType);
  Value version(ts.version.c_str(), (rapidjson::SizeType)ts.version.size());
  assetobj.AddMember("version", version,
                     alloc); // defines the JSON schema for tileset.json and
                             // the base set of tile formats
  if (!ts.tilesetVersion.empty()) {
    Value v(ts.tilesetVersion.c_str(), (rapidjson::SizeType)ts.version.size());
    assetobj.AddMember("tilesetVersion", v, alloc);
  }
  if (ts.gltfUpAxis == X || ts.gltfUpAxis == Z) // Y is deafult in schema
  {
    if (ts.gltfUpAxis == X) {
      assetobj.AddMember("gltUpAxis", "X", alloc);
    }
    if (ts.gltfUpAxis == Z) {
      assetobj.AddMember("gltUpAxis", "Z", alloc);
    }
  }

  document.AddMember("asset", assetobj, alloc);

  // properties
  // https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/properties.schema.json
  /*
  A dictionary object of metadata about per-feature properties.
  -maximum required
  -minimum required
  */
  if (ts.height_max != 0 && ts.height_min != 0) {
    Value propertiesobj(kObjectType);
    Value heightobj(kObjectType);
    heightobj.AddMember("minimum", ts.height_min, alloc);
    heightobj.AddMember("maximum", ts.height_max, alloc);
    propertiesobj.AddMember("Height", heightobj, alloc);

    document.AddMember("properties", propertiesobj, alloc);
  }

  // geometricError
  /*
  The error, in meters, introduced if this tileset is not rendered.
  At runtime, the geometric error is used to compute screen space error (SSE),
  i.e., the error measured in pixels. minimum = 0
  */
  document.AddMember("geometricError", ts.geometricError,
                     alloc); // error when the entire tileset is not rendered

  auto root_tileset = write_tileset(ts, alloc);
  document.AddMember("root", root_tileset, alloc);

  if (fs::exists(filepath)) {
    std::error_code removeErrorCode;
    auto success = fs::remove(filepath, removeErrorCode);
    if (!success) {
      std::cerr << "Could not remove file \"" << filepath << "\" ("
                << removeErrorCode.message() << ")" << std::endl;
      return false;
    }
  }

  /*
  FILE* filePtr;
  auto fopenErr = fopen_s(&filePtr, filepath.c_str(), "wb");
  if (fopenErr) {
    std::cerr << "Error writing tileset JSON to \"" << filepath << "\" ("
              << strerror(fopenErr) << ")" << std::endl;
    return false;
  }

  char writeBuffer[65536];

  FileWriteStream os(filePtr, writeBuffer, sizeof(writeBuffer));
  */

  struct Stream {
    std::ofstream of;

    explicit Stream(const std::string &filepath)
        : of{filepath, std::ios::binary} {}

    typedef char Ch;
    void Put(Ch ch) { of.put(ch); }
    void Flush() {}
  };

  Stream fs{filepath};
  if (!fs.of.is_open()) {
    std::cerr << "Error writing tileset JSON to \"" << filepath << "\""
              << std::endl;
    return false;
  }

  Writer<Stream> writer(fs);
  document.Accept(writer);

  /*
  auto fcloseErr = fclose(filePtr);
  if (fcloseErr) {
    std::cerr << "Error " << fcloseErr << " while closing file handle for \""
              << filepath << "\"" << std::endl;
  }
  */

  return true;
}
