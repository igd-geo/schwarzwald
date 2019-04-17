#include "TileSetWriter.h"
#include "type_util.h"

#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

#include <fstream>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace rapidjson;

template <typename Alloc>
static Value writeBoundingVolume(const BoundingVolume_t& boundingVolume,
                                 Alloc& alloc) {
  Value boundingVolumeObj{kObjectType};
  Value boundingVolumeArr{kArrayType};

  const auto boundingVolumeAsArray = boundingVolumeToArray(boundingVolume);
  for (auto entry : boundingVolumeAsArray) {
    boundingVolumeArr.PushBack(entry, alloc);
  }

  std::visit(overloaded{[&](const BoundingRegion& br) {
               boundingVolumeObj.AddMember("region", boundingVolumeArr, alloc);
             }, 
            [&](const BoundingBox& bb) {
               boundingVolumeObj.AddMember("box", boundingVolumeArr, alloc);
             } },
             boundingVolume);

  return boundingVolumeObj;
}

bool Potree::writeTilesetJSON(const string& filepath, const Tileset& ts) {
  // std::cout << "Writing tileset JSON \"" << filepath << "\"..." << std::endl;

  Document document;
  document.SetObject();

  auto& alloc = document.GetAllocator();

  Value assetobj(kObjectType);
  Value propertiesobj(kObjectType);
  Value heightobj(kObjectType);
  Value rootobj(kObjectType);
  Value rootboundingvolumeobj(kObjectType);
  Value rootboundingvolumregionarr(kArrayType);

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
  Value version(ts.version.c_str(), (rapidjson::SizeType)ts.version.size());
  assetobj.AddMember("version", version,
                     alloc);  // defines the JSON schema for tileset.json and
                              // the base set of tile formats
  if (!ts.tilesetVersion.empty()) {
    Value v(ts.tilesetVersion.c_str(), (rapidjson::SizeType)ts.version.size());
    assetobj.AddMember("tilesetVersion", v, alloc);
  }
  if (ts.gltfUpAxis == X || ts.gltfUpAxis == Z)  // Y is deafult in schema
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
                     alloc);  // error when the entire tileset is not rendered

  /*
  -boundingVolume required
  -viewerRequestVolume
  -geometricError required
  -refine
  -transform
  -content required for leaf tiles
  -children ->with external Tilesets: must be undefined or an empty array
  */
  auto boundingVolumeObj = writeBoundingVolume(ts.boundingVolume, alloc);
  rootobj.AddMember("boundingVolume", boundingVolumeObj, alloc);

  // TODO root object geometric error should not be the same as tile geometric
  // error
  /*
    From the 3d-tiles spec:
    "root.geometricError is not the same as the tileset's top-level
    geometricError. The tileset's geometricError is used at runtime to determine
    the SSE at which the tileset's root tile renders; root.geometricError is
    used at runtime to determine the SSE at which the root tile's children are
    rendered."
  */
  rootobj.AddMember("geometricError", ts.geometricError, alloc);

  if (ts.writeRefine == true) {
    switch (ts.refine) {
      case ADD:
        rootobj.AddMember("refine", "ADD", alloc);
        break;
      case REFINE:
        rootobj.AddMember("refine", "REFINE", alloc);
        break;
    }
  }

  // Add transform

  // content
  Value contentObj(kObjectType);
  // optional: add bounding box for that object
  Value co_url(ts.content_url.c_str(),
               (rapidjson::SizeType)ts.content_url.size());
  contentObj.AddMember("url", co_url, alloc);

  rootobj.AddMember("content", contentObj, alloc);

  // Add up to 8 children with bounding voulume(box), geometricError and content
  // (url to pnt file)
  Value childrenArr(kArrayType);
  Value childObj(kObjectType);
  Value val;
  Value o(kObjectType);
  Value a(kArrayType);
  for (auto const& child : ts.children) {
    // Create Children Tile with boundingVolume geometric error and content-url
    // pointing to extrernal tileset
    Value box(kArrayType);

    auto childBoundingVolumeObj =
        writeBoundingVolume(child->boundingVolume, alloc);

    childObj.SetObject();
    assert(childObj.IsObject());

    childObj.AddMember("boundingVolume", childBoundingVolumeObj, alloc);

    childObj.AddMember("geometricError", child->geometricError, alloc);

    Value cObj(kObjectType);
    Value ch_url(child->url.c_str(), (rapidjson::SizeType)child->url.size());
    cObj.AddMember("url", ch_url, alloc);
    childObj.AddMember("content", cObj, alloc);

    childrenArr.PushBack(childObj, alloc);
  }

  rootobj.AddMember("children", childrenArr, alloc);

  document.AddMember("root", rootobj, alloc);

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

    explicit Stream(const std::string& filepath) : 
      of {filepath, std::ios::binary} {}

    typedef char Ch;
    void Put (Ch ch) {of.put (ch);}
    void Flush() {}
  };

  Stream fs{filepath};
  if(!fs.of.is_open()) {
    std::cerr << "Error writing tileset JSON to \"" << filepath << "\"" << std::endl;
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
