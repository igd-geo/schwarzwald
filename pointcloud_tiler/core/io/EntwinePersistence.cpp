#include "io/EntwinePersistence.h"

#include "datastructures/DynamicMortonIndex.h"
#include "datastructures/OctreeNodeIndex.h"
#include "util/Error.h"
#include "util/stuff.h"

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>

namespace rj = rapidjson;

static std::string
extension_from_entwine_format(EntwineFormat format)
{
  switch (format) {
    case EntwineFormat::LAS:
      return ".las";
    case EntwineFormat::LAZ:
      return ".laz";
    // case EntwineFormat::Binary:
    //   return ".bin";
    default:
      throw std::runtime_error{ concat("Unrecognized EntwineFormat: ",
                                       static_cast<uint32_t>(format)) };
  }
}

static void
create_ept_folder_structure(const fs::path& root_dir)
{
  if (!fs::exists(root_dir)) {
    if (!fs::create_directories(root_dir)) {
      throw std::runtime_error{ concat("Could not create source directory: ",
                                       root_dir.string()) };
    }
  }

  if (!fs::create_directory(root_dir / "ept-data")) {
    throw std::runtime_error{ "Could not create ept-data directory" };
  }
  if (!fs::create_directory(root_dir / "ept-hierarchy")) {
    throw std::runtime_error{ "Could not create ept-hierarchy directory" };
  }
  if (!fs::create_directory(root_dir / "ept-sources")) {
    throw std::runtime_error{ "Could not create ept-sources directory" };
  }
}

static void
create_hierarchy_files(const fs::path& root_dir,
                       const std::unordered_map<std::string, size_t>& hierarchy)
{
  // Split large tree into subtrees of defined depth
  constexpr uint32_t SPLIT_DEPTH = 5;

  using Hierarchy = std::unordered_map<OctreeNodeIndex64, int64_t>;

  std::unordered_map<OctreeNodeIndex64, Hierarchy> split_hierarchies;

  const auto get_parent_index_in_hierarchy =
    [](const OctreeNodeIndex64& index) -> OctreeNodeIndex64 {
    auto parent_index = index;
    while (parent_index.levels() % SPLIT_DEPTH != 0) {
      parent_index = parent_index.parent();
    }
    return parent_index;
  };

  const auto write_hierarchy_json = [&root_dir](const OctreeNodeIndex64& parent,
                                                const Hierarchy& hierarchy) {
    rj::Document document;
    auto& allocator = document.GetAllocator();

    document.SetObject();

    for (auto& kv : hierarchy) {
      const auto entry_name = OctreeNodeIndex64::to_string(
        kv.first, MortonIndexNamingConvention::Entwine);
      rj::Value entry_name_value{ entry_name.c_str(),
                                  static_cast<rj::SizeType>(entry_name.size()),
                                  allocator };
      document.AddMember(entry_name_value, kv.second, allocator);
    }

    const auto file_path =
      concat(root_dir.string(),
             "/ept-hierarchy/",
             OctreeNodeIndex64::to_string(parent,
                                          MortonIndexNamingConvention::Entwine),
             ".json");

    try {
      write_json_to_file(document, file_path);
    } catch (const std::exception& ex) {
      throw util::chain_error(ex, "Could not write hierarchy file");
    }
  };

  for (auto& kv : hierarchy) {
    const auto node_index = OctreeNodeIndex64::from_string(
                              kv.first, MortonIndexNamingConvention::Entwine)
                              .value();
    const auto parent = get_parent_index_in_hierarchy(node_index);

    auto hierarchy_iter = split_hierarchies.find(parent);
    if (hierarchy_iter == std::end(split_hierarchies)) {
      // This is a new parent node of a subtree. Mark this parent in ITS parent
      // subtree
      auto parent_of_parent = parent;
      while (parent_of_parent.levels() > 0) {
        const auto new_parent_of_parent =
          get_parent_index_in_hierarchy(parent_of_parent.parent());
        split_hierarchies[new_parent_of_parent][parent_of_parent] = -1;
        parent_of_parent = new_parent_of_parent;
      }
    }

    split_hierarchies[parent][node_index] = static_cast<int64_t>(kv.second);
  }

  for (auto& kv : split_hierarchies) {
    write_hierarchy_json(kv.first, kv.second);
  }
}

std::vector<EptSchemaEntry>
point_attributes_to_ept_schema(const PointAttributes& point_attributes)
{
  std::vector<EptSchemaEntry> schema;
  // TECH_DEBT Refactor
  for (auto& attribute : point_attributes) {
    switch (attribute) {
      case PointAttribute::Classification:
        schema.push_back(
          { "Classification", std::nullopt, std::nullopt, 1, "unsigned" });
        break;
      case PointAttribute::EdgeOfFlightLine:
        schema.push_back(
          { "EdgeOfFlightLine", std::nullopt, std::nullopt, 1, "unsigned" });
        break;
      case PointAttribute::GPSTime:
        schema.push_back({ "GpsTime", std::nullopt, std::nullopt, 8, "float" });
        break;
      case PointAttribute::Intensity:
        schema.push_back(
          { "Intensity", std::nullopt, std::nullopt, 2, "unsigned" });
        break;
      case PointAttribute::Normal:
        schema.push_back({ "NX", std::nullopt, std::nullopt, 4, "float" });
        schema.push_back({ "NY", std::nullopt, std::nullopt, 4, "float" });
        schema.push_back({ "NZ", std::nullopt, std::nullopt, 4, "float" });
        break;
      case PointAttribute::NumberOfReturns:
        schema.push_back(
          { "NumberOfReturns", std::nullopt, std::nullopt, 1, "unsigned" });
        break;
      case PointAttribute::PointSourceID:
        schema.push_back(
          { "PointSourceID", std::nullopt, std::nullopt, 2, "unsigned" });
        break;
      case PointAttribute::Position:
        // TODO Offset and scale, where do they come from if we have multiple
        // source files?
        schema.push_back({ "X", 0, 1, 4, "signed" });
        schema.push_back({ "Y", 0, 1, 4, "signed" });
        schema.push_back({ "Z", 0, 1, 4, "signed" });
        break;
      case PointAttribute::ReturnNumber:
        schema.push_back(
          { "ReturnNumber", std::nullopt, std::nullopt, 1, "unsigned" });
        break;
      case PointAttribute::RGB:
      case PointAttribute::RGBFromIntensity:
        schema.push_back({ "Red", std::nullopt, std::nullopt, 2, "unsigned" });
        schema.push_back(
          { "Green", std::nullopt, std::nullopt, 2, "unsigned" });
        schema.push_back({ "Blue", std::nullopt, std::nullopt, 2, "unsigned" });
        break;
      case PointAttribute::ScanAngleRank:
        // TODO Entwine classifies ScanAngleRank as 4-byte float, in LAS spec it
        // is 1 byte char...
        schema.push_back(
          { "ScanAngleRank", std::nullopt, std::nullopt, 1, "signed" });
        break;
      case PointAttribute::ScanDirectionFlag:
        schema.push_back(
          { "ScanDirectionFlag", std::nullopt, std::nullopt, 1, "unsigned" });
        break;
      case PointAttribute::UserData:
        schema.push_back(
          { "UserData", std::nullopt, std::nullopt, 1, "unsigned" });
        break;
      default:
        throw std::runtime_error{
          "Unhandled PointAttribute in switch statement"
        };
    }
  }

  return schema;
}

void
write_ept_json(const fs::path& file_path, const EptJson& ept_json)
{
  rj::Document document;
  auto& allocator = document.GetAllocator();

  document.SetObject();

  rj::Value bounds_member{ rj::kArrayType };
  bounds_member.PushBack(ept_json.bounds.min.x, allocator);
  bounds_member.PushBack(ept_json.bounds.min.y, allocator);
  bounds_member.PushBack(ept_json.bounds.min.z, allocator);
  bounds_member.PushBack(ept_json.bounds.max.x, allocator);
  bounds_member.PushBack(ept_json.bounds.max.y, allocator);
  bounds_member.PushBack(ept_json.bounds.max.z, allocator);
  document.AddMember("bounds", bounds_member, allocator);

  // TODO What are conforming bounds? The bounds prior to being a cube?
  rj::Value conforming_bounds_member{ rj::kArrayType };
  conforming_bounds_member.PushBack(ept_json.conforming_bounds.min.x,
                                    allocator);
  conforming_bounds_member.PushBack(ept_json.conforming_bounds.min.y,
                                    allocator);
  conforming_bounds_member.PushBack(ept_json.conforming_bounds.min.z,
                                    allocator);
  conforming_bounds_member.PushBack(ept_json.conforming_bounds.max.x,
                                    allocator);
  conforming_bounds_member.PushBack(ept_json.conforming_bounds.max.y,
                                    allocator);
  conforming_bounds_member.PushBack(ept_json.conforming_bounds.max.z,
                                    allocator);
  document.AddMember("boundsConforming", conforming_bounds_member, allocator);

  switch (ept_json.data_type) {
    case EntwineFormat::LAS:
      document.AddMember("dataType", "las", allocator);
      break;
    case EntwineFormat::LAZ:
      document.AddMember("dataType", "laszip", allocator);
      break;
    default:
      throw std::runtime_error{
        "Unhandled enum constant for enum EntwineFormat"
      };
  }

  document.AddMember("hierarchyType", "json", allocator);
  document.AddMember("points", ept_json.points, allocator);

  rj::Value schema_member{ rj::kArrayType };
  for (const auto& schema_entry : ept_json.schema) {
    rj::Value schema_member_entry{ rj::kObjectType };
    schema_member_entry.AddMember(
      "name", rj_string(schema_entry.name, allocator), allocator);
    schema_member_entry.AddMember("size", schema_entry.size, allocator);
    schema_member_entry.AddMember(
      "type", rj_string(schema_entry.type, allocator), allocator);
    if (schema_entry.offset) {
      schema_member_entry.AddMember("offset", *schema_entry.offset, allocator);
    }
    if (schema_entry.scale) {
      schema_member_entry.AddMember("scale", *schema_entry.scale, allocator);
    }

    schema_member.PushBack(schema_member_entry, allocator);
  }
  document.AddMember("schema", schema_member, allocator);

  document.AddMember("span", ept_json.span, allocator);

  rj::Value srs_member{ rj::kObjectType };
  srs_member.AddMember(
    "authority", rj_string(ept_json.srs.authority, allocator), allocator);
  srs_member.AddMember(
    "horizontal", rj_string(ept_json.srs.horizontal, allocator), allocator);
  srs_member.AddMember(
    "wkt", rj_string(ept_json.srs.wkt, allocator), allocator);
  document.AddMember("srs", srs_member, allocator);

  document.AddMember(
    "version", rj_string(ept_json.version, allocator), allocator);

  try {
    write_json_to_file(document, file_path);
  } catch (const std::exception& ex) {
    throw util::chain_error(ex, "Could not write ept.json file");
  }
}

EntwinePersistence::EntwinePersistence(const std::string& work_dir,
                                       const PointAttributes& point_attributes,
                                       EntwineFormat format)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _format(format)
  , _file_extension(extension_from_entwine_format(format))
  , _las_persistence(concat(work_dir, "/ept-data"),
                     point_attributes,
                     format == EntwineFormat::LAZ ? Compressed::Yes
                                                  : Compressed::No)
  , _hierarchy_lock(std::make_unique<std::mutex>())
{
  create_ept_folder_structure(work_dir);
}

EntwinePersistence::~EntwinePersistence()
{
  create_hierarchy_files(_work_dir, _hierarchy);
}

void
EntwinePersistence::persist_points(PointBuffer const& points,
                                   const AABB& bounds,
                                   const std::string& node_name)
{
  if (!points.count())
    return;

  const auto entwine_name = potree_name_to_entwine_name(node_name);

  _las_persistence.persist_points(points, bounds, entwine_name);

  std::lock_guard guard{ *_hierarchy_lock };
  _hierarchy[entwine_name] = points.count();
}

void
EntwinePersistence::retrieve_points(const std::string& node_name,
                                    PointBuffer& points)
{
  const auto entwine_name = potree_name_to_entwine_name(node_name);

  _las_persistence.retrieve_points(entwine_name, points);
}

bool
EntwinePersistence::node_exists(const std::string& node_name) const
{
  const auto entwine_name = potree_name_to_entwine_name(node_name);

  const auto file_path =
    concat(_work_dir.string(), "/ept-data/", entwine_name, _file_extension);
  return fs::exists(file_path);
}

std::string
EntwinePersistence::potree_name_to_entwine_name(const std::string& potree_name)
{
  return DynamicMortonIndex::parse_string(potree_name,
                                          MortonIndexNamingConvention::Potree)
    .map([](const DynamicMortonIndex& idx) {
      return to_string(idx, MortonIndexNamingConvention::Entwine);
    })
    .value();
}
