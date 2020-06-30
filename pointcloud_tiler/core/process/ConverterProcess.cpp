#include "process/ConverterProcess.h"

#include "datastructures/DynamicMortonIndex.h"
#include "io/Cesium3DTilesPersistence.h"
#include "io/PNTSWriter.h"
#include "io/PointsPersistence.h"
#include "io/TileSetWriter.h"
#include "math/AABB.h"
#include "pointcloud/Tileset.h"
#include "tiling/OctreeAlgorithms.h"
#include "util/Transformation.h"
#include "util/stuff.h"
#include <terminal/TerminalUI.h>
#include <terminal/stdout_helper.h>
#include <threading/TaskSystem.h>

#include <array>
#include <boost/format.hpp>
#include <boost/scope_exit.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <thread>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/filereadstream.h>

namespace rj = rapidjson;

constexpr float spacing_correction_factor = 2;

// function that parses a node name and returns the corresponding Morton index
// for that node
using NodeNameToMortonIndex_t =
  std::function<tl::expected<DynamicMortonIndex, std::string>(
    const std::string&)>;

struct Properties
{
  AABB root_bounds;
  float root_spacing;
  bool points_have_offset;
  NodeNameToMortonIndex_t morton_index_parser;
};

enum class DeleteSource
{
  No,
  Yes
};

static tl::expected<Properties, std::string>
parse_properties_json(const std::string& properties_json_path)
{
  auto fp = fopen(properties_json_path.c_str(), "r");
  if (!fp) {
    const auto error_msg =
      boost::format("Can't open properties.json file @ \"%1%\"") %
      properties_json_path;
    return tl::make_unexpected(error_msg.str());
  }

  BOOST_SCOPE_EXIT(&fp) { fclose(fp); }
  BOOST_SCOPE_EXIT_END

  char buf[65536];
  rj::FileReadStream stream{ fp, buf, sizeof(buf) };

  rj::Document document;
  if (document.ParseStream(stream).HasParseError()) {
    const auto parse_error_msg =
      rapidjson::GetParseError_En(document.GetParseError());
    const auto error_msg =
      boost::format("Can't parse properties.json file [%1%]") % parse_error_msg;
    return tl::make_unexpected(error_msg.str());
  }

  Properties props;

  props.root_spacing = static_cast<float>(
    document["source_properties"]["root_spacing"].GetDouble());

  const auto& bounds_member = document["source_properties"]["bounds"];
  const auto& bounds_min = bounds_member["min"];
  const auto& bounds_max = bounds_member["max"];

  props.root_bounds = AABB{ { bounds_min[0].GetDouble(),
                              bounds_min[1].GetDouble(),
                              bounds_min[2].GetDouble() },
                            { bounds_max[0].GetDouble(),
                              bounds_max[1].GetDouble(),
                              bounds_max[2].GetDouble() } };

  props.morton_index_parser = [](const std::string& name) {
    return DynamicMortonIndex::parse_string(
      name, MortonIndexNamingConvention::Potree);
  };

  props.points_have_offset = true;

  return props;
}

static tl::expected<Properties, std::string>
parse_ept_json(const std::string& ept_json_path)
{
  auto fp = fopen(ept_json_path.c_str(), "r");
  if (!fp) {
    const auto error_msg =
      boost::format("Can't open ept.json file @ \"%1%\"") % ept_json_path;
    return tl::make_unexpected(error_msg.str());
  }

  BOOST_SCOPE_EXIT(&fp) { fclose(fp); }
  BOOST_SCOPE_EXIT_END

  char buf[65536];
  rj::FileReadStream stream{ fp, buf, sizeof(buf) };

  rj::Document document;
  if (document.ParseStream(stream).HasParseError()) {
    const auto parse_error_msg =
      rapidjson::GetParseError_En(document.GetParseError());
    const auto error_msg =
      boost::format("Can't parse ept.json file [%1%]") % parse_error_msg;
    return tl::make_unexpected(error_msg.str());
  }

  Properties props;

  const auto& bounds_member = document["bounds"];
  props.root_bounds = AABB{ { bounds_member[0].GetDouble(),
                              bounds_member[1].GetDouble(),
                              bounds_member[2].GetDouble() },
                            { bounds_member[3].GetDouble(),
                              bounds_member[4].GetDouble(),
                              bounds_member[5].GetDouble() } };

  const auto span = document["span"].GetInt64();
  const auto bounds_extent = props.root_bounds.extent();

  props.root_spacing = bounds_extent.x / span;

  props.morton_index_parser = [](const std::string& name) {
    return DynamicMortonIndex::parse_string(
      name, MortonIndexNamingConvention::Entwine);
  };

  props.points_have_offset = false;

  return props;
}

static tl::expected<Properties, std::string>
parse_cloud_js(const std::string& cloud_js_path)
{
  auto fp = fopen(cloud_js_path.c_str(), "r");
  if (!fp) {
    const auto error_msg =
      boost::format("Can't open cloud.js file @ \"%1%\"") % cloud_js_path;
    return tl::make_unexpected(error_msg.str());
  }

  BOOST_SCOPE_EXIT(&fp) { fclose(fp); }
  BOOST_SCOPE_EXIT_END

  char buf[65536];
  rj::FileReadStream stream{ fp, buf, sizeof(buf) };

  rj::Document document;
  if (document.ParseStream(stream).HasParseError()) {
    const auto parse_error_msg =
      rapidjson::GetParseError_En(document.GetParseError());
    const auto error_msg =
      boost::format("Can't parse cloud.js file [%1%]") % parse_error_msg;
    return tl::make_unexpected(error_msg.str());
  }

  Properties props;

  props.root_spacing = static_cast<float>(document["spacing"].GetDouble());

  const auto& bounds_member = document["boundingBox"];

  props.root_bounds = AABB{ { bounds_member["lx"].GetDouble(),
                              bounds_member["ly"].GetDouble(),
                              bounds_member["lz"].GetDouble() },
                            { bounds_member["ux"].GetDouble(),
                              bounds_member["uy"].GetDouble(),
                              bounds_member["uz"].GetDouble() } };

  props.morton_index_parser = [](const std::string& name) {
    return DynamicMortonIndex::parse_string(
      name, MortonIndexNamingConvention::Potree);
  };

  props.points_have_offset = true;

  return props;
}

static tl::expected<Properties, std::string>
parse_properties(const std::string& source_folder)
{
  const auto properties_json_path = source_folder + "/properties.json";
  if (fs::exists(properties_json_path))
    return parse_properties_json(properties_json_path);

  const auto ept_json_path = source_folder + "/ept.json";
  if (fs::exists(ept_json_path))
    return parse_ept_json(ept_json_path);

  const auto cloud_js_path = source_folder + "/cloud.js";
  if (fs::exists(cloud_js_path))
    return parse_cloud_js(cloud_js_path);

  return tl::make_unexpected(std::string{
    "Source folder does not contain \"properties.json\" or \"ept.json\" "
    "file! "
    "Converter process only supports source folders that contain the result "
    "of a "
    "'tiler' process invocation, an Entwine 'build' process invocation or a "
    "potree-converter invocation!" });
}

static std::unique_ptr<SRSTransformHelper>
get_transformation_helper(const std::optional<std::string>& sourceProjection)
{
  if (!sourceProjection) {
    util::write_log(
      "Source projection not specified, skipping point transformation");
    return std::make_unique<IdentityTransform>();
  }

  if (std::strcmp(sourceProjection->c_str(),
                  "+proj=longlat +datum=WGS84 +no_defs") == 0) {
    util::write_log(
      "Source projection is already WGS84, skipping point transformation");
    return std::make_unique<IdentityTransform>();
  }

  try {
    return std::make_unique<Proj4Transform>(*sourceProjection);
  } catch (const std::runtime_error& err) {
    util::write_log(concat("Error while setting up coordinate transformation: ",
                           err.what(),
                           "\nSkipping point transformation"));
  }
  return std::make_unique<IdentityTransform>();
}

static std::optional<PointsPersistence>
get_persistence_for_file(const fs::path& file_path,
                         const std::string& source_folder,
                         const PointAttributes& attributes,
                         float spacing_at_root)
{
  const auto extension = file_path.extension();

  if (extension == ".bin") {
    return std::make_optional<PointsPersistence>(
      BinaryPersistence{ source_folder, attributes, Compressed::No });
  }
  if (extension == ".binz") {
    return std::make_optional<PointsPersistence>(
      BinaryPersistence{ source_folder, attributes, Compressed::Yes });
  }
  if (extension == ".las") {
    return std::make_optional<PointsPersistence>(
      LASPersistence{ source_folder, attributes, Compressed::No });
  }
  if (extension == ".laz") {
    return std::make_optional<PointsPersistence>(
      LASPersistence{ source_folder, attributes, Compressed::Yes });
  }
  if (extension == ".pnts") {
    return std::make_optional<PointsPersistence>(Cesium3DTilesPersistence{
      source_folder, attributes, spacing_at_root, {} });
  }

  return std::nullopt;
}

static bool
is_valid_file_extension(const std::string& extension)
{
  static const std::unordered_set<std::string> s_valid_extensions = {
    ".las", ".laz", ".bin", ".binz", ".pnts"
  };
  return s_valid_extensions.find(extension) != s_valid_extensions.end();
}

struct OctreeNode
{
  OctreeNode()
    : parent(nullptr)
    , level(-1)
  {}

  OctreeNode const* parent;
  std::array<std::unique_ptr<OctreeNode>, 8> children;

  int32_t level;
  std::string name;
  fs::path filepath;
  AABB bounds;
  float spacing;
};

static std::vector<fs::path>
find_all_octree_node_files(const std::string& source_folder,
                           std::optional<uint32_t> max_depth,
                           NodeNameToMortonIndex_t parse_morton_index)
{
  const auto all_files =
    get_all_files_in_directory(source_folder, Recursive::Yes);
  std::vector<fs::path> filtered_files;
  filtered_files.reserve(all_files.size());
  if (max_depth) {
    std::copy_if(all_files.begin(),
                 all_files.end(),
                 std::back_inserter(filtered_files),
                 [=](const auto& file) {
                   return is_valid_file_extension(file.extension().string()) &&
                          parse_morton_index(file.stem().string())
                            .map([=](const auto& morton_index) {
                              return morton_index.depth() <= *max_depth;
                            })
                            .value_or(false);
                 });
  } else {
    std::copy_if(all_files.begin(),
                 all_files.end(),
                 std::back_inserter(filtered_files),
                 [](const auto& file) {
                   return is_valid_file_extension(file.extension().string());
                 });
  }

  return filtered_files;
}

static std::unique_ptr<OctreeNode>
generate_tree(const std::vector<fs::path>& node_files,
              const AABB& root_bounds,
              float root_spacing,
              NodeNameToMortonIndex_t morton_index_parser)
{
  auto root_node = std::make_unique<OctreeNode>();

  const auto find_node_from_name = [&](const auto& name)
    -> tl::expected<std::pair<OctreeNode*, DynamicMortonIndex>, std::string> {
    return morton_index_parser(name).map(
      [&](const auto& current_node_morton_index)
        -> std::pair<OctreeNode*, DynamicMortonIndex> {
        auto current_node = root_node.get();
        for (auto octant : current_node_morton_index) {
          auto& child_node = current_node->children[octant];
          if (!child_node) {
            child_node = std::make_unique<OctreeNode>();
            child_node->parent = current_node;
            child_node->level = current_node->level + 1;
          }
          current_node = child_node.get();
        }
        return std::make_pair(current_node, current_node_morton_index);
      });
  };

  for (auto& file : node_files) {
    const auto& node_name = file.filename().stem().string();
    find_node_from_name(node_name)
      .map([&](const auto& node_and_morton_index) {
        const auto node_for_file = node_and_morton_index.first;
        const auto& morton_index = node_and_morton_index.second;

        node_for_file->name = node_name;
        node_for_file->filepath = file;
        node_for_file->bounds =
          get_bounds_from_morton_index(morton_index, root_bounds);
        node_for_file->spacing =
          root_spacing / std::pow(2, node_for_file->level + 1);
      })
      .or_else([&](const auto& error_msg) {
        std::cerr << "Couldn't process node \"" << node_name << "\" ["
                  << error_msg << "]" << std::endl;
      });
  }

  return root_node;
}

static std::vector<OctreeNode const*>
get_children_at_level(const OctreeNode& root_node, uint32_t level)
{
  // level 0 == root_node
  std::vector<OctreeNode const*> children;
  std::queue<OctreeNode const*> to_process;
  to_process.push(&root_node);

  while (!to_process.empty()) {
    const auto current_node = to_process.front();
    to_process.pop();

    const auto level_diff = current_node->level - root_node.level;
    if (level_diff == static_cast<int32_t>(level)) {
      children.push_back(current_node);
    } else {
      for (size_t idx = 0; idx < 8; ++idx) {
        if (current_node->children[idx] != nullptr) {
          to_process.push(current_node->children[idx].get());
        }
      }
    }
  }

  return children;
}

static std::vector<OctreeNode const*>
split_tree_into_subtrees(const OctreeNode& root_node,
                         uint32_t max_levels_per_subtree)
{
  std::vector<OctreeNode const*> subtrees;
  std::queue<OctreeNode const*> next_root_nodes;
  next_root_nodes.push(&root_node);

  while (!next_root_nodes.empty()) {
    const auto current_root_node = next_root_nodes.front();
    next_root_nodes.pop();
    subtrees.push_back(current_root_node);

    // Go down max_levels_per_subtree and find all the children of
    // current_root_node at that level
    const auto children_at_max_level =
      get_children_at_level(*current_root_node, max_levels_per_subtree);
    for (auto child : children_at_max_level) {
      next_root_nodes.push(child);
    }
  }

  return subtrees;
}

static Tileset
create_tileset_for_leaf_node(const OctreeNode& node,
                             const SRSTransformHelper& transformation);

static Tileset
create_tileset_for_interior_node(const OctreeNode& node,
                                 const SRSTransformHelper& transformation,
                                 uint32_t max_level)
{
  Tileset tileset;
  tileset.url = node.name + ".json";
  tileset.geometricError = node.spacing * spacing_correction_factor;
  tileset.boundingVolume = boundingVolumeFromAABB(node.bounds, transformation);
  tileset.content_url = node.name + ".pnts";

  for (auto& child_node : node.children) {
    if (!child_node)
      continue;

    const auto child_has_children = std::any_of(
      child_node->children.begin(),
      child_node->children.end(),
      [](const auto& child_of_child) { return child_of_child != nullptr; });

    if (max_level == 1 && child_has_children) {
      tileset.children.push_back(
        create_tileset_for_leaf_node(*child_node, transformation));
    } else {
      tileset.children.push_back(create_tileset_for_interior_node(
        *child_node, transformation, max_level - 1));
    }
  }

  return tileset;
}

static Tileset
create_tileset_for_leaf_node(const OctreeNode& node,
                             const SRSTransformHelper& transformation)
{
  Tileset tileset;

  tileset.url = node.name + ".json";
  tileset.geometricError = node.spacing * spacing_correction_factor;
  tileset.boundingVolume = boundingVolumeFromAABB(node.bounds, transformation);
  tileset.content_url = node.name + ".json";

  return tileset;
}

static void
write_tileset_json_for_subtree(const OctreeNode& subtree_root,
                               uint32_t depth_of_subtree,
                               const SRSTransformHelper& transformation,
                               const std::string& out_folder)
{
  const auto tileset_for_subtree = create_tileset_for_interior_node(
    subtree_root, transformation, depth_of_subtree);

  const auto tileset_path = out_folder + "/" + tileset_for_subtree.url;
  writeTilesetJSON(tileset_path, tileset_for_subtree);
}

static std::string
get_pnts_file_path(const fs::path& las_file_path,
                   const std::string& output_folder)
{
  const auto name_stem = las_file_path.stem();
  return concat(output_folder, "/", name_stem.generic_string(), ".pnts");
}

static void
convert_to_pnts_file(const std::string& source_folder,
                     const fs::path& input_file,
                     const std::string& output_folder,
                     const PointAttributes& attributes,
                     float spacing_at_root,
                     const SRSTransformHelper& transformation,
                     DeleteSource delete_source)
{
  auto persistence = get_persistence_for_file(
    input_file, source_folder, attributes, spacing_at_root);
  if (!persistence) {
    util::write_log(concat("Could not read source file \"",
                           input_file.filename().string(),
                           "\": Unrecognized format!\n"));
    return;
  }

  const auto out_file_name = get_pnts_file_path(input_file, output_folder);
  PNTSWriter writer{ out_file_name, attributes };

  const auto node_name = input_file.filename().stem().string();

  PointBuffer source_data;
  persistence->retrieve_points(node_name, source_data);
  transformation.transformPositionsTo(TargetSRS::CesiumWorld,
                                      gsl::make_span(source_data.positions()));
  auto local_offset_to_world =
    setOriginToSmallestPoint(source_data.positions());

  std::vector<PointBuffer::PointReference> point_references;
  point_references.reserve(source_data.count());
  std::copy(std::begin(source_data),
            std::end(source_data),
            std::back_inserter(point_references));

  writer.write_points(point_references);
  writer.flush(local_offset_to_world);

  if (delete_source == DeleteSource::Yes) {
    util::write_log("Deleting source file!");
    std::error_code ec;
    if (!fs::remove(input_file, ec)) {
      std::cerr << ec.message() << std::endl;
    }
  }
}

static void
convert_to_las_file(const std::string& source_folder,
                    const fs::path& input_file,
                    const std::string& output_folder,
                    const PointAttributes& attributes,
                    const SRSTransformHelper& transformation,
                    const AABB& root_bounds,
                    float spacing_at_root,
                    NodeNameToMortonIndex_t morton_index_parser,
                    Compressed compressed,
                    DeleteSource delete_source)
{
  auto persistence = get_persistence_for_file(
    input_file, source_folder, attributes, spacing_at_root);
  if (!persistence) {
    util::write_log(concat("Could not read source file \"",
                           input_file.filename().string(),
                           "\": Unrecognized format!\n"));
    return;
  }

  LASPersistence las_persistence{ output_folder, attributes, compressed };

  const auto node_name = input_file.filename().stem().string();

  morton_index_parser(node_name)
    .map([&](const auto& node_morton_index) {
      const auto node_bounds =
        get_bounds_from_morton_index(node_morton_index, root_bounds);

      PointBuffer source_data;
      persistence->retrieve_points(node_name, source_data);

      las_persistence.persist_points(
        std::begin(source_data), std::end(source_data), node_bounds, node_name);

      if (delete_source == DeleteSource::Yes) {
        std::error_code ec;
        if (!fs::remove(input_file, ec)) {
          std::cerr << ec.message() << std::endl;
        }
      }
    })
    .or_else(
      [&](const auto& error_msg) { std::cerr << error_msg << std::endl; });
}

static void
convert_to_3dtiles_impl(const ConverterArguments& args,
                        const Properties& properties,
                        ProgressReporter& progress_reporter)
{
  const auto transformation = get_transformation_helper(args.source_projection);
  // Gather source files and create a tree structure in memory
  const auto node_files = find_all_octree_node_files(
    args.source_folder, args.max_depth, properties.morton_index_parser);

  progress_reporter.register_progress_counter<size_t>(progress::CONVERTING,
                                                      node_files.size());

  TaskSystem task_system;
  task_system.run();

  std::vector<async::Awaitable<void>> futures;

  futures.push_back(task_system.push([&]() {
    const auto octree_root = generate_tree(node_files,
                                           properties.root_bounds,
                                           properties.root_spacing,
                                           properties.morton_index_parser);

    // Split tree into subtrees
    const auto subtrees = split_tree_into_subtrees(
      *octree_root, 3); // 3 levels: 8³ = 512 max nodes per subtree

    progress_reporter.register_progress_counter<size_t>(
      progress::GENERATING_TILESETS, subtrees.size());

    // Generate a tileset.json file for each subtree
    // Need 2 methods: One for a full tileset with all content information,
    // children etc., and one for a leaf node, only containing references to the
    // next tileset.json and bounds
    for (auto subtree : subtrees) {
      write_tileset_json_for_subtree(
        *subtree, 3, *transformation, args.output_folder);

      progress_reporter.increment_progress<size_t>(
        progress::GENERATING_TILESETS, 1);
    }
  }));

  // Convert LAS/LAZ files into .pnts files
  for (auto& node_file : node_files) {
    futures.push_back(task_system.push([&]() {
      convert_to_pnts_file(args.source_folder,
                           node_file,
                           args.output_folder,
                           args.output_attributes,
                           properties.root_spacing,
                           *transformation,
                           (args.delete_source_files) ? DeleteSource::Yes
                                                      : DeleteSource::No);

      progress_reporter.increment_progress<size_t>(progress::CONVERTING, 1);
    }));
  }

  async::all(std::move(futures)).await();

  task_system.stop_and_join();
}

static void
convert_to_3dtiles(const ConverterArguments& args,
                   ProgressReporter& progress_reporter)
{
  // Look for a description file in the source folder and extract properties
  // from it This can be either a properties.json file (generated by the Tiler
  // process) or an ept.json file (generated by an Entwine conversion)
  parse_properties(args.source_folder)
    .map([&](const Properties& properties) {
      convert_to_3dtiles_impl(args, properties, progress_reporter);
    })
    .or_else([](const auto& error_msg) {
      std::cerr << error_msg;
      std::exit(1);
    });
}

static void
convert_to_las_impl(const ConverterArguments& args,
                    const Properties& properties,
                    ProgressReporter& progress_reporter,
                    Compressed compressed)
{
  const auto transformation = get_transformation_helper(args.source_projection);
  // Gather source files and create a tree structure in memory
  const auto node_files = find_all_octree_node_files(
    args.source_folder, args.max_depth, properties.morton_index_parser);

  util::write_log(concat("Converting ", node_files.size(), " files\n"));

  progress_reporter.register_progress_counter<size_t>(progress::CONVERTING,
                                                      node_files.size());

  TaskSystem task_system;
  task_system.run();

  std::vector<async::Awaitable<void>> futures;
  futures.reserve(node_files.size());

  // Convert .bin files into .laz files
  for (auto& node_file : node_files) {
    futures.push_back(task_system.push([&]() {
      convert_to_las_file(args.source_folder,
                          node_file,
                          args.output_folder,
                          args.output_attributes,
                          *transformation,
                          properties.root_bounds,
                          properties.root_spacing,
                          properties.morton_index_parser,
                          compressed,
                          (args.delete_source_files) ? DeleteSource::Yes
                                                     : DeleteSource::No);

      progress_reporter.increment_progress<size_t>(progress::CONVERTING, 1);
    }));
  }

  async::all(std::move(futures)).await();

  task_system.stop_and_join();
}

static void
convert_to_las(const ConverterArguments& args,
               ProgressReporter& progress_reporter,
               Compressed compressed)
{
  parse_properties(args.source_folder)
    .map([&](const Properties& properties) {
      convert_to_las_impl(args, properties, progress_reporter, compressed);
    })
    .or_else([](const auto& error_msg) {
      std::cerr << error_msg;
      std::exit(1);
    });
}

static void
prepare_conversion(const std::string& source_folder,
                   const std::string& target_folder)
{
  if (!fs::exists(source_folder)) {
    util::write_log(
      concat("Source folder \"", source_folder, "\" does not exist!"));
    std::exit(EXIT_FAILURE);
  }

  if (target_folder == source_folder)
    return;

  // TODO Add option to clear target folder!
  if (fs::exists(target_folder)) {
    fs::remove_all(target_folder);
  }

  if (!fs::create_directories(target_folder)) {
    util::write_log(
      concat("Could not create output folder \"", source_folder, "\"!"));
    std::exit(EXIT_FAILURE);
  }
}

void
run_conversion(const ConverterArguments& args)
{

  UIState ui_state;
  TerminalUI ui{ &ui_state };

  std::atomic_bool draw_ui = true;
  std::thread ui_thread{ [&ui, &draw_ui]() {
    while (draw_ui) {
      ui.redraw();
      std::this_thread::sleep_for(std::chrono::milliseconds{ 50 });
    }
  } };

  prepare_conversion(args.source_folder, args.output_folder);
  switch (args.output_format) {
    case OutputFormat::CZM_3DTILES:
      convert_to_3dtiles(args, ui_state.get_progress_reporter());
      break;
    case OutputFormat::LAS:
      convert_to_las(args, ui_state.get_progress_reporter(), Compressed::No);
      break;
    case OutputFormat::LAZ:
      convert_to_las(args, ui_state.get_progress_reporter(), Compressed::Yes);
      break;
    default:
      break;
  }

  draw_ui = false;
  ui_thread.join();
}