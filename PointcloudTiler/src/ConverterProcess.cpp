#include "ConverterProcess.h"

#include "AABB.h"
#include "LASPointReader.h"
#include "PNTSWriter.h"
#include "TileSetWriter.h"
#include "Tileset.h"
#include "Transformation.h"
#include "octree/OctreeAlgorithms.h"
#include "stuff.h"
#include "util/TaskSystem.h"

#include <array>
#include <boost/scope_exit.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/filereadstream.h>

namespace fs = std::experimental::filesystem;
namespace rj = rapidjson;

constexpr float spacing_correction_factor = 2;

struct Properties {
  AABB root_bounds;
  float root_spacing;
};

static std::optional<Properties>
parse_properties_json(const std::string &source_folder) {
  const auto properties_json_path = source_folder + "/properties.json";
  if (!fs::exists(properties_json_path)) {
    return std::nullopt;
  }

  auto fp = fopen(properties_json_path.c_str(), "r");
  if (!fp)
    return std::nullopt;

  BOOST_SCOPE_EXIT(&fp) { fclose(fp); }
  BOOST_SCOPE_EXIT_END

  char buf[65536];
  rj::FileReadStream stream{fp, buf, sizeof(buf)};

  rj::Document document;
  if (document.ParseStream(stream).HasParseError()) {
    return std::nullopt;
  }

  Properties props;

  props.root_spacing = static_cast<float>(
      document["source_properties"]["root_spacing"].GetDouble());

  const auto &bounds_member = document["source_properties"]["bounds"];
  const auto &bounds_min = bounds_member["min"];
  const auto &bounds_max = bounds_member["max"];

  props.root_bounds =
      AABB{{bounds_min[0].GetDouble(), bounds_min[1].GetDouble(),
            bounds_min[2].GetDouble()},
           {bounds_max[0].GetDouble(), bounds_max[1].GetDouble(),
            bounds_max[2].GetDouble()}};

  return std::make_optional(props);
}

static std::unique_ptr<SRSTransformHelper>
get_transformation_helper(const std::optional<std::string> &sourceProjection) {
  if (!sourceProjection) {
    std::cout << "Source projection not specified, skipping point "
                 "transformation..."
              << std::endl;
    return std::make_unique<IdentityTransform>();
  }

  if (std::strcmp(sourceProjection->c_str(),
                  "+proj=longlat +datum=WGS84 +no_defs") == 0) {
    std::cout << "Source projection is already WGS84, skipping point "
                 "transformation..."
              << std::endl;
    return std::make_unique<IdentityTransform>();
  }

  try {
    return std::make_unique<Proj4Transform>(*sourceProjection);
  } catch (const std::runtime_error &err) {
    std::cerr << "Error while setting up coordinate transformation:\n"
              << err.what() << std::endl;
    std::cerr << "Skipping point transformation..." << std::endl;
  }
  return std::make_unique<IdentityTransform>();
}

struct OctreeNode {
  OctreeNode() : parent(nullptr), level(-1) {}

  OctreeNode const *parent;
  std::array<std::unique_ptr<OctreeNode>, 8> children;

  int32_t level;
  std::string name;
  fs::path filepath;
  AABB bounds;
  float spacing;
};

static std::vector<fs::path>
find_all_octree_node_files(const std::string &source_folder,
                           std::optional<uint32_t> max_depth) {
  const auto all_files =
      get_all_files_in_directory(source_folder, Recursive::Yes);
  std::vector<fs::path> filtered_files;
  filtered_files.reserve(all_files.size());
  if (max_depth) {
    std::copy_if(
        all_files.begin(), all_files.end(), std::back_inserter(filtered_files),
        [=](const auto &file) {
          const auto name_length = file.stem().string().size();
          return (file.extension() == ".las" || file.extension() == ".laz") &&
                 (name_length <=
                  (*max_depth +
                   1)); // root node is level 0, but has name length 1 ('r')
        });
  } else {
    std::copy_if(all_files.begin(), all_files.end(),
                 std::back_inserter(filtered_files), [](const auto &file) {
                   return file.extension() == ".las" ||
                          file.extension() == ".laz";
                 });
  }

  return filtered_files;
}

static std::unique_ptr<OctreeNode>
generate_tree(const std::vector<fs::path> &node_files, const AABB &root_bounds,
              float root_spacing) {
  auto root_node = std::make_unique<OctreeNode>();

  const auto find_node_from_name = [&](const auto &name) {
    auto current_node = root_node.get();
    auto hierarchy_name =
        std::string(name.begin() + 1, name.begin() + name.find("."));
    for (auto octant_char : hierarchy_name) {
      const auto octant_idx = static_cast<uint32_t>(octant_char - '0');
      auto &child_node = current_node->children[octant_idx];
      if (!child_node) {
        child_node = std::make_unique<OctreeNode>();
        child_node->parent = current_node;
        child_node->level = current_node->level + 1;
      }
      current_node = child_node.get();
    }
    return current_node;
  };

  for (auto &file : node_files) {
    const auto &filename = file.filename().string();
    auto node_for_file = find_node_from_name(filename);

    // Populate node
    node_for_file->name = std::string{
        filename.begin(),
        filename.begin() +
            filename.find(".")}; // This will be something like 'r01234'
    node_for_file->filepath = file;
    node_for_file->bounds =
        get_bounds_from_node_name(node_for_file->name, root_bounds);
    node_for_file->spacing =
        root_spacing / std::pow(2, node_for_file->level + 1);
  }

  return root_node;
}

static std::vector<OctreeNode const *>
get_children_at_level(const OctreeNode &root_node, uint32_t level) {
  // level 0 == root_node
  std::vector<OctreeNode const *> children;
  std::queue<OctreeNode const *> to_process;
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

static std::vector<OctreeNode const *>
split_tree_into_subtrees(const OctreeNode &root_node,
                         uint32_t max_levels_per_subtree) {
  std::vector<OctreeNode const *> subtrees;
  std::queue<OctreeNode const *> next_root_nodes;
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
create_tileset_for_leaf_node(const OctreeNode &node,
                             const SRSTransformHelper &transformation);

static Tileset
create_tileset_for_interior_node(const OctreeNode &node,
                                 const SRSTransformHelper &transformation,
                                 uint32_t max_level) {
  Tileset tileset;
  tileset.url = node.name + ".json";
  tileset.geometricError = node.spacing * spacing_correction_factor;
  tileset.boundingVolume = boundingVolumeFromAABB(node.bounds, transformation);
  tileset.content_url = node.name + ".pnts";

  // TODO create child tilesets and add here. Do this up to the maximum level of
  // the subtree; for this we need a root-level parameter passed into this
  // function!
  for (auto &child_node : node.children) {
    if (!child_node)
      continue;

    const auto child_has_children = std::any_of(
        child_node->children.begin(), child_node->children.end(),
        [](const auto &child_of_child) { return child_of_child != nullptr; });

    if (max_level == 0 && child_has_children) {
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
create_tileset_for_leaf_node(const OctreeNode &node,
                             const SRSTransformHelper &transformation) {
  Tileset tileset;

  tileset.url = node.name + ".json";
  tileset.geometricError = node.spacing * spacing_correction_factor;
  tileset.boundingVolume = boundingVolumeFromAABB(node.bounds, transformation);

  return tileset;
}

static void write_tileset_json_for_subtree(
    const OctreeNode &subtree_root, uint32_t depth_of_subtree,
    const SRSTransformHelper &transformation, const std::string &out_folder) {
  const auto tileset_for_subtree = create_tileset_for_interior_node(
      subtree_root, transformation, depth_of_subtree);

  const auto tileset_path = out_folder + "/" + tileset_for_subtree.url;
  writeTilesetJSON(tileset_path, tileset_for_subtree);
}

static std::string get_pnts_file_path(const fs::path &las_file_path,
                                      const std::string &output_folder) {
  const auto name_stem = las_file_path.stem();
  return concat(output_folder, "/", name_stem.generic_string(), ".pnts");
}

static void convert_to_pnts_file(const fs::path &input_file,
                                 const std::string &output_folder,
                                 const PointAttributes &attributes,
                                 const SRSTransformHelper &transformation) {
  LIBLASReader reader{input_file.string(), attributes};
  const auto num_points = static_cast<size_t>(reader.numPoints());

  const auto out_file_name = get_pnts_file_path(input_file, output_folder);
  PNTSWriter writer{out_file_name, attributes};

  auto read_points = reader.readNextBatch(num_points);
  const auto points_offset = reader.getOffset();
  for (auto &position : read_points.positions()) {
    position += points_offset;
  }

  transformation.transformPositionsTo(TargetSRS::CesiumWorld,
                                      gsl::make_span(read_points.positions()));
  auto local_offset_to_world =
      setOriginToSmallestPoint(read_points.positions());

  writer.write_points(read_points);
  writer.flush(local_offset_to_world);
}

static void convert_to_3dtiles(const std::string &source_folder,
                               const std::string &target_folder,
                               const PointAttributes &attributes,
                               std::optional<std::string> projection,
                               std::optional<uint32_t> max_depth) {
  // Look for properties.json and parse it
  const auto properties = parse_properties_json(source_folder);
  if (!properties) {
    std::cout
        << "Can't locate properties.json file in source folder. Make sure "
           "source folder "
           "contains the result of a previous invocation of the tiler process!"
        << std::endl;
    std::exit(1);
  }

  const auto transformation = get_transformation_helper(projection);
  // Gather source files and create a tree structure in memory
  const auto node_files = find_all_octree_node_files(source_folder, max_depth);

  TaskSystem task_system;
  task_system.run();

  std::vector<async::Awaitable<void>> futures;

  futures.push_back(task_system.push([&]() {
    const auto octree_root = generate_tree(node_files, properties->root_bounds,
                                           properties->root_spacing);

    // Split tree into subtrees
    const auto subtrees = split_tree_into_subtrees(
        *octree_root, 3); // 3 levels: 8³ = 512 max nodes per subtree

    // Generate a tileset.json file for each subtree
    // Need 2 methods: One for a full tileset with all content information,
    // children etc., and one for a leaf node, only containing references to the
    // next tileset.json and bounds
    for (auto subtree : subtrees) {
      write_tileset_json_for_subtree(*subtree, 3, *transformation,
                                     target_folder);
    }
  }));

  // Convert LAS/LAZ files into .pnts files
  for (auto &node_file : node_files) {
    futures.push_back(task_system.push([&]() {
      convert_to_pnts_file(node_file, target_folder, attributes,
                           *transformation);
    }));
  }

  async::all(std::move(futures)).await();

  task_system.stop_and_join();
}

static void prepare_conversion(const std::string &source_folder,
                               const std::string &target_folder) {
  if (!fs::exists(source_folder)) {
    std::cout << "Source folder \"" << source_folder << "\" does not exist!"
              << std::endl;
    std::exit(1);
  }

  if (fs::exists(target_folder)) {
    fs::remove_all(target_folder);
  }

  if (!fs::create_directories(target_folder)) {
    std::cout << "Could not create output folder \"" << target_folder << "\"!"
              << std::endl;
    std::exit(1);
  }
}

void run_conversion(const std::string &source_folder,
                    const std::string &target_folder,
                    OutputFormat target_format,
                    const PointAttributes &attributes,
                    std::optional<std::string> projection,
                    std::optional<uint32_t> max_depth) {
  prepare_conversion(source_folder, target_folder);
  switch (target_format) {
  case OutputFormat::CZM_3DTILES:
    convert_to_3dtiles(source_folder, target_folder, attributes, projection,
                       max_depth);
  default:
    break;
  }
}