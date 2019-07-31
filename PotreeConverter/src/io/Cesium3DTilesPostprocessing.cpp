#include "io/Cesium3DTilesPostprocessing.h"

#include "AABB.h"
#include "PNTSWriter.h"
#include "PointAttributes.hpp"
#include "TileSetWriter.h"
#include "Tileset.h"
#include "Transformation.h"
#include "definitions.hpp"
#include "octree/OctreeAlgorithms.h"
#include "octree/OctreeNodeKey.h"
#include "stuff.h"
#include "ui/ProgressReporter.h"

#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>

static std::unordered_set<std::string>
gather_unique_node_names(const std::vector<fs::path>& files)
{
  std::unordered_set<std::string> node_names;
  for (auto& file : files) {
    const auto filename = file.filename().string();
    node_names.insert(filename.substr(0, filename.find_first_of('.')));
  }
  return node_names;
}

static void
create_tilesets(const std::unordered_set<std::string>& node_names,
                const Potree::AABB& root_bounds,
                float spacing_at_root,
                const std::string& work_dir,
                const Potree::SRSTransformHelper& transform,
                ProgressReporter* progress_reporter)
{
  constexpr float GeometricErrorCorrectionFactor = 2.f;

  std::vector<std::string> sorted_node_names{ node_names.begin(), node_names.end() };
  std::sort(sorted_node_names.begin(), sorted_node_names.end(), [](const auto& l, const auto& r) {
    // Sort first by length, then lexicographically
    if (l.size() == r.size())
      return l < r;
    return l.size() < r.size();
  });

  /**
   * Returns the names of all nodes that are children of the given parent node. Children are
   * nodes whose name is exactly one character longer than the parent name and whose name begins
   * with the parent node name (e.g. 'r1234' is a child of 'r123' but not of 'r12')
   */
  const auto find_child_nodes = [&sorted_node_names](const std::string& parent_node_name,
                                                     size_t start_idx) {
    // Since the nodes are sorted by length, we can do a linear search and terminate once the node
    // names are too long
    std::vector<std::string> child_names;
    for (size_t idx = start_idx; idx < sorted_node_names.size(); ++idx) {
      const auto& current_node_name = sorted_node_names[idx];
      if (current_node_name.size() == parent_node_name.size())
        continue;
      if (current_node_name.size() > (parent_node_name.size() + 1))
        break;

      assert(current_node_name.size() == (parent_node_name.size() + 1));
      if (boost::starts_with(current_node_name, parent_node_name)) {
        child_names.push_back(current_node_name);
      }
    }
    return child_names;
  };

  std::unordered_map<std::string, std::unique_ptr<Tileset>> tilesets;
  tilesets.reserve(sorted_node_names.size());

  for (size_t idx = 0; idx < sorted_node_names.size(); ++idx) {
    const auto& current_node_name = sorted_node_names[idx];
    const auto current_node_level = (current_node_name.size() - 1);
    const auto current_node_key = from_string<42>(
      current_node_name); // TODO What if the node name is too long to fit in the key?
    const auto current_node_bounds =
      get_bounds_from_octree_key(current_node_key, root_bounds, current_node_level);
    const auto current_node_spacing = spacing_at_root / std::pow(2, current_node_level + 1);

    Tileset* tileset;
    const auto iter = tilesets.find(current_node_name);
    if (iter == tilesets.end()) {
      auto tileset_ptr = std::make_unique<Tileset>();
      tileset = tileset_ptr.get();
      tilesets[current_node_name] = std::move(tileset_ptr);
    } else {
      tileset = iter->second.get();
    }

    const auto children_names = find_child_nodes(current_node_name, idx);

    tileset->name = current_node_name;
    tileset->content_url = concat(current_node_name, ".pnts");
    tileset->url = concat(current_node_name, ".json");
    tileset->geometricError = current_node_spacing * GeometricErrorCorrectionFactor;
    tileset->boundingVolume = boundingVolumeFromAABB(current_node_bounds, transform);

    for (auto& child_name : children_names) {
      auto child_tileset = std::make_unique<Tileset>();
      tileset->children.push_back(child_tileset.get());
      tilesets[child_name] = std::move(child_tileset);
    }
  }

  constexpr size_t UpdateProgressInterval = 1024;
  size_t processed_tilesets = 0;

  // Write out all tilesets
  for (auto& kv : tilesets) {
    const auto& tileset = *kv.second;
    Potree::writeTilesetJSON(concat(work_dir, "/", tileset.name, ".json"), tileset);
    ++processed_tilesets;

    if (progress_reporter && (processed_tilesets == UpdateProgressInterval)) {
      progress_reporter->increment_progress<size_t>(progress::POSTPROCESSING, processed_tilesets);
      processed_tilesets = 0;
    }
  }

  if (progress_reporter) {
    progress_reporter->increment_progress<size_t>(progress::POSTPROCESSING, processed_tilesets);
  }
}

void
do_cesium_3dtiles_postprocessing(const std::string& work_dir,
                                 const Potree::AABB& bounds,
                                 float spacing_at_root,
                                 const Potree::SRSTransformHelper& transform,
                                 const Potree::PointAttributes& point_attributes,
                                 ProgressReporter* progress_reporter)
{
  const auto is_pnts_file = [](const fs::path& file_path) {
    return file_path.extension() == ".pnts";
  };
  const auto is_idx_file = [](const fs::path& file_path) {
    return file_path.extension() == ".idx";
  };

  const auto postprocess_file = [&](const fs::path& file) {
    if (is_idx_file(file)) {
      if (!fs::remove(file)) {
        std::cerr << "Could not remove index file " << file.string() << std::endl;
      }

      if (progress_reporter) {
        progress_reporter->increment_progress<size_t>(progress::POSTPROCESSING, 1);
      }
    } else if (is_pnts_file(file)) {
      transform_pnts_file_coordinates(
        file.string(), Potree::Recenter::Yes, transform, point_attributes);

      if (progress_reporter) {
        progress_reporter->increment_progress<size_t>(progress::POSTPROCESSING, 1);
      }
    }
  };

  const auto files = get_all_files_in_directory(work_dir);
  const auto node_names = gather_unique_node_names(files);

  if (progress_reporter) {
    const auto total_files_to_process =
      files.size() + node_names.size(); // All existing files have to be touched (pnts files for
                                        // transformation, idx files for deletion). In addition, for
                                        // each node one tileset JSON file is created

    progress_reporter->register_progress_counter<size_t>(progress::POSTPROCESSING,
                                                         total_files_to_process);
  }

  create_tilesets(node_names, bounds, spacing_at_root, work_dir, transform, progress_reporter);

  const auto hardware_concurrency = std::thread::hardware_concurrency();
  if (files.size() < hardware_concurrency) {
    // Run serial, no need to parallelize
    for (auto& file : files) {
      postprocess_file(file);
    }
    return;
  }

  // Parallelize
  const auto file_ranges =
    split_range_into_chunks(hardware_concurrency, files.begin(), files.end());

  std::vector<std::thread> postprocess_threads;
  postprocess_threads.reserve(hardware_concurrency);
  for (size_t idx = 0; idx < hardware_concurrency; ++idx) {
    const auto range_begin = file_ranges[idx].first;
    const auto range_end = file_ranges[idx].second;
    postprocess_threads.emplace_back(
      [=]() { std::for_each(range_begin, range_end, postprocess_file); });
  }

  for (auto& thread : postprocess_threads) {
    thread.join();
  }
}