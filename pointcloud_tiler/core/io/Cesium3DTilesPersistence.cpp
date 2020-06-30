#include "io/Cesium3DTilesPersistence.h"

#include "datastructures/DynamicMortonIndex.h"
#include "datastructures/OctreeNodeIndex.h"
#include "io/PNTSReader.h"
#include "io/PNTSWriter.h"
#include "io/TileSetWriter.h"
#include "pointcloud/PointAttributes.h"
#include "threading/Parallel.h"
#include "tiling/OctreeAlgorithms.h"
#include "util/Transformation.h"
#include "util/stuff.h"

#include <fstream>
#include <queue>
#include <taskflow/taskflow.hpp>

Cesium3DTilesPersistence::Cesium3DTilesPersistence(
  const std::string& work_dir,
  const PointAttributes& point_attributes,
  float spacing_at_root,
  Vector3<double> const& global_offset)
  : _work_dir(work_dir)
  , _point_attributes(point_attributes)
  , _spacing_at_root(spacing_at_root)
  , _global_offset(global_offset)
  , _tilesets_lock(std::make_unique<std::mutex>())
{}

Cesium3DTilesPersistence::~Cesium3DTilesPersistence()
{
  if (!_root_tileset)
    return;

  write_tilesets();
}

void
Cesium3DTilesPersistence::persist_points(PointBuffer const& points,
                                         const AABB& bounds,
                                         const std::string& node_name)
{
  if (points.empty()) {
    throw std::runtime_error{ "persist_points requires a non-empty range" };
  }

  PNTSWriter writer{ concat(_work_dir, "/", node_name, ".pnts"),
                     _point_attributes };
  writer.write_points(points);
  writer.flush(_global_offset);

  on_write_node(node_name, bounds);
}

void
Cesium3DTilesPersistence::retrieve_points(const std::string& node_name,
                                          PointBuffer& points)
{
  const auto file_path = concat(_work_dir, "/", node_name, ".pnts");
  if (!std::experimental::filesystem::exists(file_path))
    return;

  auto pnts_content = readPNTSFile(file_path);
  points = std::move(pnts_content->points);
}

void
Cesium3DTilesPersistence::on_write_node(const std::string& node_name,
                                        const AABB& node_bounds)
{
  std::lock_guard _{ *_tilesets_lock };

  const auto setup_tileset = [this](Tileset& tileset,
                                    const std::string& node_name,
                                    const AABB& node_bounds) {
    const auto node_morton_index =
      DynamicMortonIndex::parse_string(node_name,
                                       MortonIndexNamingConvention::Potree)
        .value();

    tileset.boundingVolume =
      boundingVolumeFromAABB(node_bounds.translate(_global_offset));
    tileset.content_url = concat(node_name, ".pnts");
    tileset.url = concat(node_name, ".json");
    tileset.geometricError =
      _spacing_at_root /
      std::pow(2.0, static_cast<double>(node_morton_index.depth()));
    tileset.name = node_name;
  };

  if (!_root_tileset) {
    const auto node_index =
      OctreeNodeIndex64::from_string(node_name.substr(1)).value();
    const auto root_bounds = get_root_bounds_from_node(node_index, node_bounds);

    _root_tileset = Tileset{};
    setup_tileset(*_root_tileset, "r", root_bounds);

    auto current_tileset = &*_root_tileset;
    auto current_bounds = root_bounds;

    for (uint32_t level = 0; level < node_index.levels(); ++level) {
      const auto current_octant = node_index.octant_at_level(level);
      current_bounds = get_octant_bounds(current_octant, current_bounds);
      const auto child_name =
        current_tileset->name + (static_cast<char>('0' + current_octant));

      auto& child = current_tileset->children.emplace_back();
      setup_tileset(child, child_name, current_bounds);

      current_tileset = &child;
    }

    return;
  }

  // Find the matching tileset or one of its parents, and create all the missing
  // tilesets between the next parent and this node
  const auto node_index =
    OctreeNodeIndex64::from_string(node_name.substr(1)).value();
  auto current_tileset = &*_root_tileset;
  auto current_bounds = get_root_bounds_from_node(node_index, node_bounds);

  for (size_t idx = 1; idx < node_name.size(); ++idx) {
    const auto child_index = node_index.parent_at_level(idx);
    const auto octant = child_index.octant_at_level(idx);

    auto substr = node_name.substr(0, idx + 1);
    auto child_iter = std::find_if(
      std::begin(current_tileset->children),
      std::end(current_tileset->children),
      [&substr](const Tileset& tileset) { return tileset.name == substr; });

    const auto child_bounds = get_octant_bounds(octant, current_bounds);

    if (child_iter == std::end(current_tileset->children)) {

      auto& child = current_tileset->children.emplace_back();
      setup_tileset(child, substr, child_bounds);

      child_iter = std::prev(std::end(current_tileset->children));

      if (idx == node_name.size() - 1) {
        break;
      }
    }

    current_tileset = &*child_iter;
    current_bounds = child_bounds;
  }
}

static void
iterate_tileset_children(Tileset const& current_tileset,
                         std::queue<Tileset const*>& working_queue,
                         uint32_t remaining_levels)
{
  if (remaining_levels == 0) {
    for (auto& child : current_tileset.children) {
      working_queue.push(&child);
    }
  } else {
    for (auto& child : current_tileset.children) {
      iterate_tileset_children(child, working_queue, remaining_levels - 1);
    }
  }
}

void
Cesium3DTilesPersistence::write_tilesets() const
{
  // Maximum depth from a tileset JSON file at which PNTS files are included
  // directly in the tileset. Below this depth, tilesets are included as
  // external tilesets. This creates a tree of JSON files where each JSON file
  // includes a tree of MAX_DEPTH depth of PNTS files
  constexpr uint32_t MAX_DEPTH = 2;

  std::vector<Tileset const*> roots;
  std::queue<Tileset const*> working_queue;

  working_queue.push(&*_root_tileset);

  while (!working_queue.empty()) {
    auto current_root = working_queue.front();
    working_queue.pop();

    roots.push_back(current_root);

    // Find children that are MAX_DEPTH away from current_root
    iterate_tileset_children(*current_root, working_queue, MAX_DEPTH);
  }

  tf::Taskflow taskflow;
  parallel::for_each(
    std::begin(roots),
    std::end(roots),
    [this](Tileset const* root) {
      const auto filepath = concat(_work_dir, "/", root->name, ".json");
      writeTilesetJSON(filepath, *root, MAX_DEPTH + 1);
    },
    taskflow,
    std::thread::hardware_concurrency());

  tf::Executor executor{ std::thread::hardware_concurrency() };
  executor.run(taskflow).wait();
}

bool
Cesium3DTilesPersistence::node_exists(const std::string& node_name) const
{
  const auto file_path = concat(_work_dir, "/", node_name, ".pnts");
  return fs::exists(file_path);
}