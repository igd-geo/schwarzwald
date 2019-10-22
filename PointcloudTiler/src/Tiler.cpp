#include "Tiler.h"

#include "PNTSReader.h"
#include "PNTSWriter.h"
#include "PointBuffer.h"
#include "TileSetWriter.h"
#include "Tileset.h"
#include "definitions.hpp"
#include "io/IPointsPersistence.h"
#include "octree/OctreeAlgorithms.h"
#include "octree/OctreeIndexWriter.h"
#include "stuff.h"
#include "ui/ProgressReporter.h"
#include "util/Async.h"

#include <algorithm>
#include <cmath>
#include <experimental/filesystem>
#include <functional>
#include <future>
#include <iomanip>
#include <numeric>
#include <queue>
#include <stdint.h>
#include <unordered_map>

constexpr size_t MAX_POINTS_CACHED_BEFORE_INDEXING = 10'000'000;
constexpr uint32_t MAX_OCTREE_LEVELS = 21;

/**
 * Description parameters for processing a specific octree node
 */
struct OctreeNodeDescription
{
  MortonIndex<MAX_OCTREE_LEVELS> key;
  int32_t level; // root = -1, since it is only a single node it doesn't make sense
                 // to have root be level 0, especially given the MortonIndex
  AABB bounds;
  std::vector<IndexedPoint<MAX_OCTREE_LEVELS>> indexed_points;
  std::string root_node_name;
  uint32_t max_depth;
};

// TODO Quick and dirty hack for the Tilesets
std::unordered_map<std::string, std::unique_ptr<Tileset>> s_tilesets;
std::mutex s_tilesets_lock;
std::mutex s_points_cache_lock;

static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
merge_indexed_points_sorted(std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>&& l,
                            std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>&& r)
{
  if (l.empty())
    return r;
  if (r.empty())
    return l;

  std::vector<IndexedPoint<MAX_OCTREE_LEVELS>> merged;
  merged.reserve(l.size() + r.size());
  std::merge(l.begin(),
             l.end(),
             r.begin(),
             r.end(),
             std::back_inserter(merged),
             [](const auto& idx_l, const auto& idx_r) {
               return idx_l.morton_index.get() < idx_r.morton_index.get();
             });
  return merged;
}

static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
merge_indexed_points_unsorted(std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>&& l,
                              std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>&& r)
{
  if (l.empty())
    return r;
  if (r.empty())
    return l;

  l.insert(l.end(), r.begin(), r.end());
  return l;
}

template<typename Iter>
static void
flush_indices(Iter indexed_points_begin,
              Iter indexed_points_end,
              const std::string& node_name,
              IPointsPersistence& persistence)
{
  std::vector<MortonIndex<MAX_OCTREE_LEVELS>> indices;
  const auto num_points_selected = std::distance(indexed_points_begin, indexed_points_end);
  indices.reserve(num_points_selected);
  std::transform(indexed_points_begin,
                 indexed_points_end,
                 std::back_inserter(indices),
                 [](const auto& indexed_point) { return indexed_point.morton_index; });

  persistence.persist_indices(gsl::make_span(indices), node_name);
}

static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
read_pnts_from_disk(const std::string& node_name,
                    const AABB& octree_bounds,
                    const AABB& node_bounds,
                    std::vector<std::unique_ptr<PointBuffer>>& points_cache,
                    IPointsPersistence& persistence)
{
  PointBuffer tmp_points;
  persistence.retrieve_points(node_name, tmp_points);
  if (!tmp_points.count())
    return {};

  auto new_points_buffer = std::make_unique<PointBuffer>(std::move(tmp_points));
  auto& points = *new_points_buffer;

  // Storing points in LAS/LAZ files introduced quantization, we try to fix this
  // here by clamping points into the bounding box of their octree node
  for (auto& position : tmp_points.positions()) {
    if (!node_bounds.isInside(position)) {
      position.x = std::min(node_bounds.max.x, std::max(node_bounds.min.x, position.x));
      position.y = std::min(node_bounds.max.y, std::max(node_bounds.min.y, position.x));
      position.z = std::min(node_bounds.max.z, std::max(node_bounds.min.z, position.x));
      // std::cout
      //     << std::setprecision(16) << "At node " << current_node_name
      //     << " the point [" << pos.x << ";" << pos.y << ";" << pos.z
      //     << "] was not contained "
      //     << "in the nodes bounding box (" << node_description.bounds.min.x
      //     << ";" << node_description.bounds.min.y << ";"
      //     << node_description.bounds.min.z << ") x ("
      //     << node_description.bounds.max.x << ";"
      //     << node_description.bounds.max.y << ";"
      //     << node_description.bounds.max.z << ") after reading the "
      //     << "point back from disk! This is a bug, terminating the program
      //     now!"
      //     << std::endl;
      // std::terminate();
    }
  }

  s_points_cache_lock.lock();
  points_cache.push_back(std::move(new_points_buffer));
  s_points_cache_lock.unlock();

  std::vector<MortonIndex<MAX_OCTREE_LEVELS>> octree_indices;
  octree_indices.reserve(points.count());
  calculate_morton_indices_for_points<MAX_OCTREE_LEVELS>(points.positions().begin(),
                                                         points.positions().end(),
                                                         std::back_inserter(octree_indices),
                                                         octree_bounds);
  // persistence.retrieve_indices(node_name, octree_indices);

  // Points are still sorted here
  return index_points(octree_indices, points);
}

static int32_t
get_node_level_for_target_spacing(float spacing_at_root,
                                  int32_t target_node_level,
                                  const AABB& root_node_bounds)
{
  // Returns the last level (starting from root) at which the node size is >=
  // target spacing. Target spacing is calculated from root spacing by halfing
  // at each level Since the root node is level -1, spacing at level 0 is half
  // spacing_at_root
  const auto spacing_at_target_node = spacing_at_root / std::pow(2, target_node_level + 1);

  return std::max(
    -1,
    (int)std::floor(std::log2f(root_node_bounds.extent().x / spacing_at_target_node)) -
      1); // the root node (whole octree) is level '-1', so
          // level 0 has a sidelength of half the max octree,
          // hence we have to subtract one here
}

static void
process_terminal_node(const std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>& all_points,
                      const std::string& node_name,
                      const AABB& node_bounds,
                      size_t max_points_per_node,
                      size_t previously_taken_points_count,
                      IPointsPersistence& persistence,
                      ProgressReporter* progress_reporter)
{
  const auto num_points = all_points.size();
  const auto points_to_take = std::min(num_points, max_points_per_node);
  const auto partition_point = all_points.begin() + points_to_take;

  if (partition_point != all_points.end()) {
    const auto dropped_count = all_points.size() - points_to_take;
    if (progress_reporter)
      progress_reporter->increment_progress<size_t>(progress::INDEXING, dropped_count);
    std::cout << "Dropping " << dropped_count << " points at node " << node_name << "!"
              << std::endl;

    std::vector<PointBuffer::PointReference> dropped_points;
    dropped_points.reserve(dropped_count);
    std::transform(partition_point,
                   all_points.end(),
                   std::back_inserter(dropped_points),
                   [](const auto& indexed_point) { return indexed_point.point_reference; });
    persistence.persist_points(gsl::make_span(dropped_points), node_bounds, node_name + "_dropped");
  }

  std::vector<PointBuffer::PointReference> selected_points;
  std::vector<MortonIndex<MAX_OCTREE_LEVELS>> selected_indices;
  selected_points.reserve(points_to_take);
  selected_indices.reserve(points_to_take);
  std::for_each(all_points.begin(), partition_point, [&](const auto& indexed_point) {
    selected_points.push_back(indexed_point.point_reference);
    selected_indices.push_back(indexed_point.morton_index);
  });

  persistence.persist_points(gsl::make_span(selected_points), node_bounds, node_name);
  // persistence.persist_indices(gsl::make_span(selected_indices), node_name);

  if (progress_reporter)
    progress_reporter->increment_progress<size_t>(progress::INDEXING,
                                                  points_to_take - previously_taken_points_count);
}

static std::vector<OctreeNodeDescription>
process_internal_node(std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>& all_points,
                      const std::string& node_name,
                      const AABB& node_bounds,
                      MortonIndex<MAX_OCTREE_LEVELS> node_key,
                      int32_t node_level,
                      const AABB& root_node_bounds,
                      const std::string& root_node_name, // TODO Refactor this into Node structures
                                                         // and pass 'currentNode' and 'rootNode'
                      float spacing_at_root,
                      size_t previously_taken_points_count,
                      uint32_t max_depth,
                      SamplingStrategy& sampling_strategy,
                      IPointsPersistence& persistence,
                      ProgressReporter* progress_reporter)
{
  // Use sampling strategy to select points
  const auto partition_point = filter_points_for_octree_node(all_points.begin(),
                                                             all_points.end(),
                                                             node_key,
                                                             node_level,
                                                             root_node_bounds,
                                                             spacing_at_root,
                                                             sampling_strategy);

  // Persist the points
  const auto points_taken = std::distance(all_points.begin(), partition_point);
  std::vector<PointBuffer::PointReference> selected_points;
  std::vector<MortonIndex<MAX_OCTREE_LEVELS>> selected_indices;
  selected_points.reserve(points_taken);
  selected_indices.reserve(points_taken);
  std::for_each(all_points.begin(), partition_point, [&](const auto& indexed_point) {
    selected_points.push_back(indexed_point.point_reference);
    selected_indices.push_back(indexed_point.morton_index);
  });

  persistence.persist_points(gsl::make_span(selected_points), node_bounds, node_name);
  // persistence.persist_indices(gsl::make_span(selected_indices), node_name);

  if (progress_reporter)
    progress_reporter->increment_progress<size_t>(progress::INDEXING,
                                                  points_taken - previously_taken_points_count);

  // Partition remaining points into the 8 child octants
  const auto child_level = static_cast<uint32_t>(node_level + 1);
  const auto child_ranges =
    partition_points_into_child_octants(partition_point, all_points.end(), child_level);
  std::vector<OctreeNodeDescription> child_descriptions;
  for (uint8_t idx = 0; idx < uint8_t(8); ++idx) {
    const auto& child_range = child_ranges.at(idx);
    if (child_range.first == child_range.second)
      continue;

    auto child_key = node_key;
    child_key.set_octant_at_level(child_level, idx);

    // Copy the indices into a new vector and pass it on to the child processing
    std::vector<IndexedPoint<MAX_OCTREE_LEVELS>> child_indexed_points{ child_range.first,
                                                                       child_range.second };

    const auto child_bounds = get_octant_bounds(idx, node_bounds);

    child_descriptions.push_back(OctreeNodeDescription{ child_key,
                                                        static_cast<int32_t>(child_level),
                                                        child_bounds,
                                                        std::move(child_indexed_points),
                                                        root_node_name,
                                                        max_depth });
  }

  return child_descriptions;
}

static std::vector<OctreeNodeDescription>
process_octree_node(OctreeNodeDescription&& node_description,
                    std::vector<std::unique_ptr<PointBuffer>>& points_cache,
                    const AABB& root_bounds,
                    float spacing_at_root,
                    size_t max_points_per_node,
                    SamplingStrategy& sampling_strategy,
                    ProgressReporter* progress_reporter,
                    IPointsPersistence& persistence)
{
  const auto current_node_name =
    node_description.root_node_name + to_string(node_description.key, node_description.level + 1);

  auto cached_points = read_pnts_from_disk(
    current_node_name, root_bounds, node_description.bounds, points_cache, persistence);
  const auto cached_points_count = cached_points.size();

  const auto node_level_to_sample_from =
    get_node_level_for_target_spacing(spacing_at_root, node_description.level, root_bounds);

  // Depending on the level that we want to sample from, we have to do different
  // things

  if (node_level_to_sample_from >= static_cast<int32_t>(node_description.max_depth)) {
    // If we are at the max level or even deeper, we take all the points we can
    // and are done for this node
    const auto all_points_for_this_node = merge_indexed_points_unsorted(
      std::move(node_description.indexed_points), std::move(cached_points));

    process_terminal_node(all_points_for_this_node,
                          current_node_name,
                          node_description.bounds,
                          max_points_per_node,
                          cached_points_count,
                          persistence,
                          progress_reporter);

    return {}; // Done for this node, no more children
  }

  if (node_level_to_sample_from >= static_cast<int32_t>(MAX_OCTREE_LEVELS - 1)) {
    // If we are so deep that we exceed the capacity of the MortonIndex, we
    // have to index our points again with the current node as new root node. We
    // also have to carry the information that we have a new root over to the
    // children so that the paths of the nodes are correct.

    // Fun fact: We don't have to adjust the loaded indices because if we ever
    // get to a node this deep again, the indices have been calculated with the
    // new root the last time also, so everything is as it should be
    auto all_points_for_this_node = merge_indexed_points_unsorted(
      std::move(node_description.indexed_points), std::move(cached_points));

    // Set this node as the new root node
    const auto new_max_depth = node_description.max_depth - node_description.level;
    const auto new_root_node_name = current_node_name;
    const auto new_root_bounds = node_description.bounds;
    const auto new_spacing_at_root = spacing_at_root / std::pow(2, node_description.level + 1);

    // Compute new indices based upon this node as root node
    for (auto& indexed_point : all_points_for_this_node) {
      indexed_point.morton_index = calculate_morton_index<MAX_OCTREE_LEVELS>(
        indexed_point.point_reference.position(), new_root_bounds);
    }

    // Make sure everything is sorted again
    std::sort(
      all_points_for_this_node.begin(),
      all_points_for_this_node.end(),
      [](const auto& l, const auto& r) { return l.morton_index.get() < r.morton_index.get(); });

    return process_internal_node(all_points_for_this_node,
                                 current_node_name,
                                 node_description.bounds,
                                 node_description.key,
                                 -1, // this node is new root, so its level is -1
                                 new_root_bounds,
                                 new_root_node_name,
                                 new_spacing_at_root,
                                 cached_points_count,
                                 new_max_depth,
                                 sampling_strategy,
                                 persistence,
                                 progress_reporter);
  }

  auto all_points_for_this_node = merge_indexed_points_sorted(
    std::move(node_description.indexed_points), std::move(cached_points));

  return process_internal_node(all_points_for_this_node,
                               current_node_name,
                               node_description.bounds,
                               node_description.key,
                               node_description.level,
                               root_bounds,
                               node_description.root_node_name,
                               spacing_at_root,
                               cached_points_count,
                               node_description.max_depth,
                               sampling_strategy,
                               persistence,
                               progress_reporter);
}

template<typename Func>
static void
process_node_sync(OctreeNodeDescription&& node_description, Func&& process_call)
{
  auto child_nodes = process_call(std::move(node_description));

  for (auto& child_node : child_nodes) {
    process_node_sync(std::move(child_node), process_call);
  }
}

template<typename Func>
static async::Awaitable<void>
process_node_async(OctreeNodeDescription&& node_description,
                   Func&& process_call,
                   TaskSystem& task_system)
{
  auto child_nodes = process_call(std::move(node_description));

  constexpr size_t MinPointsForAsyncProcessing = 100'000;

  // Sort child nodes by point cound in descending order. This way, we first process
  // all nodes that require async processing and then process the sync nodes
  std::sort(std::begin(child_nodes), std::end(child_nodes), [](const auto& l, const auto& r) {
    return r.indexed_points.size() < l.indexed_points.size();
  });

  std::vector<async::Awaitable<void>> awaitables;
  awaitables.reserve(child_nodes.size());
  for (auto& child_node : child_nodes) {
    if (child_node.indexed_points.size() >= MinPointsForAsyncProcessing) {
      auto sub_awaitable =
        task_system.push([&, process_call, child_node{ std::move(child_node) }]() mutable {
          return process_node_async(std::move(child_node), process_call, task_system);
        });
      awaitables.push_back(async::flatten(std::move(sub_awaitable)));
    } else {
      process_node_sync(std::move(child_node), process_call);
    }
  }

  return async::all(std::move(awaitables));
}

Tiler::Tiler(const AABB& aabb,
             float spacing_at_root,
             uint32_t max_depth,
             size_t max_points_per_node,
             SamplingStrategy sampling_strategy,
             ProgressReporter* progress_reporter,
             IPointsPersistence& persistence)
  : _aabb(aabb)
  , _spacing(spacing_at_root)
  , _max_depth(max_depth)
  , _max_points_per_node(max_points_per_node)
  , _sampling_strategy(std::move(sampling_strategy))
  , _progress_reporter(progress_reporter)
  , _persistence(persistence)
  , _producers(0)
  , _consumers(1)
  , _run_indexing_thread(true)
{
  const auto root_spacing_to_bounds_ratio = std::log2f(aabb.extent().x / spacing_at_root);
  if (root_spacing_to_bounds_ratio >= MAX_OCTREE_LEVELS) {
    throw std::runtime_error{ "spacing at root node is too small compared to bounds of data!" };
  }

  _indexing_thread = std::thread([this]() { run_worker(); });
}

void
Tiler::index()
{
  _consumers.wait();
  _indexing_point_cache = std::move(_store);
  _store = {};
  _producers.notify();
}

void
Tiler::wait_until_indexed()
{}

bool
Tiler::needs_indexing() const
{
  return _store.count() >= MAX_POINTS_CACHED_BEFORE_INDEXING;
}

void
Tiler::cache(const PointBuffer& points)
{
  _store.append_buffer(points);
  if (_progress_reporter)
    _progress_reporter->increment_progress<size_t>(progress::LOADING, points.count());
}

void
Tiler::flush()
{
  if (!_store.count())
    return;
  index();
}

bool
Tiler::needs_flush() const
{
  return false;
}

void
Tiler::close()
{
  _consumers.wait();
  _run_indexing_thread = false;
  _producers.notify();

  _indexing_thread.join();
}

void
Tiler::run_worker()
{
  TaskSystem task_system;
  // Use max_threads - 1 because one thread is reserved for reading points
  const auto concurrency = std::max(1u, std::thread::hardware_concurrency() - 1);
  task_system.run(concurrency);

  auto run_async = [&](const auto& indexed_points) {
    // Async version

    std::vector<std::unique_ptr<PointBuffer>> points_cache;

    const auto process_call = [&](OctreeNodeDescription&& node_desc) {
      return process_octree_node(std::move(node_desc),
                                 points_cache,
                                 _aabb,
                                 _spacing,
                                 _max_points_per_node,
                                 _sampling_strategy,
                                 _progress_reporter,
                                 _persistence);
    };

    OctreeNodeDescription root_node_desc{ MortonIndex<MAX_OCTREE_LEVELS>{}, -1,  _aabb,
                                          std::move(indexed_points),        "r", _max_depth };

    auto awaitable = process_node_async(std::move(root_node_desc), process_call, task_system);
    awaitable.await();
  };

  while (_run_indexing_thread) {
    _producers.wait();
    if (!_run_indexing_thread)
      break;

    std::vector<MortonIndex<MAX_OCTREE_LEVELS>> octree_keys;
    octree_keys.resize(_indexing_point_cache.count());

    // Calculate morton indices in parallel
    {
      std::vector<async::Awaitable<void>> futures;
      futures.reserve(concurrency);

      const auto keys_chunks =
        split_range_into_chunks(concurrency, octree_keys.begin(), octree_keys.end());
      const auto positions_chunks =
        split_range_into_chunks(concurrency,
                                _indexing_point_cache.positions().begin(),
                                _indexing_point_cache.positions().end());

      // This is parallel_for_each
      for (size_t idx = 0; idx < concurrency; ++idx) {
        const auto cur_points_chunk_begin = positions_chunks[idx].first;
        const auto cur_points_chunk_end = positions_chunks[idx].second;
        const auto cur_key_chunk_begin = keys_chunks[idx].first;
        futures.push_back(task_system.push([=]() {
          calculate_morton_indices_for_points<MAX_OCTREE_LEVELS>(
            cur_points_chunk_begin, cur_points_chunk_end, cur_key_chunk_begin, _aabb);
        }));
      }

      async::all(std::move(futures)).await();
    }

    auto indexed_points = index_points(octree_keys, _indexing_point_cache);

    // Important to sort all points by their octree keys
    std::sort(indexed_points.begin(), indexed_points.end(), [](const auto& l, const auto& r) {
      return l.morton_index.get() < r.morton_index.get();
    });

    // Process root
    // Partition into 8 child ranges
    // Process each child range
    // Partition again etc.
    // Make sure to sync threads at each level, no higher level must be
    // processed before lower levels

    run_async(indexed_points);

    _consumers.notify();
  }

  task_system.stop_and_join();
}