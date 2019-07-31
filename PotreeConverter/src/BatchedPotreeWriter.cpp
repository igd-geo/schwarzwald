#include "BatchedPotreeWriter.h"

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

#include <algorithm>
#include <cmath>
#include <experimental/filesystem>
#include <functional>
#include <future>
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
  OctreeNodeKey<MAX_OCTREE_LEVELS> key;
  int32_t level; // root = -1, since it is only a single node it doesn't make sense to have root be
                 // level 0, especially given the OctreeNodeKey
  Potree::AABB bounds;
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
               return idx_l.octree_node_index.get() < idx_r.octree_node_index.get();
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
  std::vector<OctreeNodeKey<MAX_OCTREE_LEVELS>> indices;
  const auto num_points_selected = std::distance(indexed_points_begin, indexed_points_end);
  indices.reserve(num_points_selected);
  std::transform(indexed_points_begin,
                 indexed_points_end,
                 std::back_inserter(indices),
                 [](const auto& indexed_point) { return indexed_point.octree_node_index; });

  persistence.persist_indices(gsl::make_span(indices), node_name);
}

static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
read_pnts_from_disk(const std::string& node_name,
                    std::vector<std::unique_ptr<Potree::PointBuffer>>& points_cache,
                    IPointsPersistence& persistence)
{
  Potree::PointBuffer tmp_points;
  persistence.retrieve_points(node_name, tmp_points);
  if (!tmp_points.count())
    return {};

  auto new_points_buffer = std::make_unique<Potree::PointBuffer>(std::move(tmp_points));
  auto& points = *new_points_buffer;

  s_points_cache_lock.lock();
  points_cache.push_back(std::move(new_points_buffer));
  s_points_cache_lock.unlock();

  std::vector<OctreeNodeKey<MAX_OCTREE_LEVELS>> octree_indices;
  persistence.retrieve_indices(node_name, octree_indices);

  // Points are still sorted here
  return index_points(octree_indices, points);
}

static int32_t
get_node_level_for_target_spacing(float spacing_at_root,
                                  int32_t target_node_level,
                                  const Potree::AABB& root_node_bounds)
{
  // Returns the last level (starting from root) at which the node size is >= target spacing. Target
  // spacing is calculated from root spacing by halfing at each level Since the root node is level
  // -1, spacing at level 0 is half spacing_at_root
  const auto spacing_at_target_node = spacing_at_root / std::pow(2, target_node_level + 1);

  return std::max(-1,
                  (int)std::floor(std::log2f(root_node_bounds.size.x / spacing_at_target_node)) -
                    1); // the root node (whole octree) is level '-1', so level 0 has a sidelength
                        // of half the max octree, hence we have to subtract one here
}

static void
process_terminal_node(const std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>& all_points,
                      const std::string& node_name,
                      size_t max_points_per_node,
                      size_t previously_taken_points_count,
                      IPointsPersistence& persistence,
                      ProgressReporter* progress_reporter)
{
  // TODO Use ISamplingStrategy
  const auto num_points = all_points.size();
  const auto points_to_take = std::min(num_points, max_points_per_node);
  const auto partition_point = all_points.begin() + points_to_take;

  if (partition_point != all_points.end()) {
    const auto dropped_count = all_points.size() - points_to_take;
    if (progress_reporter)
      progress_reporter->increment_progress<size_t>(progress::INDEXING, dropped_count);
    std::cout << "Dropping " << dropped_count << " points at node " << node_name << "!"
              << std::endl;
  }

  std::vector<Potree::PointBuffer::PointReference> selected_points;
  std::vector<OctreeNodeKey<MAX_OCTREE_LEVELS>> selected_indices;
  selected_points.reserve(points_to_take);
  selected_indices.reserve(points_to_take);
  std::for_each(all_points.begin(), partition_point, [&](const auto& indexed_point) {
    selected_points.push_back(indexed_point.point_reference);
    selected_indices.push_back(indexed_point.octree_node_index);
  });

  persistence.persist_points(gsl::make_span(selected_points), node_name);
  persistence.persist_indices(gsl::make_span(selected_indices), node_name);

  if (progress_reporter)
    progress_reporter->increment_progress<size_t>(progress::INDEXING,
                                                  points_to_take - previously_taken_points_count);
}

static std::vector<OctreeNodeDescription>
process_internal_node(std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>& all_points,
                      const std::string& node_name,
                      const Potree::AABB& node_bounds,
                      OctreeNodeKey<MAX_OCTREE_LEVELS> node_key,
                      int32_t node_level,
                      const Potree::AABB& root_bounds,
                      const std::string& root_node_name, // TODO Refactor this into Node structures
                                                         // and pass 'currentNode' and 'rootNode'
                      float spacing_at_root,
                      size_t max_points_per_node,
                      size_t previously_taken_points_count,
                      uint32_t max_depth,
                      IPointsPersistence& persistence,
                      ProgressReporter* progress_reporter)
{
  // Use sampling strategy to select points
  const auto partition_point = filter_points_for_octree_node(all_points.begin(),
                                                             all_points.end(),
                                                             root_bounds,
                                                             node_key,
                                                             node_level,
                                                             spacing_at_root,
                                                             max_points_per_node);

  // Persist the points
  const auto points_taken = std::distance(all_points.begin(), partition_point);
  std::vector<Potree::PointBuffer::PointReference> selected_points;
  std::vector<OctreeNodeKey<MAX_OCTREE_LEVELS>> selected_indices;
  selected_points.reserve(points_taken);
  selected_indices.reserve(points_taken);
  std::for_each(all_points.begin(), partition_point, [&](const auto& indexed_point) {
    selected_points.push_back(indexed_point.point_reference);
    selected_indices.push_back(indexed_point.octree_node_index);
  });

  persistence.persist_points(gsl::make_span(selected_points), node_name);
  persistence.persist_indices(gsl::make_span(selected_indices), node_name);

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
                    std::vector<std::unique_ptr<Potree::PointBuffer>>& points_cache,
                    const Potree::AABB& root_bounds,
                    float spacing_at_root,
                    size_t max_points_per_node,
                    ProgressReporter* progress_reporter,
                    IPointsPersistence& persistence)
{
  const auto current_node_name =
    node_description.root_node_name + to_string(node_description.key, node_description.level + 1);

  auto cached_points = read_pnts_from_disk(current_node_name, points_cache, persistence);
  const auto cached_points_count = cached_points.size();

  const auto node_level_to_sample_from =
    get_node_level_for_target_spacing(spacing_at_root, node_description.level, root_bounds);

  // Depending on the level that we want to sample from, we have to do different things

  if (node_level_to_sample_from >= static_cast<int32_t>(node_description.max_depth)) {
    // If we are at the max level or even deeper, we take all the points we can and are done for
    // this node
    const auto all_points_for_this_node = merge_indexed_points_unsorted(
      std::move(node_description.indexed_points), std::move(cached_points));

    process_terminal_node(all_points_for_this_node,
                          current_node_name,
                          max_points_per_node,
                          cached_points_count,
                          persistence,
                          progress_reporter);

    return {}; // Done for this node, no more children
  }

  if (node_level_to_sample_from >= static_cast<int32_t>(MAX_OCTREE_LEVELS - 1)) {
    // If we are so deep that we exceed the capacity of the OctreeNodeKey, we have to index our
    // points again with the current node as new root node. We also have to carry the information
    // that we have a new root over to the children so that the paths of the nodes are correct.

    // Fun fact: We don't have to adjust the loaded indices because if we ever get to a node this
    // deep again, the indices have been calculated with the new root the last time also, so
    // everything is as it should be
    auto all_points_for_this_node = merge_indexed_points_unsorted(
      std::move(node_description.indexed_points), std::move(cached_points));

    // Set this node as the new root node
    const auto new_max_depth = node_description.max_depth - node_description.level;
    const auto new_root_node_name = current_node_name;
    const auto new_root_bounds = node_description.bounds;
    const auto new_spacing_at_root = spacing_at_root / std::pow(2, node_description.level + 1);

    // Compute new indices based upon this node as root node
    for (auto& indexed_point : all_points_for_this_node) {
      indexed_point.octree_node_index = calculate_octree_key<MAX_OCTREE_LEVELS>(
        indexed_point.point_reference.position(), new_root_bounds);
    }

    // Make sure everything is sorted again
    std::sort(all_points_for_this_node.begin(),
              all_points_for_this_node.end(),
              [](const auto& l, const auto& r) {
                return l.octree_node_index.get() < r.octree_node_index.get();
              });

    return process_internal_node(all_points_for_this_node,
                                 current_node_name,
                                 node_description.bounds,
                                 node_description.key,
                                 -1, // this node is new root, so it's level is -1
                                 new_root_bounds,
                                 new_root_node_name,
                                 new_spacing_at_root,
                                 max_points_per_node,
                                 cached_points_count,
                                 new_max_depth,
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
                               max_points_per_node,
                               cached_points_count,
                               node_description.max_depth,
                               persistence,
                               progress_reporter);
}

Potree::BatchedPotreeWriter::BatchedPotreeWriter(const std::string& work_dir,
                                                 const AABB& aabb,
                                                 float spacing,
                                                 uint32_t maxDepth,
                                                 PointAttributes pointAttributes,
                                                 ConversionQuality quality,
                                                 const SRSTransformHelper& transform,
                                                 uint32_t max_memory_usage_MiB,
                                                 ProgressReporter* progress_reporter,
                                                 IPointsPersistence& persistence)
  : _work_dir(work_dir)
  , _aabb(aabb)
  , _spacing(spacing)
  , _max_depth(maxDepth)
  , _point_attributes(pointAttributes)
  , _transform_helper(transform)
  , _progress_reporter(progress_reporter)
  , _persistence(persistence)
  , _producers(0)
  , _consumers(1)
  , _run_indexing_thread(true)
{
  const auto root_spacing_to_bounds_ratio = std::log2f(aabb.size.x / spacing);
  if (root_spacing_to_bounds_ratio >= MAX_OCTREE_LEVELS) {
    throw std::runtime_error{ "spacing at root node is too small compared to bounds of data!" };
  }

  _indexing_thread = std::thread([this]() { run_worker(); });
}

void
Potree::BatchedPotreeWriter::index()
{
  _consumers.wait();
  _indexing_point_cache = std::move(_store);
  _store = {};
  _producers.notify();
}

void
Potree::BatchedPotreeWriter::wait_until_indexed()
{}

bool
Potree::BatchedPotreeWriter::needs_indexing() const
{
  return _store.count() >= MAX_POINTS_CACHED_BEFORE_INDEXING;
}

void
Potree::BatchedPotreeWriter::cache(const PointBuffer& points)
{
  _store.append_buffer(points);
  if (_progress_reporter)
    _progress_reporter->increment_progress<size_t>(progress::LOADING, points.count());
}

void
Potree::BatchedPotreeWriter::flush()
{
  if (!_store.count())
    return;
  index();
}

bool
Potree::BatchedPotreeWriter::needs_flush() const
{
  return false;
}

void
Potree::BatchedPotreeWriter::close()
{
  _consumers.wait();
  _run_indexing_thread = false;
  _producers.notify();

  _indexing_thread.join();
}

void
Potree::BatchedPotreeWriter::run_worker()
{

  // const auto run_sync = [&](const auto& indexed_points) {
  //   std::vector<std::unique_ptr<PointBuffer>> points_cache;

  //   std::queue<OctreeNodeDescription> nodes_to_process;
  //   nodes_to_process.push(OctreeNodeDescription{
  //     OctreeNodeKey<MAX_OCTREE_LEVELS>{}, -1, _aabb, std::move(indexed_points), "r", _max_depth
  //     });

  //   while (!nodes_to_process.empty()) {
  //     auto next_node = std::move(nodes_to_process.front());
  //     nodes_to_process.pop();

  //     auto child_nodes = process_octree_node(std::move(next_node),
  //                                            points_cache,
  //                                            _aabb,
  //                                            _spacing,
  //                                            20'000,
  //                                            _progress_reporter,
  //                                            _persistence);

  //     for (auto& child_node : child_nodes) {
  //       nodes_to_process.push(std::move(child_node));
  //     }
  //   }
  // };

  const auto run_async = [&](const auto& indexed_points) {
    // Async version

    std::vector<std::unique_ptr<PointBuffer>> points_cache;

    std::function<void(OctreeNodeDescription &&)> process_node_sync;
    process_node_sync = [&](OctreeNodeDescription&& node_desc) {
      auto child_nodes = process_octree_node(std::move(node_desc),
                                             points_cache,
                                             _aabb,
                                             _spacing,
                                             20'000,
                                             _progress_reporter,
                                             _persistence);

      for (auto& child_node : child_nodes) {
        process_node_sync(std::move(child_node));
      }
    };

    std::function<void(OctreeNodeDescription &&)> process_node_async;
    process_node_async = [&](OctreeNodeDescription&& node_desc) mutable {
      auto child_nodes = process_octree_node(std::move(node_desc),
                                             points_cache,
                                             _aabb,
                                             _spacing,
                                             20'000,
                                             _progress_reporter,
                                             _persistence);

      constexpr size_t MinPointsForAsyncProcessing = 100'000;

      std::vector<std::future<void>> futures;
      futures.reserve(child_nodes.size());
      for (auto& child_node : child_nodes) {
        if (child_node.indexed_points.size() >= MinPointsForAsyncProcessing) {
          futures.emplace_back(
            std::async(std::launch::async, process_node_async, std::move(child_node)));
        } else {
          process_node_sync(std::move(child_node)); // TODO First start all async nodes, then run
                                                    // all sync nodes, then wait for async nodes
        }
      }

      for (auto& future : futures)
        future.wait();
    };

    OctreeNodeDescription root_node_desc{ OctreeNodeKey<MAX_OCTREE_LEVELS>{}, -1,  _aabb,
                                          std::move(indexed_points),          "r", _max_depth };

    process_node_async(std::move(root_node_desc));
  };

  while (_run_indexing_thread) {
    _producers.wait();
    if (!_run_indexing_thread)
      break;

    std::vector<OctreeNodeKey<MAX_OCTREE_LEVELS>> octree_keys;
    octree_keys.resize(_indexing_point_cache.count());

    // Depending on the available amount of concurrency, either calculate the keys in serial fashion
    // or in parallel
    const auto hardware_concurrency = std::thread::hardware_concurrency();
    if (hardware_concurrency <= 2) {
      calculate_octree_keys_for_points<MAX_OCTREE_LEVELS>(_indexing_point_cache.positions().begin(),
                                                          _indexing_point_cache.positions().end(),
                                                          octree_keys.begin(),
                                                          _aabb);
    } else {
      const auto num_parallel_workers = hardware_concurrency - 2;
      const auto keys_chunks =
        split_range_into_chunks(num_parallel_workers, octree_keys.begin(), octree_keys.end());
      const auto positions_chunks =
        split_range_into_chunks(num_parallel_workers,
                                _indexing_point_cache.positions().begin(),
                                _indexing_point_cache.positions().end());

      std::vector<std::future<void>> futures;
      futures.reserve(num_parallel_workers - 1);
      for (size_t idx = 0; idx < num_parallel_workers - 1; ++idx) {
        const auto cur_points_chunk_begin = positions_chunks[idx].first;
        const auto cur_points_chunk_end = positions_chunks[idx].second;
        const auto cur_key_chunk_begin = keys_chunks[idx].first;
        futures.push_back(std::async(std::launch::async, [=]() {
          calculate_octree_keys_for_points<MAX_OCTREE_LEVELS>(
            cur_points_chunk_begin, cur_points_chunk_end, cur_key_chunk_begin, _aabb);
        }));
      }

      const auto last_points_chunk_begin = positions_chunks.back().first;
      const auto last_points_chunk_end = positions_chunks.back().second;
      const auto last_key_chunk_begin = keys_chunks.back().first;
      calculate_octree_keys_for_points<MAX_OCTREE_LEVELS>(
        last_points_chunk_begin, last_points_chunk_end, last_key_chunk_begin, _aabb);

      for (auto& future : futures)
        future.wait();
    }

    auto indexed_points = index_points(octree_keys, _indexing_point_cache);

    // Important to sort all points by their octree keys
    std::sort(indexed_points.begin(), indexed_points.end(), [](const auto& l, const auto& r) {
      return l.octree_node_index.get() < r.octree_node_index.get();
    });

    // Process root
    // Partition into 8 child ranges
    // Process each child range
    // Partition again etc.
    // Make sure to sync threads at each level, no higher level must be processed before lower
    // levels

    // run_sync(indexed_points);
    run_async(indexed_points);

    _consumers.notify();
  }
}