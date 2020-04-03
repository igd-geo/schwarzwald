#include "Tiler.h"

#include "datastructures/PointBuffer.h"
#include "io/PNTSReader.h"
#include "io/PNTSWriter.h"
#include "io/TileSetWriter.h"
#include "octree/Node.h"
#include "octree/OctreeAlgorithms.h"
#include "octree/OctreeIndexWriter.h"
#include "pointcloud/Tileset.h"
#include "util/ExecutorObserver.h"
#include "util/stuff.h"
#include <containers/DestructuringIterator.h>
#include <debug/Journal.h>
#include <debug/ProgressReporter.h>
#include <terminal/stdout_helper.h>
#include <threading/Parallel.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/scope_exit.hpp>
#include <cmath>
#include <experimental/filesystem>
#include <functional>
#include <future>
#include <iomanip>
#include <map>
#include <numeric>
#include <queue>
#include <stdint.h>
#include <unordered_map>

#include <taskflow/taskflow.hpp>

constexpr uint32_t MAX_OCTREE_LEVELS = 21;

struct NodeProcessingResult {
  NodeProcessingResult() {}
  NodeProcessingResult(octree::NodeData data, octree::NodeStructure node,
                       octree::NodeStructure root_node)
      : data(std::move(data)), node(node), root_node(root_node) {}

  octree::NodeData data;
  octree::NodeStructure node;
  octree::NodeStructure root_node;
};

#ifdef PRECISE_VERIFICATION_MODE
static void
validate_points(const std::vector<IndexedPoint<MAX_OCTREE_LEVELS>> &points,
                const std::string &node_name, const AABB &bounds) {
  for (size_t idx = 0; idx < points.size(); ++idx) {
    const auto &p = points[idx];
    const auto &pos = p.point_reference.position();
    if (!bounds.isInside(pos)) {
      std::cerr << "validate_points failed @ node " << node_name << "("
                << bounds << ")!\n"
                << "\tPoint " << pos << " is outside of bounding box!\n";
      std::exit(EXIT_FAILURE);
    }
  }
}

template <typename Iter>
static void sorted_check(Iter begin, Iter end, const std::string &node_name) {
  if (!std::is_sorted(begin, end, [](const auto &l, const auto &r) {
        return l.morton_index.get() < r.morton_index.get();
      })) {
    std::cerr << "sorted_check failed @ node " << node_name << "!\n";
    std::exit(EXIT_FAILURE);
  }
}

template <typename Iter>
static void morton_check(Iter begin, Iter end, uint32_t level,
                         const std::string &node_name,
                         const AABB &node_bounds) {
  for (auto next = begin + 1; next < end; ++next) {
    const auto next_morton_idx = next->morton_index.truncate_to_level(level);
    const auto cur_morton_idx =
        (next - 1)->morton_index.truncate_to_level(level);
    if (cur_morton_idx.get() != next_morton_idx.get()) {
      std::cerr << "morton_check failed @ node " << node_name << "("
                << node_bounds << ")!\n"
                << "\tPoint A: " << (next - 1)->point_reference.position()
                << " with Morton index " << to_string(cur_morton_idx) << "\n"
                << "\tPoint B: " << next->point_reference.position()
                << " with Morton index " << to_string(next_morton_idx) << "\n";
      std::exit(EXIT_FAILURE);
    }
  }
}

template <typename Iter>
static void inside_check(Iter begin, Iter end, const std::string &node_name,
                         const AABB &bounds) {
  std::for_each(begin, end, [&](const auto &indexed_point) {
    const auto &pos = indexed_point.point_reference.position();
    if (!bounds.isInside(pos)) {
      std::cerr << "validate_points failed @ node " << node_name << "("
                << bounds << ")!\n"
                << "\tPoint " << pos << " is outside of bounding box!\n";
      std::exit(EXIT_FAILURE);
    }
  });
}
#endif

std::mutex s_points_cache_lock;

static std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>
read_pnts_from_disk(const std::string &node_name, const AABB &octree_bounds,
                    const AABB &node_bounds,
                    std::vector<std::unique_ptr<PointBuffer>> &points_cache,
                    PointsPersistence &persistence) {
  PointBuffer tmp_points;
  persistence.retrieve_points(node_name, tmp_points);
  if (!tmp_points.count())
    return {};

  auto new_points_buffer = std::make_unique<PointBuffer>(std::move(tmp_points));
  auto &points = *new_points_buffer;

  s_points_cache_lock.lock();
  points_cache.push_back(std::move(new_points_buffer));
  s_points_cache_lock.unlock();

  std::vector<IndexedPoint<MortonIndex64Levels>> indexed_points;
  indexed_points.reserve(points.count());
  try {
    index_points<MortonIndex64Levels>(
        std::begin(points), std::end(points),
        std::back_inserter(indexed_points), octree_bounds,
        OutlierPointsBehaviour::ClampToBounds); // Clamp to fix floating point
                                                // errors
  } catch (const std::runtime_error &ex) {
    throw std::runtime_error{
        (boost::format("Indexing points of node %1% failed:\n%2%") % node_name %
         ex.what())
            .str()};
  }

  return indexed_points;
}

static void process_terminal_node(octree::NodeData const &all_points,
                                  octree::NodeStructure const &node,
                                  size_t max_points_per_node,
                                  size_t previously_taken_points_count,
                                  PointsPersistence &persistence,
                                  ProgressReporter *progress_reporter) {
  const auto num_points = all_points.size();
  const auto points_to_take = std::min(num_points, max_points_per_node);
  const auto partition_point = all_points.begin() + points_to_take;

  if (partition_point != all_points.end()) {
    const auto dropped_count = all_points.size() - points_to_take;
    if (progress_reporter)
      progress_reporter->increment_progress<size_t>(progress::INDEXING,
                                                    dropped_count);

    util::write_log(concat("Dropping ", dropped_count, " points at node ",
                           node.name, "!\n"));

    persistence.persist_points(
        member_iterator(partition_point, &IndexedPoint64::point_reference),
        member_iterator(std::end(all_points), &IndexedPoint64::point_reference),
        node.bounds, node.name + "_dropped");
  }

  persistence.persist_points(
      member_iterator(std::begin(all_points), &IndexedPoint64::point_reference),
      member_iterator(partition_point, &IndexedPoint64::point_reference),
      node.bounds, node.name);

  if (progress_reporter)
    progress_reporter->increment_progress<size_t>(
        progress::INDEXING, points_to_take - previously_taken_points_count);
}

static std::vector<NodeProcessingResult> process_internal_node(
    octree::NodeData &all_points, octree::NodeStructure const &node,
    octree::NodeStructure const &root_node,
    size_t previously_taken_points_count, SamplingStrategy &sampling_strategy,
    PointsPersistence &persistence, ProgressReporter *progress_reporter,
    std::map<std::string, std::string> &timings) {

  // Use sampling strategy to select points
  const auto filter_tbegin = std::chrono::high_resolution_clock::now();

  const auto partition_point = filter_points_for_octree_node(
      all_points.begin(), all_points.end(), node.morton_index, node.level,
      root_node.bounds, root_node.max_spacing, sampling_strategy);
  const auto points_taken =
      std::distance(std::begin(all_points), partition_point);

  if (debug::Journal::instance().is_enabled()) {
    timings["filter_points_for_octree_node"] = std::to_string(
        (std::chrono::high_resolution_clock::now() - filter_tbegin).count());
  }

#ifdef PRECISE_VERIFICATION_MODE
  sorted_check(std::begin(all_points), partition_point,
               node_name + " (taken points)");
  sorted_check(partition_point, all_points_end,
               node_name + " (not taken points)");
#endif

  // Persist the points
  const auto persist_tbegin = std::chrono::high_resolution_clock::now();
  persistence.persist_points(
      destructure_iterator(std::begin(all_points),
                           &IndexedPoint64::point_reference),
      destructure_iterator(partition_point, &IndexedPoint64::point_reference),
      node.bounds, node.name);
  if (debug::Journal::instance().is_enabled()) {
    timings["persist_points"] = std::to_string(
        (std::chrono::high_resolution_clock::now() - persist_tbegin).count());
    timings["point_count_persist"] = std::to_string(points_taken);
  }

#ifdef PRECISE_VERIFICATION_MODE
  inside_check(all_points.begin(), partition_point, node_name, node_bounds);
#endif

  if (progress_reporter)
    progress_reporter->increment_progress<size_t>(
        progress::INDEXING, points_taken - previously_taken_points_count);

  // Partition remaining points into the 8 child octants
  const auto child_partitioning_tstart =
      std::chrono::high_resolution_clock::now();
  const auto child_level = static_cast<uint32_t>(node.level + 1);
  const auto child_ranges = partition_points_into_child_octants(
      partition_point, all_points.end(), child_level);

  // TODO We could re-use the all_points vector to save some
  // copying/allocations?!
  std::vector<NodeProcessingResult> child_descriptions;
  for (uint8_t idx = 0; idx < uint8_t(8); ++idx) {
    const auto &child_range = child_ranges.at(idx);
    if (child_range.first == child_range.second)
      continue;

    auto child_node = node;
    child_node.morton_index.set_octant_at_level(child_level, idx);
    child_node.bounds = get_octant_bounds(idx, node.bounds);
    child_node.level = child_level;
    child_node.max_spacing /= 2;
    child_node.name = concat(node.name, static_cast<char>('0' + idx));

    // Copy the indices into a new vector and pass it on to the child processing
    child_descriptions.emplace_back(
        octree::NodeData{child_range.first, child_range.second}, child_node,
        root_node);

#ifdef PRECISE_VERIFICATION_MODE
    validate_points(child_descriptions.back().indexed_points,
                    node_name + (char)('0' + idx) + " (child)",
                    child_descriptions.back().bounds);
#endif
  }

  if (debug::Journal::instance().is_enabled()) {
    timings["partition_child_octants"] = std::to_string(
        (std::chrono::high_resolution_clock::now() - child_partitioning_tstart)
            .count());
  }

  return child_descriptions;
}

static std::vector<NodeProcessingResult> process_octree_node(
    octree::NodeData &&node_data, octree::NodeStructure const &node,
    octree::NodeStructure const &root_node,
    std::vector<std::unique_ptr<PointBuffer>> &points_cache,
    size_t max_points_per_node, SamplingStrategy &sampling_strategy,
    ProgressReporter *progress_reporter, PointsPersistence &persistence) {
  std::map<std::string, std::string> timings;
  static auto s_journal_headers_written = false;
  BOOST_SCOPE_EXIT(&timings, &node) {
    if (!debug::Journal::instance().is_enabled())
      return;

    if (!s_journal_headers_written) {
      debug::Journal::instance().add_entry(
          boost::algorithm::join(keys(timings), ";"));

      s_journal_headers_written = true;
    }

    debug::Journal::instance().add_entry(
        boost::algorithm::join(values(timings), ";"));
  }
  BOOST_SCOPE_EXIT_END

  const auto t_begin = std::chrono::high_resolution_clock::now();

  BOOST_SCOPE_EXIT(&timings, &t_begin, &node) {
    if (!debug::Journal::instance().is_enabled())
      return;

    timings["process_octree_node"] = std::to_string(
        (std::chrono::high_resolution_clock::now() - t_begin).count());
    timings["name"] = node.name;
  }
  BOOST_SCOPE_EXIT_END

  const auto current_node_name =
      concat(root_node.name, to_string(node.morton_index, node.level + 1));

  auto cached_points =
      read_pnts_from_disk(current_node_name, root_node.bounds, node.bounds,
                          points_cache, persistence);

  if (debug::Journal::instance().is_enabled()) {
    timings["read_pnts_from_disk"] = std::to_string(
        (std::chrono::high_resolution_clock::now() - t_begin).count());

    timings["point_count_disk"] = std::to_string(cached_points.size());
    timings["point_count_new"] = std::to_string(node_data.size());
    timings["point_count_total"] =
        std::to_string(cached_points.size() + node_data.size());
  }

  const auto cached_points_count = cached_points.size();

#ifdef PRECISE_VERIFICATION_MODE
  validate_points(node_description.indexed_points, current_node_name,
                  node_description.bounds);
  validate_points(cached_points, current_node_name + " (cached)",
                  node_description.bounds);
  sorted_check(std::begin(cached_points), std::end(cached_points),
               current_node_name + " (cached)");
  sorted_check(std::begin(node_description.indexed_points),
               std::end(node_description.indexed_points), current_node_name);
  if (node_description.level >= 0) {
    morton_check(std::begin(node_description.indexed_points),
                 std::end(node_description.indexed_points),
                 static_cast<uint32_t>(node_description.level),
                 current_node_name, node_description.bounds);
    morton_check(std::begin(cached_points), std::end(cached_points),
                 static_cast<uint32_t>(node_description.level),
                 current_node_name + " (cached)", node_description.bounds);
  }
#endif

  // TODO Only certain sampling modes (GRID_CENTER, RANDOM_SORTED_GRID) need to
  // know the target level to sample from. This means that for all other
  // sampling modes, we could rather check the level of this node (instead of
  // the level to sample from) and base the distinction interior/terminal node
  // on this

  const auto node_level_to_sample_from =
      octree::get_node_level_to_sample_from(node.level, root_node);

  // Depending on the level that we want to sample from, we have to do different
  // things

  if (node_level_to_sample_from >= static_cast<int32_t>(node.max_depth)) {
    // If we are at the max level or even deeper, we take all the points we can
    // and are done for this node
    const auto all_points_for_this_node = octree::merge_node_data_unsorted(
        std::move(node_data), std::move(cached_points));

    process_terminal_node(all_points_for_this_node, node, max_points_per_node,
                          cached_points_count, persistence, progress_reporter);

    return {}; // Done for this node, no more children
  }

  if (node_level_to_sample_from >=
      static_cast<int32_t>(MAX_OCTREE_LEVELS - 1)) {
    // If we are so deep that we exceed the capacity of the MortonIndex, we
    // have to index our points again with the current node as new root node. We
    // also have to carry the information that we have a new root over to the
    // children so that the paths of the nodes are correct.

    // Fun fact: We don't have to adjust the loaded indices because if we ever
    // get to a node this deep again, the indices have been calculated with the
    // new root the last time also, so everything is as it should be
    auto all_points_for_this_node = octree::merge_node_data_unsorted(
        std::move(node_data), std::move(cached_points));

    // Set this node as the new root node
    auto new_root_node = node;
    new_root_node.max_depth = node.max_depth - node.level;

    // Compute new indices based upon this node as root node
    for (auto &indexed_point : all_points_for_this_node) {
      indexed_point.morton_index = calculate_morton_index<MAX_OCTREE_LEVELS>(
          indexed_point.point_reference.position(), new_root_node.bounds);
    }

    // Make sure everything is sorted again
    std::sort(all_points_for_this_node.begin(), all_points_for_this_node.end(),
              [](const auto &l, const auto &r) {
                return l.morton_index.get() < r.morton_index.get();
              });

    return process_internal_node(all_points_for_this_node, node, new_root_node,
                                 cached_points_count, sampling_strategy,
                                 persistence, progress_reporter, timings);
  }

  auto all_points_for_this_node = octree::merge_node_data_sorted(
      std::move(node_data), std::move(cached_points));

#ifdef PRECISE_VERIFICATION_MODE
  sorted_check(std::begin(all_points_for_this_node),
               std::end(all_points_for_this_node),
               current_node_name + " (combined)");
#endif

  return process_internal_node(all_points_for_this_node, node, root_node,
                               cached_points_count, sampling_strategy,
                               persistence, progress_reporter, timings);
}

template <typename Func, typename Taskflow>
static void process_node(octree::NodeData &&node_data,
                         octree::NodeStructure const &node,
                         octree::NodeStructure const &root_node,
                         Func process_call, Taskflow &taskflow) {
  constexpr size_t MinPointsForAsyncProcessing = 100'000;

  // util::write_log(concat("Processing node ", node.name, "\n"));

  auto child_nodes = process_call(std::move(node_data), node, root_node);

  // Sort child nodes by point count in descending order. This way, we
  // first process all nodes that require async processing and then
  // process the sync nodes
  std::sort(std::begin(child_nodes), std::end(child_nodes),
            [](const auto &l, const auto &r) {
              return r.data.size() < l.data.size();
            });

  for (auto &child_node_description : child_nodes) {
    if (child_node_description.data.size() >= MinPointsForAsyncProcessing) {
      const auto child_node_name = child_node_description.node.name;
      const auto child_points_count = child_node_description.data.size();
      const auto child_task_name =
          (boost::format("%1% [%2%]") % child_node_name % child_points_count)
              .str();
      taskflow
          .emplace([_child_node = std::move(child_node_description),
                    process_call](tf::Subflow &subflow) mutable {
            // util::write_log(concat("Start async processing node ",
            // _child_node.node.name, "\n"));
            process_node(std::move(_child_node.data), _child_node.node,
                         _child_node.root_node, process_call, subflow);
          })
          .name(child_task_name);
    } else {
      process_node(std::move(child_node_description.data),
                   child_node_description.node,
                   child_node_description.root_node, process_call, taskflow);
    }
  }
}

Tiler::Tiler(const AABB &aabb, const TilerMetaParameters &meta_parameters,
             SamplingStrategy sampling_strategy,
             ProgressReporter *progress_reporter,
             PointsPersistence &persistence, fs::path output_directory)
    : _aabb(aabb), _meta_parameters(meta_parameters),
      _sampling_strategy(std::move(sampling_strategy)),
      _progress_reporter(progress_reporter), _persistence(persistence),
      _output_directory(std::move(output_directory)), _producers(0),
      _consumers(1), _run_indexing_thread(true) {
  const auto root_spacing_to_bounds_ratio =
      std::log2f(aabb.extent().x / meta_parameters.spacing_at_root);
  if (root_spacing_to_bounds_ratio >= MAX_OCTREE_LEVELS) {
    throw std::runtime_error{
        "spacing at root node is too small compared to bounds of data!"};
  }

  _indexing_thread = std::thread([this]() { run_worker(); });
}

Tiler::~Tiler() {
  if (!_run_indexing_thread)
    return;
  close();
}

void Tiler::index() {
  if (!_store.count())
    return;

  _consumers.wait();
  _indexing_point_cache = std::move(_store);
  _store = {};
  _producers.notify();
}

void Tiler::wait_until_indexed() { _consumers.wait(); }

bool Tiler::needs_indexing() const {
  return _store.count() >= _meta_parameters.internal_cache_size;
}

void Tiler::cache(const PointBuffer &points) {
  _store.append_buffer(points);
  if (_progress_reporter)
    _progress_reporter->increment_progress<size_t>(progress::LOADING,
                                                   points.count());
}

void Tiler::close() {
  _consumers.wait();
  _run_indexing_thread = false;
  _producers.notify();

  _indexing_thread.join();
}

void Tiler::run_worker() {
  // Use max_threads - 1 because one thread is reserved for reading points
  const auto concurrency =
      std::max(1u, std::thread::hardware_concurrency() - 1);

  tf::Executor executor{concurrency};
  ExecutorObserver *executor_observer = nullptr;
  if (debug::Journal::instance().is_enabled()) {
    executor_observer = executor.make_observer<ExecutorObserver>();
  }

  while (_run_indexing_thread) {
    _producers.wait();
    if (!_run_indexing_thread)
      break;

    std::vector<std::unique_ptr<PointBuffer>> points_cache;
    octree::NodeData root_node_points{_indexing_point_cache.count()};

    // Index points in parallel
    tf::Taskflow taskflow;

    taskflow.name("pointcloud_tiling");
    auto indexing_tasks = parallel::transform(
        std::begin(_indexing_point_cache), std::end(_indexing_point_cache),
        std::begin(root_node_points),
        [this](PointBuffer::PointReference point) {
          return index_point<MAX_OCTREE_LEVELS>(
              point, _aabb, OutlierPointsBehaviour::ClampToBounds);
        },
        taskflow, concurrency, "indexing");

    auto sort_task =
        taskflow
            .emplace([&root_node_points]() {
              std::sort(root_node_points.begin(), root_node_points.end());
            })
            .name("sort");

    const auto process_call = [&](octree::NodeData &&node_data,
                                  octree::NodeStructure const &node,
                                  octree::NodeStructure const &root_node) {
      return process_octree_node(
          std::move(node_data), node, root_node, points_cache,
          _meta_parameters.max_points_per_node, _sampling_strategy,
          _progress_reporter, _persistence);
    };

    octree::NodeStructure root_node;
    root_node.bounds = _aabb;
    root_node.level = -1;
    root_node.max_depth = _meta_parameters.max_depth;
    root_node.max_spacing = _meta_parameters.spacing_at_root;
    root_node.morton_index = {};
    root_node.name = "r";

    auto process_task =
        taskflow
            .emplace([&, root_node](tf::Subflow &subflow) {
              process_node(std::move(root_node_points), root_node, root_node,
                           process_call, subflow);
              root_node_points = {};
            })
            .name(concat(root_node.name, " [", root_node_points.size(), "]"));

    indexing_tasks.second.precede(sort_task);
    sort_task.precede(process_task);

    executor.run(taskflow).wait();

    _consumers.notify();
  }

  if (debug::Journal::instance().is_enabled()) {
    std::ofstream ofs{concat(_output_directory.string(), "/taskflow.json")};
    executor_observer->dump(ofs);
  }
}