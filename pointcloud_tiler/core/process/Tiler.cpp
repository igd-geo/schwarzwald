#include "Tiler.h"

#include "containers/Range.h"
#include "datastructures/PointBuffer.h"
#include "io/PNTSReader.h"
#include "io/PNTSWriter.h"
#include "io/TileSetWriter.h"
#include "pointcloud/Tileset.h"
#include "tiling/Node.h"
#include "tiling/OctreeAlgorithms.h"
#include "tiling/OctreeIndexWriter.h"
#include "tiling/TilingAlgorithms.h"
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

struct NodeProcessingResult
{
  NodeProcessingResult() {}
  NodeProcessingResult(octree::NodeData data,
                       octree::NodeStructure node,
                       octree::NodeStructure root_node)
    : data(std::move(data))
    , node(node)
    , root_node(root_node)
  {}

  octree::NodeData data;
  octree::NodeStructure node;
  octree::NodeStructure root_node;
};

#ifdef PRECISE_VERIFICATION_MODE
static void
validate_points(const std::vector<IndexedPoint<MAX_OCTREE_LEVELS>>& points,
                const std::string& node_name,
                const AABB& bounds)
{
  for (size_t idx = 0; idx < points.size(); ++idx) {
    const auto& p = points[idx];
    const auto& pos = p.point_reference.position();
    if (!bounds.isInside(pos)) {
      std::cerr << "validate_points failed @ node " << node_name << "(" << bounds << ")!\n"
                << "\tPoint " << pos << " is outside of bounding box!\n";
      std::exit(EXIT_FAILURE);
    }
  }
}

template<typename Iter>
static void
sorted_check(Iter begin, Iter end, const std::string& node_name)
{
  if (!std::is_sorted(begin, end, [](const auto& l, const auto& r) {
        return l.morton_index.get() < r.morton_index.get();
      })) {
    std::cerr << "sorted_check failed @ node " << node_name << "!\n";
    std::exit(EXIT_FAILURE);
  }
}

template<typename Iter>
static void
morton_check(Iter begin,
             Iter end,
             uint32_t level,
             const std::string& node_name,
             const AABB& node_bounds)
{
  for (auto next = begin + 1; next < end; ++next) {
    const auto next_morton_idx = next->morton_index.truncate_to_level(level);
    const auto cur_morton_idx = (next - 1)->morton_index.truncate_to_level(level);
    if (cur_morton_idx.get() != next_morton_idx.get()) {
      std::cerr << "morton_check failed @ node " << node_name << "(" << node_bounds << ")!\n"
                << "\tPoint A: " << (next - 1)->point_reference.position() << " with Morton index "
                << to_string(cur_morton_idx) << "\n"
                << "\tPoint B: " << next->point_reference.position() << " with Morton index "
                << to_string(next_morton_idx) << "\n";
      std::exit(EXIT_FAILURE);
    }
  }
}

template<typename Iter>
static void
inside_check(Iter begin, Iter end, const std::string& node_name, const AABB& bounds)
{
  std::for_each(begin, end, [&](const auto& indexed_point) {
    const auto& pos = indexed_point.point_reference.position();
    if (!bounds.isInside(pos)) {
      std::cerr << "validate_points failed @ node " << node_name << "(" << bounds << ")!\n"
                << "\tPoint " << pos << " is outside of bounding box!\n";
      std::exit(EXIT_FAILURE);
    }
  });
}
#endif

using PointsPerNode = std::unordered_map<DynamicMortonIndex, size_t>;
using NodeWithPointCount = std::pair<DynamicMortonIndex, size_t>;

[[maybe_unused]] static PointsPerNode
find_start_nodes_of_batch(PointBuffer::PointIterator points_begin,
                          PointBuffer::PointIterator points_end,
                          std::vector<IndexedPoint64>::iterator indices_begin,
                          const AABB& root_bounds,
                          size_t desired_concurrency,
                          uint32_t max_tree_scan_depth)
{
  index_points<MAX_OCTREE_LEVELS>(
    points_begin, points_end, indices_begin, root_bounds, OutlierPointsBehaviour::ClampToBounds);

  // Count # of points for each morton index for the first 2/3 levels or so
  const auto num_points = std::distance(points_begin, points_end);
  PointsPerNode histogram;
  std::for_each(indices_begin,
                indices_begin + num_points,
                [&histogram, max_tree_scan_depth](const IndexedPoint64& indexed_point) {
                  for (uint32_t level = 0; level < max_tree_scan_depth; ++level) {
                    const DynamicMortonIndex dynamic_index = { indexed_point.morton_index, level };
                    ++histogram[dynamic_index];
                  }
                });

  return histogram;
}

[[maybe_unused]] static std::vector<NodeWithPointCount>
select_start_nodes(const PointsPerNode& nodes_tree, size_t desired_nodes_count)
{
  // Let 'nodes_tree' be an octree where each node contains the number of points
  // that belong to this node. This function then selects as close to
  // 'desired_nodes_count' nodes as possible from this tree, starting from the
  // root node. Nodes are recursively split as long as the total node count is
  // less than 'desired_nodes_count'. For splitting, the node with the largest
  // number of points is used.
  PointsPerNode selected_nodes;

  const auto node_has_children = [](const NodeWithPointCount& node, const PointsPerNode& nodes) {
    for (uint8_t octant = 0; octant < 8; ++octant) {
      if (nodes.find(node.first.child(octant)) != std::end(nodes))
        return true;
    }
    return false;
  };

  const auto find_max_node =
    [node_has_children](const PointsPerNode& nodes) -> std::optional<NodeWithPointCount> {
    std::optional<NodeWithPointCount> max_node;

    for (auto& node_with_count : nodes) {
      if (!node_has_children(node_with_count, nodes))
        continue;

      if (!max_node || max_node->second < node_with_count.second) {
        max_node = node_with_count;
      }
    }

    return max_node;
  };

  const auto split_node_with_largest_count =
    [&selected_nodes, &nodes_tree, find_max_node]() -> bool {
    // Find node with largest count THAT HAS CHILDREN! If no node has children,
    // we are done
    const auto max_node = find_max_node(selected_nodes);
    if (!max_node)
      return false;

    selected_nodes.erase(selected_nodes.find(max_node->first));

    for (uint8_t octant = 0; octant < 8; ++octant) {
      const auto child_node = max_node->first.child(octant);

      const auto iter = nodes_tree.find(child_node);
      if (iter == std::end(nodes_tree) || iter->second == 0)
        continue;

      selected_nodes.insert(*iter);
    }

    return true;
  };

  const DynamicMortonIndex root_index;
  selected_nodes[root_index] = nodes_tree.at(root_index);

  while (selected_nodes.size() < desired_nodes_count) {
    if (!split_node_with_largest_count())
      break;
  }

  return { selected_nodes.begin(), selected_nodes.end() };
}

using IndexedPointIter = std::vector<IndexedPoint64>::iterator;
using IndexedPointRange = util::Range<IndexedPointIter>;

[[maybe_unused]] static std::vector<IndexedPointRange>
filter_and_sort_indexed_points(std::vector<IndexedPoint64>::iterator indexed_points_begin,
                               std::vector<IndexedPoint64>::iterator indexed_points_end,
                               const std::vector<std::pair<DynamicMortonIndex, size_t>>& nodes)
{
  // Sort the indexed points...
  std::sort(indexed_points_begin, indexed_points_end);

  //...then split the whole range up into N ranges where each range encompasses
  // all points that
  // belong to a single node in 'nodes'
  std::vector<IndexedPointRange> indexed_ranges;
  indexed_ranges.reserve(nodes.size());

  const auto find_start_index = [indexed_points_begin,
                                 indexed_points_end](const DynamicMortonIndex& morton_index) {
    return std::lower_bound(
      indexed_points_begin,
      indexed_points_end,
      morton_index,
      [](const IndexedPoint64& indexed_point, const DynamicMortonIndex& ref) {
        return indexed_point.morton_index.truncate_to_level(ref.depth()).get() <
               ref.to_static_morton_index<MAX_OCTREE_LEVELS>().get();
      });
  };

  const auto find_end_index = [indexed_points_begin,
                               indexed_points_end](const DynamicMortonIndex& morton_index) {
    return std::upper_bound(
      indexed_points_begin,
      indexed_points_end,
      morton_index,
      [](const DynamicMortonIndex& ref, const IndexedPoint64& indexed_point) {
        return ref.to_static_morton_index<MAX_OCTREE_LEVELS>().get() <
               indexed_point.morton_index.truncate_to_level(ref.depth()).get();
      });
  };

  for (auto& node_with_count : nodes) {
    const auto start_index = find_start_index(node_with_count.first);
    const auto end_index = find_end_index(node_with_count.first);
    indexed_ranges.push_back({ start_index, end_index });
  }

  const auto selected_points =
    std::accumulate(std::begin(indexed_ranges),
                    std::end(indexed_ranges),
                    size_t{ 0 },
                    [](size_t accum, const auto& range) -> size_t { return accum + range.size(); });
  if (selected_points !=
      static_cast<size_t>(std::distance(indexed_points_begin, indexed_points_end))) {
    throw std::runtime_error{ "Node partitioning does not span the full range of points!" };
  }

  return indexed_ranges;
}

template<typename Func, typename Taskflow>
static void
merge_and_process_points(const std::vector<IndexedPointRange>& sorted_ranges_for_node,
                         const DynamicMortonIndex& morton_index_of_node,
                         const octree::NodeStructure& root_node,
                         const TilerMetaParameters& meta_parameters,
                         Func process_call,
                         Taskflow& taskflow)
{
  const auto num_points = util::range(sorted_ranges_for_node)
                            .accumulate([](size_t accum, const IndexedPointRange& range) {
                              return accum + range.size();
                            });

  octree::NodeData node_data;
  node_data.reserve(num_points);

  // Merge all ranges into node_data using an N-ary variant of std::merge
  auto point_ranges = sorted_ranges_for_node;
  const auto get_index_of_range_containing_lowest_point = [&point_ranges]() {
    size_t lowest_index = 0;
    MortonIndex64 lowest_morton_index = MortonIndex64::MAX;
    for (size_t idx = 0; idx < point_ranges.size(); ++idx) {
      const auto& cur_range = point_ranges[idx];
      if (cur_range.size() == 0)
        continue;
      const auto& cur_morton_index = cur_range.begin()->morton_index;
      if (cur_morton_index.get() < lowest_morton_index.get()) {
        lowest_morton_index = cur_morton_index;
        lowest_index = idx;
      }
    }
    return lowest_index;
  };

  for (size_t idx = 0; idx < num_points; ++idx) {
    const auto index_of_next_range = get_index_of_range_containing_lowest_point();
    auto& next_range = point_ranges[index_of_next_range];
    node_data.push_back(*next_range.begin++);
  }

  // TODO Continue implementing this algorithm. Should be just creating the
  // NodeStructure and calling process_node
  octree::NodeStructure node_structure;
  node_structure.bounds = get_bounds_from_morton_index(morton_index_of_node, root_node.bounds);
  node_structure.level =
    static_cast<int32_t>(morton_index_of_node.depth()) - 1; // TODO Definitiely fix this hacky
                                                            // 'root-level-is-negative-one' stuff...
  node_structure.max_depth = root_node.max_depth;
  node_structure.max_spacing = root_node.max_spacing / (1 << morton_index_of_node.depth());
  node_structure.morton_index = morton_index_of_node.to_static_morton_index<MAX_OCTREE_LEVELS>();
  node_structure.name = to_string(morton_index_of_node, MortonIndexNamingConvention::Potree);

  const auto task_name =
    (boost::format("%1% [%2%]") % node_structure.name % node_data.size()).str();

  taskflow
    .emplace([_node_data = std::move(node_data), node_structure, root_node, process_call](
               auto& subflow) mutable {
      process_node(std::move(_node_data), node_structure, root_node, process_call, subflow);
    })
    .name(task_name);
}

Tiler::Tiler(const AABB& aabb,
             const TilerMetaParameters& meta_parameters,
             SamplingStrategy sampling_strategy,
             ProgressReporter* progress_reporter,
             PointsPersistence& persistence,
             fs::path output_directory)
  : _aabb(aabb)
  , _meta_parameters(meta_parameters)
  , _sampling_strategy(std::move(sampling_strategy))
  , _progress_reporter(progress_reporter)
  , _persistence(persistence)
  , _output_directory(std::move(output_directory))
  , _producers(0)
  , _consumers(1)
  , _run_indexing_thread(true)
{
  const auto root_spacing_to_bounds_ratio =
    std::log2f(aabb.extent().x / meta_parameters.spacing_at_root);
  if (root_spacing_to_bounds_ratio >= MAX_OCTREE_LEVELS) {
    throw std::runtime_error{ "spacing at root node is too small compared to bounds of data!" };
  }

  _indexing_thread = std::thread([this]() { run_worker(); });

  _tiling_algorithm =
    std::make_unique<TilingAlgorithmV3>(_sampling_strategy,
                                        _progress_reporter,
                                        _persistence,
                                        _meta_parameters,
                                        _output_directory,
                                        std::max(1u, std::thread::hardware_concurrency() - 1));
}

Tiler::~Tiler()
{
  if (!_run_indexing_thread)
    return;
  close();
}

void
Tiler::index()
{
  if (!_store.count())
    return;

  _consumers.wait();
  _indexing_point_cache = std::move(_store);
  _store = {};
  _producers.notify();
}

void
Tiler::wait_until_indexed()
{
  _consumers.wait();
}

bool
Tiler::needs_indexing() const
{
  return _store.count() >= _meta_parameters.internal_cache_size;
}

void
Tiler::cache(const PointBuffer& points)
{
  _store.append_buffer(points);
  if (_progress_reporter)
    _progress_reporter->increment_progress<size_t>(progress::LOADING, points.count());
}

void
Tiler::close()
{
  _consumers.wait();
  _run_indexing_thread = false;
  _producers.notify();

  _indexing_thread.join();

  _tiling_algorithm->finalize(_aabb);
}

void
Tiler::run_worker()
{
  // Use max_threads - 1 because one thread is reserved for reading points
  const auto concurrency = std::max(1u, std::thread::hardware_concurrency() - 1);
  // const auto concurrency = 1u;

  tf::Executor executor{ concurrency };
  ExecutorObserver* executor_observer = nullptr;
  if (debug::Journal::instance().is_enabled()) {
    executor_observer = executor.make_observer<ExecutorObserver>();
  }

  while (_run_indexing_thread) {
    _producers.wait();
    if (!_run_indexing_thread)
      break;

    tf::Taskflow taskflow;

    _tiling_algorithm->build_execution_graph(_indexing_point_cache, _aabb, taskflow);

    executor.run(taskflow).wait();

    _consumers.notify();
  }

  if (debug::Journal::instance().is_enabled()) {
    std::ofstream ofs{ concat(_output_directory.string(), "/taskflow.json") };
    executor_observer->dump(ofs);
  }
}