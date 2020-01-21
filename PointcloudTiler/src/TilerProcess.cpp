#include "TilerProcess.h"

#include <experimental/filesystem>

#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "LASPointReader.h"
#include "ThroughputCounter.hpp"
#include "Tiler.h"
#include "Transformation.h"
#include "io/BinaryPersistence.h"
#include "io/Cesium3DTilesPersistence.h"
#include "io/LASFile.h"
#include "io/LASPersistence.h"
#include "io/stdout_helper.h"
#include "stuff.h"
#include "ui/ProgressReporter.h"
#include "util/Debugging.h"
#include "util/Stats.h"

#include <boost/format.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include "TileSetWriter.h"

namespace rj = rapidjson;

constexpr auto PROCESS_COUNT = 1'000'000;

/// <summary>
/// Verify that output directory is valid
/// </summary>
static void
prepare_output_directory(const std::string& output_directory)
{
  if (fs::exists(output_directory)) {
    // TODO We could add a progress bar here!
    util::write_log("Output directory not empty, removing existing files\n");
    for (auto& entry : fs::directory_iterator{ output_directory }) {
      fs::remove_all(entry);
    }
  } else {
    util::write_log("Output directory does not exist, creating it\n");
    fs::create_directories(output_directory);
  }
}

static void
write_properties_json(const std::string& output_directory,
                      const AABB& bounds,
                      float root_spacing,
                      const PerformanceStats& perf)
{
  rj::Document document;
  document.SetObject();

  auto& alloc = document.GetAllocator();

  rj::Value source_props(rj::kObjectType);
  rj::Value perf_stats(rj::kObjectType);

  // Bounds
  {
    rj::Value bounds_min(rj::kArrayType);
    rj::Value bounds_max(rj::kArrayType);

    bounds_min.PushBack(bounds.min.x, alloc);
    bounds_min.PushBack(bounds.min.y, alloc);
    bounds_min.PushBack(bounds.min.z, alloc);

    bounds_max.PushBack(bounds.max.x, alloc);
    bounds_max.PushBack(bounds.max.y, alloc);
    bounds_max.PushBack(bounds.max.z, alloc);

    rj::Value bounds(rj::kObjectType);
    bounds.AddMember("min", bounds_min, alloc);
    bounds.AddMember("max", bounds_max, alloc);

    source_props.AddMember("bounds", bounds, alloc);
  }

  // Root spacing
  {
    source_props.AddMember("root_spacing", root_spacing, alloc);
  }

  // Point stats
  {
    source_props.AddMember("processed_points", perf.points_processed, alloc);
  }

  // Performance stats
  {
    perf_stats.AddMember("prepare_duration", perf.prepare_duration.count(), alloc);
    perf_stats.AddMember("indexing_duration", perf.indexing_duration.count(), alloc);
  }

  document.AddMember("source_properties", source_props, alloc);
  document.AddMember("performance_stats", perf_stats, alloc);

  struct Stream
  {
    std::ofstream of;

    explicit Stream(const std::string& filepath)
      : of{ filepath, std::ios::binary }
    {}

    typedef char Ch;
    void Put(Ch ch) { of.put(ch); }
    void Flush() {}
  };

  Stream fs{ output_directory + "/properties.json" };
  if (!fs.of.is_open()) {
    std::cerr << "Error writing properties.json file!" << std::endl;
    return;
  }

  rj::Writer<Stream> writer(fs);
  document.Accept(writer);
}

using PointFile = std::variant<LASFile>; // and more files

PointFile
open_point_file(const std::experimental::filesystem::path& path)
{
  if (path.extension() == ".las" || path.extension() == ".laz") {
    return { LASFile{ path, LASFile::OpenMode::Read } };
  }
  throw std::runtime_error{
    (boost::format("Unsupported file extension '%1%'") % path.extension().string()).str()
  };
}

TilerProcess::TilerProcess(Arguments const& args)
  : _args(args)
  , _ui(&_ui_state)
{}

void
TilerProcess::prepare()
{
  prepare_output_directory(_args.output_directory);

  // if sources contains directories, use files inside the directory instead
  std::vector<fs::path> filtered_source_files;
  for (const auto& source : _args.sources) {
    fs::path pSource(source);
    if (fs::is_directory(pSource)) {
      fs::recursive_directory_iterator it(pSource);
      for (; it != fs::recursive_directory_iterator(); it++) {
        fs::path pDirectoryEntry = it->path();
        if (fs::is_regular_file(pDirectoryEntry)) {
          std::string filepath = pDirectoryEntry.string();
          if (iEndsWith(filepath, ".las") || iEndsWith(filepath, ".laz") ||
              iEndsWith(filepath, ".xyz") || iEndsWith(filepath, ".pts") ||
              iEndsWith(filepath, ".ptx") || iEndsWith(filepath, ".ply")) {
            filtered_source_files.push_back(filepath);
          }
        }
      }
    } else if (fs::is_regular_file(pSource)) {
      filtered_source_files.push_back(source);
    } else {
      util::write_log(concat("Can't open input file \"", source, "\n"));
    }
  }

  // Remove input files that don't exist and notify user
  filtered_source_files.erase(std::remove_if(filtered_source_files.begin(),
                                             filtered_source_files.end(),
                                             [](const auto& path) {
                                               if (fs::exists(path))
                                                 return false;
                                               util::write_log(
                                                 concat("Can't open input file \"", path, "\n"));
                                               return true;
                                             }),
                              filtered_source_files.end());

  _args.sources = std::move(filtered_source_files);

  _point_attributes = point_attributes_from_strings(_args.output_attributes);

  const auto attributesDescription = print_attributes(_point_attributes);
  util::write_log(concat("Writing the following point attributes: ", attributesDescription, "\n"));
}

void
TilerProcess::cleanUp()
{
  const auto temp_path = _args.output_directory.append("/temp");
  if (fs::exists(temp_path)) {
    fs::remove(temp_path);
  }
}

AABB
TilerProcess::calculateAABB()
{
  AABB aabb;
  for (const auto& source : _args.sources) {
    auto point_file = open_point_file(source);

    const auto bounds = pc::get_bounds(point_file);
    aabb.update(bounds.min);
    aabb.update(bounds.max);
  }

  return aabb;
}

size_t
TilerProcess::get_total_points_count() const
{
  size_t total_count = 0;
  for (auto& source : _args.sources) {
    auto point_file = open_point_file(source);
    total_count += pc::get_point_count(point_file);
  }
  return total_count;
}

void
TilerProcess::run()
{
  const auto prepare_start = std::chrono::high_resolution_clock::now();

  prepare();

  const auto total_points_count = get_total_points_count();

  util::write_log(concat("Total points: ", total_points_count, "\n"));

  // We don't transform the AABBs here, since this would break the process of
  // partitioning the points. Instead, we will transform only upon writing the
  // bounding boxes to the JSON files

  AABB aabb = calculateAABB();
  util::write_log(concat("Bounds:\n", aabb, "\n"));
  aabb.makeCubic();
  util::write_log(concat("Bounds (cubic):\n", aabb, "\n"));

  if (_args.diagonal_fraction != 0) {
    _args.spacing = (float)(aabb.extent().length() / _args.diagonal_fraction);
    util::write_log(concat("Spacing calculated from diagonal: ", _args.spacing, "\n"));
  }

  auto& progress_reporter = _ui_state.get_progress_reporter();
  progress_reporter.register_progress_counter<size_t>(progress::LOADING, total_points_count);
  progress_reporter.register_progress_counter<size_t>(progress::INDEXING, total_points_count);

  PointsPersistence persistence{ BinaryPersistence{ _args.output_directory,
                                                    _point_attributes,
                                                    _args.use_compression ? Compressed::Yes
                                                                          : Compressed::No } };

  const auto max_depth =
    (_args.max_depth <= 0)
      ? (100u)
      : static_cast<uint32_t>(_args.max_depth); // TODO max_depth parameter with uint32_t max
                                                // results in only root level being created...

  util::write_log(concat("Using ", _args.sampling_strategy, " sampling\n"));
  auto sampling_strategy = [&]() -> SamplingStrategy {
    if (_args.sampling_strategy == "RANDOM_GRID")
      return RandomSortedGridSampling{ _args.max_points_per_node };
    if (_args.sampling_strategy == "GRID_CENTER")
      return GridCenterSampling{ _args.max_points_per_node };
    if (_args.sampling_strategy == "MIN_DISTANCE")
      return PoissonDiskSampling{ _args.max_points_per_node };
    if (_args.sampling_strategy == "MIN_DISTANCE_FAST")
      return AdaptivePoissonDiskSampling{ _args.max_points_per_node,
                                          [](int32_t node_level) -> float {
                                            if (node_level < 0)
                                              return 0.25f;
                                            if (node_level < 1)
                                              return 0.5f;
                                            return 1.f;
                                          } };
    throw std::invalid_argument{
      (boost::format("Unrecognized sampling strategy %1%") % _args.sampling_strategy).str()
    };
  }();

  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = _args.spacing;
  tiler_meta_parameters.max_depth = max_depth;
  tiler_meta_parameters.max_points_per_node = _args.max_points_per_node;
  tiler_meta_parameters.internal_cache_size = _args.internal_cache_size;

  Tiler tiler{ aabb,        tiler_meta_parameters, sampling_strategy, &progress_reporter,
               persistence, _args.output_directory };

  std::atomic_bool draw_ui = true;
  std::thread ui_thread{ [this, &draw_ui]() {
    while (draw_ui) {
      _ui.redraw();
      std::this_thread::sleep_for(std::chrono::milliseconds{ 50 });
    }
  } };

  const auto prepare_end = std::chrono::high_resolution_clock::now();
  const auto prepare_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(prepare_end - prepare_start);

  const auto indexing_start = std::chrono::high_resolution_clock::now();

  for (const auto& source : _args.sources) {
    try {
      auto point_file = open_point_file(source);

      std::visit(
        [this, &tiler](auto& typed_file) {
          // TODO Would love to chunk the full file range here, but this
          // doesn't work because the iterator is not random access
          auto begin = typed_file.cbegin();
          const auto end = typed_file.cend();
          while (begin != end) {
            PointBuffer point_batch;
            begin = pc::read_points(begin,
                                    _args.max_batch_read_size,
                                    pc::metadata(typed_file),
                                    _point_attributes,
                                    point_batch);
            if (point_batch.empty())
              break;

            tiler.cache(point_batch);
            if (tiler.needs_indexing()) {
              tiler.index();
            }
          }
        },
        point_file);
    } catch (std::exception const& ex) {
      util::write_log(concat("Could not process file ", source.string(), "(", ex.what(), ")\n"));
    }
  }

  tiler.index();
  tiler.close();

  const auto indexing_end = std::chrono::high_resolution_clock::now();
  const auto indexing_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(indexing_end - indexing_start);

  PerformanceStats stats;
  stats.prepare_duration = prepare_duration;
  stats.indexing_duration = indexing_duration;
  stats.points_processed = total_points_count;

  write_properties_json(_args.output_directory, aabb, _args.spacing, stats);

  draw_ui = false;
  ui_thread.join();
}
