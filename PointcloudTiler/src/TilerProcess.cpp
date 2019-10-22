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
#include "definitions.hpp"
#include "io/Cesium3DTilesPersistence.h"
#include "io/LASPersistence.h"
#include "stuff.h"
#include "ui/ProgressReporter.h"
#include "util/Stats.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include "TileSetWriter.h"

namespace fs = std::experimental::filesystem;
namespace rj = rapidjson;

constexpr auto PROCESS_COUNT = 1'000'000;

// static std::unique_ptr<SRSTransformHelper>
// get_transformation_helper(const std::optional<std::string>& sourceProjection)
// {
//   if (!sourceProjection) {
//     std::cout << "Source projection not specified, skipping point "
//                  "transformation..."
//               << std::endl;
//     return std::make_unique<IdentityTransform>();
//   }

//   if (std::strcmp(sourceProjection->c_str(), "+proj=longlat +datum=WGS84
//   +no_defs") == 0) {
//     std::cout << "Source projection is already WGS84, skipping point "
//                  "transformation..."
//               << std::endl;
//     return std::make_unique<IdentityTransform>();
//   }

//   try {
//     return std::make_unique<Proj4Transform>(*sourceProjection);
//   } catch (const std::runtime_error& err) {
//     std::cerr << "Error while setting up coordinate transformation:\n" <<
//     err.what() << std::endl; std::cerr << "Skipping point transformation..."
//     << std::endl;
//   }
//   return std::make_unique<IdentityTransform>();
// }

/// <summary>
/// Verify that output directory is valid
/// </summary>
static void
verifyWorkDir(const std::string& workDir)
{
  if (fs::exists(workDir)) {
    std::cout << "Output directory not empty, removing existing files..." << std::endl;
    for (auto& entry : fs::directory_iterator{ workDir }) {
      fs::remove_all(entry);
    }
  } else {
    std::cout << "Output directory does not exist, creating it..." << std::endl;
    fs::create_directories(workDir);
  }
}

static void
write_properties_json(const std::string& work_dir,
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

  Stream fs{ work_dir + "/properties.json" };
  if (!fs.of.is_open()) {
    std::cerr << "Error writing properties.json file!" << std::endl;
    return;
  }

  rj::Writer<Stream> writer(fs);
  document.Accept(writer);
}

PointReader*
TilerProcess::createPointReader(const std::string& path,
                                const PointAttributes& pointAttributes) const
{
  PointReader* reader = NULL;
  if (iEndsWith(path, ".las") || iEndsWith(path, ".laz")) {
    reader = new LASPointReader(path, pointAttributes);
  }

  return reader;
}

TilerProcess::TilerProcess(const std::string& workDir, std::vector<std::string> sources)
  : _ui(&_ui_state)
{
  this->workDir = workDir;
  this->sources = sources;
}

void
TilerProcess::prepare()
{
  verifyWorkDir(workDir);

  // if sources contains directories, use files inside the directory instead
  std::vector<std::string> sourceFiles;
  for (const auto& source : sources) {
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
            sourceFiles.push_back(filepath);
          }
        }
      }
    } else if (fs::is_regular_file(pSource)) {
      sourceFiles.push_back(source);
    } else {
      std::cout << "Can't open input file \"" << source << "\"" << std::endl;
    }
  }

  // Remove input files that don't exist and notify user
  sourceFiles.erase(std::remove_if(sourceFiles.begin(),
                                   sourceFiles.end(),
                                   [](const auto& path) {
                                     if (fs::exists(path))
                                       return false;
                                     std::cout << "Can't open input file \"" << path << "\""
                                               << std::endl;
                                     return true;
                                   }),
                    sourceFiles.end());

  this->sources = sourceFiles;

  pointAttributes = point_attributes_from_strings(outputAttributes);

  const auto attributesDescription = pointAttributes.toString();
  std::cout << "Writing the following point attributes: " << attributesDescription << std::endl;
}

void
TilerProcess::cleanUp()
{
  const auto tempPath = workDir + "/temp";
  if (fs::exists(tempPath)) {
    fs::remove(tempPath);
  }
}

AABB
TilerProcess::calculateAABB()
{
  AABB aabb;
  for (const auto& source : sources) {
    PointReader* reader = createPointReader(source, pointAttributes);

    AABB sourceAABB = reader->getAABB();
    aabb.update(sourceAABB.min);
    aabb.update(sourceAABB.max);

    reader->close();
    delete reader;
  }

  return aabb;
}

size_t
TilerProcess::get_total_points_count() const
{
  size_t total_count = 0;
  for (auto& source : sources) {
    PointReader* reader = createPointReader(source, pointAttributes);
    total_count += reader->numPoints();
    delete reader;
  }
  return total_count;
}

void
TilerProcess::run()
{
  const auto prepare_start = std::chrono::high_resolution_clock::now();

  prepare();

  const auto total_points_count = get_total_points_count();

  std::cout << "Total points: " << total_points_count << std::endl;

  // We don't transform the AABBs here, since this would break the process of
  // partitioning the points. Instead, we will transform only upon writing the
  // bounding boxes to the JSON files

  AABB aabb = calculateAABB();
  std::cout << "AABB: " << std::endl << aabb << std::endl;
  aabb.makeCubic();
  std::cout << "cubic AABB: " << std::endl << aabb << std::endl;

  if (diagonalFraction != 0) {
    spacing = (float)(aabb.extent().length() / diagonalFraction);
    std::cout << "spacing calculated from diagonal: " << spacing << std::endl;
  }

  auto& progress_reporter = _ui_state.get_progress_reporter();
  progress_reporter.register_progress_counter<size_t>(progress::LOADING, total_points_count);
  progress_reporter.register_progress_counter<size_t>(progress::INDEXING, total_points_count);

  const auto persistence = [&]() -> std::unique_ptr<IPointsPersistence> {
    if (outputFormat == OutputFormat::LAS) {
      return std::make_unique<LASPersistence>(
        workDir, pointAttributes, LASPersistence::Compressed::No);
    } else if (outputFormat == OutputFormat::LAZ) {
      return std::make_unique<LASPersistence>(
        workDir, pointAttributes, LASPersistence::Compressed::Yes);
    }
    throw new std::runtime_error{ "Unrecognized OutputFormat!" };
  }();

  const auto max_depth =
    (maxDepth <= 0)
      ? (100u)
      : static_cast<uint32_t>(maxDepth); // TODO max_depth parameter with uint32_t max
                                         // results in only root level being created...

  std::cout << "Using " << samplingStrategy << " sampling" << std::endl;
  auto sampling_strategy = make_sampling_strategy_from_name(samplingStrategy, max_points_per_node);

  Tiler tiler{
    aabb,        spacing, max_depth, max_points_per_node, sampling_strategy, &progress_reporter,
    *persistence
  };

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

  std::vector<AABB> boundingBoxes;
  std::vector<int> numPoints;
  std::vector<std::string> sourceFilenames;

  for (const auto& source : sources) {
    PointReader* reader = createPointReader(source, pointAttributes);
    // TODO We should issue a warning here if 'pointAttributes' do not match the
    // attributes available in the current file!

    boundingBoxes.push_back(reader->getAABB());
    numPoints.push_back(reader->numPoints());
    sourceFilenames.push_back(fs::path(source).filename().string());

    // NOTE Points from the source file(s) are read here
    while (true) {
      auto pointBatch = reader->readPointBatch();
      if (pointBatch.empty())
        break;

      tiler.cache(pointBatch);

      if (tiler.needs_indexing()) {
        tiler.index();
        tiler.wait_until_indexed();
      }
      if (tiler.needs_flush()) {
        tiler.flush();
      }
    }
    reader->close();
    delete reader;
  }

  tiler.flush();
  tiler.close();

  const auto indexing_end = std::chrono::high_resolution_clock::now();
  const auto indexing_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(indexing_end - indexing_start);

  PerformanceStats stats;
  stats.prepare_duration = prepare_duration;
  stats.indexing_duration = indexing_duration;
  stats.points_processed = total_points_count;

  write_properties_json(workDir, aabb, spacing, stats);

  draw_ui = false;
  ui_thread.join();
}
