

#include <experimental/filesystem>

#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "BINPointReader.hpp"
#include "BatchedPotreeWriter.h"
#include "LASPointReader.h"
#include "PTXPointReader.h"
#include "PlyPointReader.h"
#include "PotreeConverter.h"
#include "PotreeException.h"
#include "PotreeWriter.h"
#include "ThroughputCounter.hpp"
#include "Transformation.h"
#include "XYZPointReader.hpp"
#include "definitions.hpp"
#include "io/Cesium3DTilesPersistence.h"
#include "io/Cesium3DTilesPostprocessing.h"
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

using rapidjson::Document;
using rapidjson::PrettyWriter;
using rapidjson::StringBuffer;
using rapidjson::Value;
using rapidjson::Writer;

using std::find;
using std::fstream;
using std::map;
using std::string;
using std::stringstream;
using std::vector;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace fs = std::experimental::filesystem;

namespace Potree {

constexpr auto PROCESS_COUNT = 1'000'000;

static std::unique_ptr<SRSTransformHelper>
get_transformation_helper(const std::optional<std::string>& sourceProjection)
{
  if (!sourceProjection) {
    std::cout << "Source projection not specified, skipping point "
                 "transformation..."
              << std::endl;
    return std::make_unique<IdentityTransform>();
  }

  if (std::strcmp(sourceProjection->c_str(), "+proj=longlat +datum=WGS84 +no_defs") == 0) {
    std::cout << "Source projection is already WGS84, skipping point "
                 "transformation..."
              << std::endl;
    return std::make_unique<IdentityTransform>();
  }

  try {
    return std::make_unique<Proj4Transform>(*sourceProjection);
  } catch (const std::runtime_error& err) {
    std::cerr << "Error while setting up coordinate transformation:\n" << err.what() << std::endl;
    std::cerr << "Skipping point transformation..." << std::endl;
  }
  return std::make_unique<IdentityTransform>();
}

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

PointReader*
PotreeConverter::createPointReader(const string& path, const PointAttributes& pointAttributes) const
{
  PointReader* reader = NULL;
  if (iEndsWith(path, ".las") || iEndsWith(path, ".laz")) {
    reader = new LASPointReader(path, pointAttributes);
  }

  return reader;
}

PotreeConverter::PotreeConverter(const string& workDir, vector<string> sources)
  : _ui(&_ui_state)
{
  this->workDir = workDir;
  this->sources = sources;
}

void
PotreeConverter::prepare()
{
  verifyWorkDir(workDir);

  // if sources contains directories, use files inside the directory instead
  vector<string> sourceFiles;
  for (const auto& source : sources) {
    fs::path pSource(source);
    if (fs::is_directory(pSource)) {
      fs::directory_iterator it(pSource);
      for (; it != fs::directory_iterator(); it++) {
        fs::path pDirectoryEntry = it->path();
        if (fs::is_regular_file(pDirectoryEntry)) {
          string filepath = pDirectoryEntry.string();
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

  pointAttributes.add(attributes::POSITION_CARTESIAN);
  for (const auto& attribute : outputAttributes) {
    if (attribute == "RGB") {
      pointAttributes.add(attributes::COLOR_PACKED);
    } else if (attribute == "RGB_FROM_INTENSITY") {
      pointAttributes.add(attributes::COLOR_FROM_INTENSITY);
    } else if (attribute == "INTENSITY") {
      pointAttributes.add(attributes::INTENSITY);
    } else if (attribute == "CLASSIFICATION") {
      pointAttributes.add(attributes::CLASSIFICATION);
    } else if (attribute == "NORMAL") {
      pointAttributes.add(attributes::NORMAL_OCT16);
    }
  }

  const auto attributesDescription = pointAttributes.toString();
  std::cout << "Writing the following point attributes: " << attributesDescription << std::endl;
}

void
PotreeConverter::cleanUp()
{
  const auto tempPath = workDir + "/temp";
  if (fs::exists(tempPath)) {
    fs::remove(tempPath);
  }
}

AABB
PotreeConverter::calculateAABB()
{
  AABB aabb;
  for (string source : sources) {
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
PotreeConverter::get_total_points_count() const
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
PotreeConverter::convert()
{
  const auto prepare_start = high_resolution_clock::now();

  prepare();

  const auto total_points_count = get_total_points_count();

  std::cout << "Total points: " << total_points_count << std::endl;

  // We don't transform the AABBs here, since this would break the process of
  // partitioning the points. Instead, we will transform only upon writing the
  // bounding boxes to the JSON files

  AABB aabb = calculateAABB();
  cout << "AABB: " << endl << aabb << endl;
  aabb.makeCubic();
  cout << "cubic AABB: " << endl << aabb << endl;

  auto transformation = get_transformation_helper(sourceProjection);

  if (diagonalFraction != 0) {
    spacing = (float)(aabb.size.length() / diagonalFraction);
    cout << "spacing calculated from diagonal: " << spacing << endl;
  }

  auto& progress_reporter = _ui_state.get_progress_reporter();
  progress_reporter.register_progress_counter<size_t>(progress::LOADING, total_points_count);
  progress_reporter.register_progress_counter<size_t>(progress::INDEXING, total_points_count);

  Cesium3DTilesPersistence persistence{ workDir, pointAttributes, *transformation };

  const auto max_depth =
    (maxDepth <= 0) ? std::numeric_limits<uint32_t>::max() : static_cast<uint32_t>(maxDepth);

  // PotreeWriter writer{ this->workDir,   aabb,
  //                      spacing,         maxDepth,
  //                      scale,           outputFormat,
  //                      pointAttributes, quality,
  //                      *transformation, max_memory_usage_MiB };
  BatchedPotreeWriter writer{ this->workDir,      aabb,       spacing,         max_depth,
                              pointAttributes,    quality,    *transformation, max_memory_usage_MiB,
                              &progress_reporter, persistence };

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

  vector<AABB> boundingBoxes;
  vector<int> numPoints;
  vector<string> sourceFilenames;

  for (const auto& source : sources) {
    // cout << "READING:  " << source << endl;

    PointReader* reader = createPointReader(source, pointAttributes);
    // TODO We should issue a warning here if 'pointAttributes' do not match the
    // attributes available in the current file!

    boundingBoxes.push_back(reader->getAABB());
    numPoints.push_back(reader->numPoints());
    sourceFilenames.push_back(fs::path(source).filename().string());

    // NOTE Points from the source file(s) are read here
    while (true) {
      // if (progress_reporter.get_progress<size_t>(progress::LOADING) >= 10'000'000) {
      //  break;
      //}

      auto pointBatch = reader->readPointBatch();
      if (pointBatch.empty())
        break;

      writer.cache(pointBatch);

      if (writer.needs_indexing()) {
        writer.index();
        writer.wait_until_indexed();
      }
      if (writer.needs_flush()) {
        writer.flush();
      }
    }
    reader->close();
    delete reader;
  }

  writer.flush();
  writer.close();

  const auto indexing_end = high_resolution_clock::now();
  const auto indexing_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(indexing_end - indexing_start);

  const auto postprocessing_start = std::chrono::high_resolution_clock::now();

  do_cesium_3dtiles_postprocessing(
    workDir, aabb, spacing, *transformation, pointAttributes, &progress_reporter);

  const auto postprocessing_end = std::chrono::high_resolution_clock::now();

  const auto postprocessing_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    postprocessing_end - postprocessing_start);

  draw_ui = false;
  ui_thread.join();

  PerformanceStats stats;
  stats.prepare_duration = prepare_duration;
  stats.indexing_duration = indexing_duration;
  stats.postprocessing_duration = postprocessing_duration;
  stats.files_written = progress_reporter.get_progress<size_t>(progress::POSTPROCESSING);
  stats.points_processed = total_points_count;
  dump_perf_stats(stats, workDir);
}

} // namespace Potree
