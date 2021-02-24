#include "TilerProcess.h"

#include <experimental/filesystem>

#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "Tiler.h"
#include "io/BinaryPersistence.h"
#include "io/Cesium3DTilesPersistence.h"
#include "io/EntwinePersistence.h"
#include "io/LASFile.h"
#include "io/LASPersistence.h"
#include "point_source/PointSource.h"
#include "util/Config.h"
#include "util/Stats.h"
#include "util/Transformation.h"
#include "util/stuff.h"

#include <debug/Journal.h>
#include <debug/ProgressReporter.h>
#include <debug/ThroughputCounter.h>
#include <terminal/stdout_helper.h>

#include <boost/format.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include "io/TileSetWriter.h"

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
      // A bit hacky, but we have to make sure that we don't delete the journal
      // output folder
      if (global_config().is_journaling_enabled &&
          entry == global_config().journal_directory) {
        // Remove only the contents of the journal output folder, so that we can
        // run the tool multiple times with the same output folder and don't get
        // leave garbage
        for (auto& journal_entry : fs::directory_iterator{ entry }) {
          fs::remove_all(journal_entry);
        }
        continue;
      }

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
    perf_stats.AddMember(
      "prepare_duration", perf.prepare_duration.count(), alloc);
    perf_stats.AddMember(
      "indexing_duration", perf.indexing_duration.count(), alloc);
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

/**
 * Check if the given file exists. Depending on the IgnoreErrors flag, the file
 * is either ignored and the user is notified, or an exception is raised
 */
static bool
check_if_file_exists(const fs::path& file, util::IgnoreErrors errors_to_ignore)
{
  if (fs::exists(file))
    return true;

  if (errors_to_ignore & util::IgnoreErrors::MissingFiles) {
    std::cout << "Ignoring file " << file.string()
              << " because it does not exist!\n";
    // util::write_log(
    //     (boost::format("Ignoring file %1% because it does not exist!") %
    //      file.filename())
    //         .str());
    return false;
  }

  const auto reason =
    (boost::format("Input file %1% does not exist!") % file.string()).str();
  throw std::runtime_error{ reason };
}

static bool
check_if_file_format_is_supported(const fs::path& file,
                                  util::IgnoreErrors errors_to_ignore)
{
  if (file_format_is_supported(file.extension()))
    return true;

  if (errors_to_ignore & util::IgnoreErrors::UnsupportedFileFormat) {
    std::cout << "Ignoring file " << file.string()
              << " because its file format (" << file.extension().string()
              << ") is not supported!\n";
    return false;
  }

  const auto reason =
    (boost::format("File format %1% of input file %2% is not supported!") %
     file.extension().string() % file.string())
      .str();
  throw std::runtime_error{ reason };
}

TilerProcess::TilerProcess(Arguments const& args)
  : _args(args)
  , _ui(&_ui_state)
{}

void
TilerProcess::prepare()
{
  // if sources contains directories, use files inside the directory instead
  std::vector<fs::path> source_files;
  for (const auto& source : _args.sources) {
    if (!check_if_file_exists(source, _args.errors_to_ignore))
      continue;

    if (fs::is_directory(source)) {
      fs::recursive_directory_iterator directory_iter{ source };
      for (; directory_iter != fs::recursive_directory_iterator{};
           directory_iter++) {
        const auto& dir_entry = directory_iter->path();
        if (!fs::is_regular_file(dir_entry))
          continue;

        source_files.push_back(dir_entry);
      }
    } else if (fs::is_regular_file(source)) {
      source_files.push_back(source);
    }
  }

  std::vector<fs::path> filtered_source_files;
  std::copy_if(std::begin(source_files),
               std::end(source_files),
               std::back_inserter(filtered_source_files),
               [this](const fs::path& file) {
                 return check_if_file_exists(file, _args.errors_to_ignore) &&
                        check_if_file_format_is_supported(
                          file, _args.errors_to_ignore);
               });

  if (filtered_source_files.empty()) {
    throw std::runtime_error{ "No files found for processing" };
  }

  _args.sources = std::move(filtered_source_files);

  determine_input_and_output_attributes();

  const auto attributesDescription = print_attributes(_output_attributes);
  util::write_log(concat(
    "Writing the following point attributes: ", attributesDescription, "\n"));

  prepare_output_directory(_args.output_directory);
}

void
TilerProcess::cleanUp()
{
  const auto temp_path = _args.output_directory.append("/temp");
  if (fs::exists(temp_path)) {
    fs::remove(temp_path);
  }
}

void
TilerProcess::determine_input_and_output_attributes()
{
  auto input_attributes = point_attributes_all();

  for (const auto& source : _args.sources) {
    open_point_file(source)
      .map([&input_attributes](const PointFile& point_file) {
        // Remove all attributes from 'attributes' that are NOT in the current
        // point file
        for (auto it = std::begin(input_attributes);
             it != std::end(input_attributes);
             ++it) {
          if (pc::has_attribute(point_file, *it))
            continue;
          it = input_attributes.erase(it);
        }
      })
      .or_else([this, source](const auto& err) {
        if (_args.errors_to_ignore & util::IgnoreErrors::InaccessibleFiles) {
          util::write_log(
            (boost::format("warning: Ignoring file %1% while determining point "
                           "attributes\ncaused by: %2%\n") %
             source.string() % err.what())
              .str());
          return;
        }

        throw util::chain_error(err, "Determining the point attributes failed");
      });
  }

  _input_attributes = std::move(input_attributes);

  // Output attributes are dependent on the attributes that the desired output
  // format supports, and on whether or not one of the input attributes should
  // be converted to RGB
  auto output_attributes = _input_attributes;
  // TODO 3D Tiles is the only format supporting RGB remapping at the moment
  if (_args.output_format == OutputFormat::CZM_3DTILES) {
    switch (_args.rgb_mapping) {
      case RGBMapping::FromIntensityLinear:
      case RGBMapping::FromIntensityLogarithmic:
        output_attributes.insert(PointAttribute::RGB);
        break;
      default:
        break;
    }
  }

  const auto supported_output_attributes =
    supported_output_attributes_for_format(_args.output_format);
  PointAttributes supported_attributes, unsupported_attributes;
  std::for_each(std::begin(output_attributes),
                std::end(output_attributes),
                [&supported_output_attributes,
                 &supported_attributes,
                 &unsupported_attributes](PointAttribute attribute) {
                  const auto is_supported =
                    supported_output_attributes.find(attribute) !=
                    std::end(supported_output_attributes);
                  if (is_supported) {
                    supported_attributes.insert(attribute);
                  } else {
                    unsupported_attributes.insert(attribute);
                  }
                });

  if (!unsupported_attributes.empty()) {
    const auto format_name = util::to_string(_args.output_format);
    util::write_log(
      (boost::format("warning: Not all point attributes in the input files are "
                     "supported when using output "
                     "format %1%. Input files have attributes %2%, %3% only "
                     "supports attributes %4%, so "
                     "attributes %5% will be ignored!\n") %
       format_name % print_attributes(_input_attributes) % format_name %
       print_attributes(supported_output_attributes) %
       print_attributes(unsupported_attributes))
        .str());

    // Remove unsupported attributes from _input_attributes
    for (auto unsupported_attribute : unsupported_attributes) {
      _input_attributes.erase(unsupported_attribute);
    }
  }

  _output_attributes = std::move(supported_attributes);
}

DatasetMetadata
TilerProcess::calculate_dataset_metadata(
  const SRSTransformHelper* srs_transform)
{
  DatasetMetadata dataset_metadata;

  for (const auto& source : _args.sources) {
    open_point_file(source)
      .map([&dataset_metadata, srs_transform, &source](
             const PointFile& point_file) {
        auto bounds = pc::get_bounds(point_file);
        auto point_count = pc::get_point_count(point_file);

        if (srs_transform) {
          srs_transform->transformAABBsTo(TargetSRS::CesiumWorld,
                                          gsl::make_span(&bounds, 1));
        }

        dataset_metadata.add_file_metadata(source, point_count, bounds);
      })
      .or_else([this, source](const auto& err) {
        if (_args.errors_to_ignore & util::IgnoreErrors::InaccessibleFiles) {
          util::write_log(
            (boost::format("warning: Ignoring file %1% while calculating "
                           "dataset metadata\ncaused by: %2%\n") %
             source.string() % err.what())
              .str());
          return;
        }

        throw util::chain_error(err, "Calculating dataset metadata failed");
      });
  }

  return dataset_metadata;
}

std::variant<FixedThreadCount, AdaptiveThreadCount>
TilerProcess::calculate_actual_thread_counts(
  const DatasetMetadata& dataset_metadata) const
{
  if (std::holds_alternative<AdaptiveThreadCount>(_args.thread_config)) {
    return _args.thread_config;
  }

  const auto& fixed_thread_config_before_adjustment =
    std::get<FixedThreadCount>(_args.thread_config);

  FixedThreadCount fixed_thread_count;
  fixed_thread_count.num_threads_for_reading =
    fixed_thread_config_before_adjustment.num_threads_for_reading;
  fixed_thread_count.num_threads_for_indexing =
    fixed_thread_config_before_adjustment.num_threads_for_indexing;

  // We can never have more reading threads than we have files! If we have less
  // files than the requested number of reading threads, we move the excess
  // reading threads over to indexing
  const auto num_files =
    gsl::narrow<uint32_t>(dataset_metadata.get_all_files_metadata().size());
  if (num_files <
      fixed_thread_config_before_adjustment.num_threads_for_reading) {
    const auto diff =
      fixed_thread_config_before_adjustment.num_threads_for_reading - num_files;
    fixed_thread_count.num_threads_for_reading = num_files;
    fixed_thread_count.num_threads_for_indexing += diff;

    std::cout << "Requested "
              << fixed_thread_config_before_adjustment.num_threads_for_reading
              << " threads for reading points but there are only " << num_files
              << " files to read from. Using "
              << fixed_thread_count.num_threads_for_reading
              << " threads for reading and "
              << fixed_thread_count.num_threads_for_indexing
              << " threads for indexing instead!\n";
  } else {
    std::cout << "Using " << fixed_thread_count.num_threads_for_reading
              << " threads for reading\n";
    std::cout << "Using " << fixed_thread_count.num_threads_for_indexing
              << " threads for indexing\n";
  }

  return { fixed_thread_count };
}

void
TilerProcess::check_for_missing_point_attributes(
  const PointAttributes& required_attributes) const
{
  // TODO Rework this method once the attribute rework is done and we support
  // custom schemas

  for (auto& source : _args.sources) {
    open_point_file(source)
      .map([this, &source, &required_attributes](const PointFile& point_file) {
        PointAttributes missing_attributes;
        if (pc::has_all_attributes(
              point_file,
              required_attributes,
              std::inserter(missing_attributes,
                            std::end(missing_attributes)))) {
          return;
        }

        const std::string attribute_label =
          (missing_attributes.size() > 1) ? "attributes" : "attribute";

        if (_args.errors_to_ignore &
            util::IgnoreErrors::MissingPointAttributes) {
          util::write_log(
            (boost::format("warning: Missing %1% %2% in file %3%\n") %
             attribute_label % print_attributes(missing_attributes) %
             source.string())
              .str());
          return;
        }

        throw std::runtime_error{
          (boost::format("Missing %1% %2% in file %3%") % attribute_label %
           print_attributes(missing_attributes) % source.string())
            .str()
        };
      })
      .or_else([this, source](const auto& err) {
        if (_args.errors_to_ignore & util::IgnoreErrors::InaccessibleFiles) {
          util::write_log(
            (boost::format(
               "warning: Ignoring file %1% while checking for missing "
               "attributes in source files\n\tcaused by: %2%\n") %
             source.string() % err.what())
              .str());
          return;
        }

        throw util::chain_error(
          err, "Checking for missing point attributes in source files failed");
      });
  }
}

SamplingStrategy
TilerProcess::make_sampling_strategy() const
{
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
  if (_args.sampling_strategy == "JITTERED")
    return JitteredSampling{ _args.max_points_per_node };

  throw std::invalid_argument{ (boost::format(
                                  "Unrecognized sampling strategy %1%") %
                                _args.sampling_strategy)
                                 .str() };
}

Tiler
TilerProcess::make_tiler(
  bool shift_points_to_center,
  uint32_t max_depth,
  std::variant<FixedThreadCount, AdaptiveThreadCount> thread_count,
  SRSTransformHelper const* srs_transform,
  DatasetMetadata dataset_metadata,
  SamplingStrategy sampling_strategy,
  ProgressReporter* progress_reporter,
  PointsPersistence& persistence) const
{
  TilerMetaParameters tiler_meta_parameters;
  tiler_meta_parameters.spacing_at_root = _args.spacing;
  tiler_meta_parameters.max_depth = max_depth;
  tiler_meta_parameters.max_points_per_node = _args.max_points_per_node;
  tiler_meta_parameters.internal_cache_size = _args.internal_cache_size;
  tiler_meta_parameters.tiling_strategy = _args.tiling_strategy;
  tiler_meta_parameters.batch_read_size = _args.max_batch_read_size;
  tiler_meta_parameters.shift_points_to_origin = shift_points_to_center;
  tiler_meta_parameters.thread_count = thread_count;

  MultiReaderPointSource point_source{ _args.sources, _args.errors_to_ignore };
  point_source.add_transformation(
    [this,
     srs_transform,
     cubic_bounds = dataset_metadata.total_bounds_cubic(),
     shift_points_to_center](util::Range<PointBuffer::PointIterator> points) {
      srs_transform->transformPointsTo(TargetSRS::CesiumWorld, points);

      // 3D Tiles is not strictly lossless as it stores 32-bit floating point
      // values instead of 64-bit. We shift all points to the center of the
      // bounding box of the full point-cloud and truncate the values to
      // 32-bit to get the maximum precision while at the same time
      // guaranteeing lossless persistence
      if (shift_points_to_center) {
        for (auto point_ref : points) {
          auto& position = point_ref.position();
          position -= cubic_bounds.getCenter();
          position.x = static_cast<float>(position.x);
          position.y = static_cast<float>(position.y);
          position.z = static_cast<float>(position.z);
        }
      }
    });

  return { std::move(dataset_metadata), tiler_meta_parameters,
           sampling_strategy,           progress_reporter,
           std::move(point_source),     persistence,
           _input_attributes,           _args.output_directory };
}

void
TilerProcess::run()
{
  const auto prepare_start = std::chrono::high_resolution_clock::now();

  prepare();

  std::unique_ptr<SRSTransformHelper> srs_transform;
  if (_args.source_projection) {
    srs_transform = std::make_unique<Proj4Transform>(*_args.source_projection);
  } else {
    srs_transform = std::make_unique<IdentityTransform>();
  }

  auto dataset_metadata = calculate_dataset_metadata(srs_transform.get());

  const auto total_points_count = dataset_metadata.total_points_count();
  const auto cubic_bounds = dataset_metadata.total_bounds_cubic();
  if (!total_points_count) {
    throw std::runtime_error{ "Found no points to process" };
  }

  util::write_log(concat("Total points: ", total_points_count, "\n"));

  util::write_log(
    concat("Bounds:\n", dataset_metadata.total_bounds_tight(), "\n"));
  util::write_log(
    concat("Bounds (cubic):\n", dataset_metadata.total_bounds_cubic(), "\n"));

  if (_args.diagonal_fraction != 0) {
    _args.spacing =
      (float)(dataset_metadata.total_bounds_cubic().extent().length() /
              _args.diagonal_fraction);
    util::write_log(
      concat("Spacing calculated from diagonal: ", _args.spacing, "\n"));
  }

  auto thread_counts = calculate_actual_thread_counts(dataset_metadata);

  auto& progress_reporter = _ui_state.get_progress_reporter();
  progress_reporter.register_progress_counter<size_t>(progress::LOADING,
                                                      total_points_count);
  progress_reporter.register_progress_counter<size_t>(progress::INDEXING,
                                                      total_points_count);

  auto persistence = make_persistence(_args.output_format,
                                      _args.output_directory,
                                      _input_attributes,
                                      _output_attributes,
                                      _args.rgb_mapping,
                                      _args.spacing,
                                      dataset_metadata.total_bounds_cubic());
  const auto shift_points_to_center =
    (_args.output_format == OutputFormat::CZM_3DTILES);

  const auto max_depth =
    (_args.max_depth <= 0)
      ? (100u)
      : static_cast<uint32_t>(
          _args.max_depth); // TODO max_depth parameter with uint32_t max
                            // results in only root level being created...

  util::write_log(concat("Using ", _args.sampling_strategy, " sampling\n"));
  auto sampling_strategy = make_sampling_strategy();

  auto tiler = make_tiler(shift_points_to_center,
                          max_depth,
                          thread_counts,
                          srs_transform.get(),
                          std::move(dataset_metadata),
                          std::move(sampling_strategy),
                          &progress_reporter,
                          persistence);

  TerminalUIAsyncRenderer ui_renderer{ _ui };

  const auto prepare_end = std::chrono::high_resolution_clock::now();
  const auto prepare_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(prepare_end -
                                                          prepare_start);

  const auto indexing_start = std::chrono::high_resolution_clock::now();

  const auto num_processed_points = tiler.run();

  const auto indexing_end = std::chrono::high_resolution_clock::now();
  const auto indexing_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(indexing_end -
                                                          indexing_start);

  PerformanceStats stats;
  stats.prepare_duration = prepare_duration;
  stats.indexing_duration = indexing_duration;
  stats.points_processed = total_points_count;

  write_properties_json(
    _args.output_directory, cubic_bounds, _args.spacing, stats);

  if (_args.output_format == OutputFormat::ENTWINE_LAS ||
      _args.output_format == OutputFormat::ENTWINE_LAZ) {
    EptJson ept_json;
    ept_json.bounds = cubic_bounds;
    ept_json.conforming_bounds = cubic_bounds;
    ept_json.data_type = (_args.output_format == OutputFormat::ENTWINE_LAZ)
                           ? EntwineFormat::LAZ
                           : EntwineFormat::LAS;
    ept_json.hierarchy_type = "json";
    ept_json.points = num_processed_points;
    ept_json.schema = point_attributes_to_ept_schema(_output_attributes);
    ept_json.span = _args.spacing;
    // ept_json.srs = ...;
    ept_json.version = "1.0.0";
    write_ept_json(_args.output_directory / "ept.json", ept_json);
  }

  const auto total_indexed_count =
    progress_reporter.get_progress<size_t>(progress::INDEXING);
  const auto dropped_points_count = total_points_count - total_indexed_count;

  if (dropped_points_count) {
    util::write_log(
      (boost::format("Tiler finished with warnings - Indexed %1% out of %2% "
                     "points (%3% points could not be indexed)") %
       total_indexed_count % total_points_count % dropped_points_count)
        .str());
  } else {
    util::write_log((boost::format("Tiler finished - Indexed %1% points") %
                     total_indexed_count)
                      .str());
  }
}
