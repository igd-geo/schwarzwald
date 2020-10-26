
#include <chrono>
#include <exception>
#include <experimental/filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/variables_map.hpp>

#include <algorithms/Strings.h>

#include "debug/Journal.h"
#include "expected.hpp"
#include "io/TileSetWriter.h"
#include "math/AABB.h"
#include "math/Vector3.h"
#include "pointcloud/Tileset.h"
#include "process/ConverterProcess.h"
#include "process/TilerProcess.h"
#include "util/Config.h"

namespace bpo = boost::program_options;

/**
 * Processing mode of the tool
 */
enum class Mode
{
  /**
   * Tiling process, i.e. creating the octree structure
   */
  Tiler,
  /**
   * Conversion process, i.e. converting into a different file format
   */
  Converter
};

static tl::expected<unit::byte, std::string>
parse_memory_size(std::string const& memory_size_string)
{
  std::regex memory_size_regex{ "([0-9])+([a-zA-Z])+" };
  if (!std::regex_match(memory_size_string, memory_size_regex)) {
    return tl::make_unexpected(
      (boost::format("Could not parse memory size %1%") % memory_size_string).str());
  }

  std::smatch quantity_match, suffix_match;
  std::regex_search(memory_size_string, quantity_match, std::regex{ "([0-9])+" });
  std::regex_search(memory_size_string, suffix_match, std::regex{ "([a-zA-Z])+" });

  size_t quantity;
  try {
    quantity = std::stoull(quantity_match.str());
  } catch (std::invalid_argument const& ex) {
    return tl::make_unexpected((boost::format("Could not parse quantity of memory size %1% (%2%)") %
                                memory_size_string % ex.what())
                                 .str());
  }

  static const std::unordered_map<std::string, size_t> si_suffix_multipliers = {
    { "B", static_cast<size_t>(1) },
    { "KB", static_cast<size_t>(1e3) },
    { "MB", static_cast<size_t>(1e6) },
    { "GB", static_cast<size_t>(1e9) },
    { "TB", static_cast<size_t>(1e12) },
    { "KiB", static_cast<size_t>(1ull << 10) },
    { "MiB", static_cast<size_t>(1ull << 20) },
    { "GiB", static_cast<size_t>(1ull << 30) },
    { "TiB", static_cast<size_t>(1ull << 40) }
  };

  const auto suffix_iter = si_suffix_multipliers.find(suffix_match.str());
  if (suffix_iter == std::end(si_suffix_multipliers)) {
    return tl::make_unexpected((boost::format("Could not parse memory size: Unrecognized SI suffix "
                                              "(%1%). Use one of \"B\", "
                                              "\"KB\", \"MB\", \"GB\", \"TB\", \"KiB\", \"MiB\", "
                                              "\"GiB\", \"TiB\"") %
                                suffix_match.str())
                                 .str());
  }

  return { quantity * suffix_iter->second * boost::units::information::byte };
}

static tl::expected<ThreadConfig, std::string>
parse_threads_count(const std::string& thread_count_string)
{
  // Thread counts are either one or two positive numbers separated by spaces
  std::vector<std::string> tokens;
  boost::split(tokens, thread_count_string, boost::is_any_of(" "));

  switch (tokens.size()) {
    case 1:
      return util::try_parse_number<uint32_t>(tokens[0])
        .map(
          [](uint32_t num_threads) { return ThreadConfig{ AdaptiveThreadCount{ num_threads } }; })
        .map_error([](const auto& err) {
          return std::visit(
            overloaded{
              [](const std::invalid_argument& ex) {
                return "Could not parse thread count as number";
              },
              [](const std::out_of_range& ex) { return "Thread count is out of range"; },
            },
            err);
        });
    case 2:
      return util::try_parse_number<uint32_t>(tokens[0])
        .and_then([&tokens](uint32_t read_threads) {
          return util::try_parse_number<uint32_t>(tokens[1]).map(
            [read_threads](uint32_t index_threads) {
              return ThreadConfig{ FixedThreadCount{ read_threads, index_threads } };
            });
        })
        .map_error([](const auto& err) {
          return std::visit(
            overloaded{
              [](const std::invalid_argument& ex) {
                return "Could not parse thread count as number";
              },
              [](const std::out_of_range& ex) { return "Thread count is out of range"; },
            },
            err);
        });
    default:
      return tl::make_unexpected("Could not parse threads count");
  }
}

namespace util {

void
validate(boost::any& v,
         const std::vector<std::string>& all_tokens,
         util::IgnoreErrors* target_type,
         int)
{

  // To make both '--ignore A B C' and '--ignore "A B C" and also '--ignore "A
  // B" C' valid, we take all_tokens and treat each entry as potentially
  // multiple tokens that we split on whitespace. This way we get all the
  // singular tokens

  auto ignore_errors = util::IgnoreErrors::None;

  for (auto& one_or_more_tokens : all_tokens) {

    std::vector<std::string> tokens;
    boost::algorithm::split(tokens, one_or_more_tokens, boost::is_any_of(" "));

    for (auto& just_one_token : tokens) {
      util::try_parse<util::IgnoreErrors>(just_one_token)
        .map([&ignore_errors](util::IgnoreErrors val) { ignore_errors = ignore_errors | val; })
        .or_else([](const std::string& reason_for_failure) {
          throw bpo::validation_error{ bpo::validation_error::kind_t::invalid_option_value,
                                       reason_for_failure };
        });
    }
  }

  v = boost::any(ignore_errors);
}

} // namespace util

class SparseGrid;

std::variant<TilerProcess::Arguments, ConverterArguments>
parseArguments(int argc, char** argv)
{
  TilerProcess::Arguments tiler_args;
  ConverterArguments converter_args;
  std::string output_folder;
  std::vector<std::string> source_files;
  std::string cache_size_string;
  std::string rgb_mapping_string;
  bool create_journal;

  bpo::options_description options("Options");
  options.add_options()("help,h", "Produce help message")(
    "tiler",
    bpo::bool_switch()->default_value(false),
    "Run the tiler process to generate an octree from the source file(s).")(
    "converter",
    bpo::bool_switch()->default_value(false),
    "Run the converter process to convert the octree into a different file "
    "format.");

  bpo::options_description tiler_options("Tiler options");
  tiler_options.add_options()(
    "source,i",
    bpo::value<std::vector<std::string>>(&source_files)->multitoken(),
    "List of one or more input files and/or folders. For each folder, all "
    "LAS/LAZ files in the "
    "folder and all its subfolders are processed.")(
    "outdir,o",
    bpo::value<std::string>(&output_folder),
    "Output directory. If unspecified, the current working directory is "
    "used.")("spacing,s",
             bpo::value<float>(&tiler_args.spacing)->default_value(0.f),
             "Distance between points at root level. Distance halves each level.")(
    "spacing-by-diagonal-fraction,d",
    bpo::value<int>(&tiler_args.diagonal_fraction)->default_value(0),
    "Maximum number of points on the diagonal in the first level (sets "
    "spacing). spacing = "
    "diagonal / value")("max-points-per-node",
                        bpo::value<size_t>(&tiler_args.max_points_per_node)->default_value(20'000),
                        "Maximum number of points in a leaf node.")(
    "internal-cache-size",
    bpo::value<size_t>(&tiler_args.internal_cache_size)->default_value(10'000'000),
    "Number of points to cache before indexer has to run")(
    "batch-read-size",
    bpo::value<size_t>(&tiler_args.max_batch_read_size)->default_value(1'000'000),
    "Maximum number of points to read in a single batch from each file")(
    "output-format",
    bpo::value<std::string>()->default_value("3DTILES"),
    "Output format for the conversion. Accepted values are: 3DTILES (Cesium 3D "
    "Tiles format), "
    "ENTWINE_LAS (Entwine format using LAS files, compatible with Potree), "
    "ENTWINE_LAZ (Entwine "
    "format using LAZ files, compatible with Potree), BIN (custom binary "
    "format, uncompressed), BINZ (custom binary format, compressed)")(
    "sampling",
    bpo::value<std::string>(&tiler_args.sampling_strategy)->default_value("MIN_DISTANCE"),
    "Sampling strategy to use. Possible values are RANDOM_GRID, GRID_CENTER, "
    "MIN_DISTANCE. The quality of the resulting point cloud can be adjusted "
    "with this parameter, with RANDOM_GRID corresponding to the lowest "
    "quality and MIN_DISTANCE to the highest quality.")(
    "calculate-rgb-from",
    bpo::value<std::string>(&rgb_mapping_string),
    "Calculate RGB values from one of the other point attributes. Accepted "
    "values are: "
    "INTENSITY_LINEAR (convert intensity values to RGB using a linear "
    "mapping), INTENSITY_LOG "
    "(convert intensity values to RGB using a logarithmic mapping), NONE (do "
    "not calculate RGB "
    "values, same as not specifying this option). This feature is only "
    "supported when "
    "output-format is 3DTILES")(
    "cache-size",
    bpo::value<std::string>(&cache_size_string),
    "Size of a local cache in memory used during conversion for storing "
    "points in. You can specify "
    "this using common SI-suffixes (e.g. 800MiB or 256MB)")(
    "journal",
    bpo::bool_switch(&create_journal)->default_value(false),
    "Create a detailed journal in the output folder with information about "
    "the tiling process. "
    "This can be used to analyze performance")(
    "source-projection",
    bpo::value<std::string>(),
    "Source spatial reference system that the points are in")(
    "ignore",
    bpo::value<util::IgnoreErrors>(&tiler_args.errors_to_ignore)
      ->multitoken()
      ->composing()
      ->default_value(util::IgnoreErrors::None, "NONE"),
    "If provided, all recoverable errors for the given categories are "
    "ignored and processing proceeds normally instead of terminating the "
    "program. Specify one or "
    "more of the following possible values:\nMISSING_FILES (= ignore missing "
    "files)\nINACCESSIBLE_FILES (= ignore inaccessible files)"
    "\nUNSUPPORTED_FILE_FORMAT (= ignore files with unsupported file formats)"
    "\nCORRUPTED_FILES (= ignore corrupted/broken files)"
    "\nMISSING_POINT_ATTRIBUTES (= ignore files that don't have the point "
    "attributes specified by the -a option. Default values will be used for "
    "these attributes)"
    "\nALL_FILE_ERRORS (= ignore all recoverable file-related errors)"
    "\nALL_ERRORS (= ignore all recoverable errors)"
    "\nNONE (= terminate program on every error)")(
    "tiling-strategy",
    bpo::value<std::string>()->default_value("FAST"),
    "The tiling strategy to use. Valid options are FAST or ACCURATE, where "
    "FAST will yield better "
    "performance but larger data.")(
    "threads",
    bpo::value<std::string>(),
    "The number of threads to use. Specify either a single number to use this many threads with "
    "adaptive scheduling (the threads will be distributed between reading and indexing "
    "dynamically), or specify two numbers for the reading threads and indexing threads, in this "
    "order. E.g.: \"--threads 6\" will use 6 threads, dynamically assigned to reading and "
    "indexing. \"--threads 2 6\" will use a total of 8 threads, 2 for reading, 6 for indexing. If "
    "unspecified, as many threads as there are logical cores on the current machine will be used "
    "and dynamically assigned to reading and indexing. Please not that reading is parallelized on "
    "a file-level, so there can never be more read threads than there are files.");

  bpo::options_description converter_options("Converter options");
  converter_options.add_options()("source,i",
                                  bpo::value<std::string>(&converter_args.source_folder),
                                  "Input directory. This directory has to contain the result of a "
                                  "previous invocation of the tiler process.")(
    "outdir,o",
    bpo::value<std::string>(&converter_args.output_folder),
    "Output directory. If unspecified, the current working directory is "
    "used.")("output-format",
             bpo::value<std::string>()->default_value("3DTILES"),
             "Output format for the conversion. Can be one of LAS, LAZ or "
             "3DTILES. Default is 3DTILES")(
    "source-projection",
    bpo::value<std::string>(),
    "Source spatial reference system that the points are in")(
    "max-depth",
    bpo::value<int32_t>(),
    "Maximum tree depth to convert. 0: only root node, "
    "1: root node and its direct children etc.")("delete-source",
                                                 bpo::bool_switch()->default_value(false),
                                                 "Delete the source files once converted?");

  if (argc == 1) {
    std::cout << options << std::endl;
    std::cout << tiler_options << std::endl;
    std::cout << converter_options << std::endl;
    std::exit(EXIT_SUCCESS);
  }

  bpo::variables_map generic_variables;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).allow_unregistered().run(),
               generic_variables);
    bpo::notify(generic_variables);
  } catch (const std::exception& ex) {
    std::cout << ex.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (generic_variables.count("help")) {
    std::cout << options << std::endl;
    std::cout << tiler_options << std::endl;
    std::cout << converter_options << std::endl;
    std::exit(EXIT_SUCCESS);
  }

  // Check if tiler or converter process
  // TODO Ensure that 'tiler' or 'converter' argument is the first argument
  // passed!
  if (generic_variables["tiler"].as<bool>()) {
    if (generic_variables["converter"].as<bool>()) {
      std::cout << "Can't specify both 'tiler' and 'converter' arguments at "
                   "the same time! Please "
                   "run with either 'tiler' or 'converter' arguments!"
                << std::endl;
      std::exit(1);
    }

    bpo::variables_map tiler_variables;
    try {
      bpo::store(
        bpo::command_line_parser(argc, argv).options(tiler_options).allow_unregistered().run(),
        tiler_variables);
      bpo::notify(tiler_variables);
    } catch (const std::exception& ex) {
      std::cout << ex.what() << std::endl;
      std::exit(EXIT_FAILURE);
    }

    tiler_args.sources.reserve(source_files.size());
    std::transform(std::begin(source_files),
                   std::end(source_files),
                   std::back_inserter(tiler_args.sources),
                   [](const auto& str) { return fs::path{ str }; });
    tiler_args.output_directory =
      output_folder.empty() ? fs::current_path() : fs::path{ output_folder };

    global_config().is_journaling_enabled = create_journal;
    global_config().root_directory = tiler_args.output_directory;
    global_config().journal_directory = tiler_args.output_directory / "journal";

    // If diagonal fraction and spacing are set, diagonal fraction wins! If none
    // are set, the default value of diagonal fraction kicks in
    if (tiler_args.diagonal_fraction != 0) {
      tiler_args.spacing = 0;
    } else if (tiler_args.spacing == 0) {
      tiler_args.diagonal_fraction = 250;
    }

    tiler_args.output_format = [&]() {
      const std::unordered_map<std::string, OutputFormat> supported_output_formats = {
        { "3DTILES", OutputFormat::CZM_3DTILES },
        { "BIN", OutputFormat::BIN },
        { "LAS", OutputFormat::LAS },
        { "LAZ", OutputFormat::LAZ },
        { "ENTWINE_LAS", OutputFormat::ENTWINE_LAS },
        { "ENTWINE_LAZ", OutputFormat::ENTWINE_LAZ }
      };
      const auto& output_format_arg = tiler_variables["output-format"].as<std::string>();
      const auto matching_output_format = supported_output_formats.find(output_format_arg);
      if (matching_output_format == supported_output_formats.end()) {
        std::cout << "Output format \"" << output_format_arg << "\" not recognized!" << std::endl;
        std::exit(EXIT_FAILURE);
      }
      return matching_output_format->second;
    }();

    if (tiler_variables.count("calculate-rgb-from")) {
      tiler_args.rgb_mapping = [&]() {
        const std::unordered_map<std::string, RGBMapping> supported_rgb_mappings = {
          { "NONE", RGBMapping::None },
          { "INTENSITY_LINEAR", RGBMapping::FromIntensityLinear },
          { "INTENSITY_LOG", RGBMapping::FromIntensityLogarithmic }
        };
        const auto matching_rgb_mapping_iter = supported_rgb_mappings.find(rgb_mapping_string);
        if (matching_rgb_mapping_iter == std::end(supported_rgb_mappings)) {
          std::cout << "Parameter \"" << rgb_mapping_string
                    << "\" for option --calculate-rgb-from not recognized!" << std::endl;
          std::exit(EXIT_FAILURE);
        }
        return matching_rgb_mapping_iter->second;
      }();
    }

    if (tiler_variables.count("cache-size")) {
      parse_memory_size(cache_size_string)
        .map([&tiler_args](unit::byte cache_size) { tiler_args.cache_size = cache_size; })
        .or_else([](std::string const& failure) { std::cout << failure << "\n"; });
    }

    tiler_args.source_projection =
      (tiler_variables.count("source-projection"))
        ? (std::make_optional(tiler_variables["source-projection"].as<std::string>()))
        : std::nullopt;

    try {
      auto absolutePath = fs::canonical(fs::system_complete(argv[0]));
      tiler_args.executable_path = absolutePath.parent_path().string();
    } catch (const fs::filesystem_error& e) {
      // do nothing
    }

    tiler_args.tiling_strategy = [&]() -> TilingStrategy {
      const std::unordered_map<std::string, TilingStrategy> supported_tiling_strategies = {
        { "ACCURATE", TilingStrategy::Accurate }, { "FAST", TilingStrategy::Fast }
      };
      const auto& arg = tiler_variables["tiling-strategy"].as<std::string>();
      const auto matching_strategy = supported_tiling_strategies.find(arg);
      if (matching_strategy == supported_tiling_strategies.end()) {
        std::cout << "Tiling strategy \"" << arg << "\" not recognized!" << std::endl;
        std::exit(EXIT_FAILURE);
      }
      return matching_strategy->second;
    }();

    if (tiler_variables.count("threads")) {

      parse_threads_count(tiler_variables["threads"].as<std::string>())
        .map([&tiler_args](const auto& thread_config) { tiler_args.thread_config = thread_config; })
        .or_else([](const auto& err) {
          std::cout << err << "\n";
          std::exit(EXIT_FAILURE);
        });
    } else {
      // Debatable what the best ratio of read to index threads is. Will be adjusted by the
      // TilerProcess based on the actual number of files anyways. We will use 1 thread for reading
      // and the rest for indexing
      const auto num_logical_cores = std::thread::hardware_concurrency();
      tiler_args.thread_config = { AdaptiveThreadCount{ num_logical_cores } };
    }

    return { tiler_args };
  } else if (generic_variables["converter"].as<bool>()) {

    bpo::variables_map converter_variables;
    try {
      bpo::store(
        bpo::command_line_parser(argc, argv).options(converter_options).allow_unregistered().run(),
        converter_variables);
      bpo::notify(converter_variables);
    } catch (const std::exception& ex) {
      std::cout << ex.what() << std::endl;
      std::exit(EXIT_FAILURE);
    }

    // Handle the remaining converter args
    converter_args.output_format = [&]() {
      const std::unordered_map<std::string, OutputFormat> supported_output_formats = {
        { "3DTILES", OutputFormat::CZM_3DTILES },
        { "LAS", OutputFormat::LAS },
        { "LAZ", OutputFormat::LAZ }
      };
      const auto& output_format_arg = converter_variables["output-format"].as<std::string>();
      const auto matching_output_format = supported_output_formats.find(output_format_arg);
      if (matching_output_format == supported_output_formats.end()) {
        std::cout << "Output format \"" << output_format_arg << "\" not recognized!" << std::endl;
        std::exit(EXIT_FAILURE);
      }
      return matching_output_format->second;
    }();

    converter_args.source_projection =
      (converter_variables.count("source-projection"))
        ? (std::make_optional(converter_variables["source-projection"].as<std::string>()))
        : std::nullopt;

    if (converter_variables.count("max-depth")) {
      const auto max_depth = converter_variables["max-depth"].as<int32_t>();
      if (max_depth >= 0) {
        converter_args.max_depth = std::make_optional(static_cast<uint32_t>(max_depth));
      }
    }

    converter_args.delete_source_files = converter_variables["delete-source"].as<bool>();

    // TODO Deal with converter output attributes

    return { converter_args };
  } else {
    std::cout << "Please specify either 'tiler' or 'converter' to indicate "
                 "which process to run!"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

int
main(int argc, char** argv)
{
  std::cout.imbue(std::locale(""));

  try {
    const auto args = parseArguments(argc, argv);

    std::visit(
      [](const auto& typed_args) {
        if constexpr (std::is_same_v<std::decay_t<decltype(typed_args)>, TilerProcess::Arguments>) {
          TilerProcess tiler_process{ typed_args };
          tiler_process.run();
        } else {
          run_conversion(typed_args);
        }
      },
      args);

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return EXIT_SUCCESS;
}
