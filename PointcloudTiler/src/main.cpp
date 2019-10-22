
#include <chrono>
#include <exception>
#include <experimental/filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/program_options/variables_map.hpp>

#include "AABB.h"
#include "ConverterProcess.h"
#include "TileSetWriter.h"
#include "TilerProcess.h"
#include "Tileset.h"
#include "Vector3.h"

namespace fs = std::experimental::filesystem;
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

static void
verifyOutputAttributes(const std::vector<std::string>& outputAttributes)
{
  const auto hasAttribute = [](const auto& attributes, const char* attribute) {
    return std::find(attributes.begin(), attributes.end(), attribute) != attributes.end();
  };

  if (hasAttribute(outputAttributes, "RGB") &&
      hasAttribute(outputAttributes, "RGB_FROM_INTENSITY")) {
    throw std::runtime_error{ "Can't define both RGB and RGB_FROM_INTENSITY attributes!" };
  }
}

class SparseGrid;

struct TilerArguments
{
  std::vector<std::string> source;
  std::string outdir;
  float spacing;
  int levels;
  int diagonalFraction;
  size_t max_points_per_node;
  OutputFormat outFormat;
  std::vector<std::string> outputAttributes;
  std::string listOfFiles = "";
  std::string samplingStrategy;
  std::string executablePath;
  uint32_t max_memory_usage_MiB = 256;
};

struct ConverterArguments
{
  std::string source_folder;
  std::string output_folder;
  OutputFormat output_format;
  std::vector<std::string> output_attributes;
  std::optional<std::string> source_projection;
  std::optional<uint32_t> max_depth;
};

std::variant<TilerArguments, ConverterArguments>
parseArguments(int argc, char** argv)
{
  TilerArguments tiler_args;
  ConverterArguments converter_args;

  bpo::options_description options("Options");
  options.add_options()("help,h", "Produce help message")(
    "tiler",
    bpo::bool_switch()->default_value(false),
    "Run the tiler process to generate an octree from the source file(s).")(
    "converter",
    bpo::bool_switch()->default_value(false),
    "Run the converter process to convert the octree into a different file format.");

  bpo::options_description tiler_options("Tiler options");
  tiler_options.add_options()(
    "source,i",
    bpo::value<std::vector<std::string>>(&tiler_args.source),
    "List of one or more input files and/or folders. For each folder, all "
    "LAS/LAZ files in the "
    "folder and all its subfolders are processed.")(
    "outdir,o",
    bpo::value<std::string>(&tiler_args.outdir),
    "Output directory. If unspecified, the current working directory is "
    "used.")("spacing,s",
             bpo::value<float>(&tiler_args.spacing)->default_value(0.f),
             "Distance between points at root level. Distance halves each level.")(
    "spacing-by-diagonal-fraction,d",
    bpo::value<int>(&tiler_args.diagonalFraction)->default_value(0),
    "Maximum number of points on the diagonal in the first level (sets "
    "spacing). spacing = "
    "diagonal / value")("levels,l",
                        bpo::value<int>(&tiler_args.levels)->default_value(-1),
                        "Number of levels that will be generated. -1: No level limit, 0: only "
                        "root, 1: root and its children etc.")(
    "max-points-per-node",
    bpo::value<size_t>(&tiler_args.max_points_per_node)->default_value(20'000),
    "Maximum number of points in a leaf node.")(
    "output-format",
    bpo::value<std::string>()->default_value("LAS"),
    "Output format for the converted pointcloud. Can be LAS or "
    "LAZ.")("output-attributes,a",
            bpo::value<std::vector<std::string>>(&tiler_args.outputAttributes)->required(),
            "Point attributes to store in the output pointcloud. Can be a "
            "combination of RGB, INTENSITY, "
            "RGB_FROM_INTENSITY, CLASSIFICATION, NORMAL. RGB_FROM_INTENSITY "
            "generates greyscale colors "
            "from intensity values in the original pointcloud and is mutually "
            "exclusive with the RGB "
            "attribute")(
    "sampling",
    bpo::value<std::string>(&tiler_args.samplingStrategy)->default_value("RANDOM_GRID"),
    "Sampling strategy to use. Possible values are RANDOM_GRID, GRID_CENTER, "
    "MIN_DISTANCE and "
    "MIN_DISTANCE_FAST");

  bpo::options_description converter_options("Converter options");
  converter_options.add_options()("source,i",
                                  bpo::value<std::string>(&converter_args.source_folder),
                                  "Input directory. This directory has to contain the result of a "
                                  "previous invocation of the tiler process.")(
    "outdir,o",
    bpo::value<std::string>(&converter_args.output_folder),
    "Output directory. If unspecified, the current working directory is used.")(
    "output-format",
    bpo::value<std::string>()->default_value("3DTILES"),
    "Output format for the conversion. Currently, only 3DTILES is supported!")(
    "output-attributes,a",
    bpo::value<std::vector<std::string>>(&converter_args.output_attributes)->required(),
    "Point attributes to store in the output pointcloud. Can be a "
    "combination of RGB, INTENSITY, "
    "RGB_FROM_INTENSITY, CLASSIFICATION, NORMAL. RGB_FROM_INTENSITY "
    "generates greyscale colors "
    "from intensity values in the original pointcloud and is mutually "
    "exclusive with the RGB "
    "attribute")(
    "source-projection",
    bpo::value<std::string>(),
    "Source projection in proj4 format. Used for conversion to 3D-Tiles format to ensure that data "
    "is correctly transformed into Cesium world space! Only required if your data is not already "
    "in Cesium world space.")("max-depth",
                              bpo::value<int32_t>(),
                              "Maximum tree depth to convert. 0: only root node, "
                              "1: root node and its direct children etc.");

  if (argc == 1) {
    std::cout << options << std::endl;
    std::cout << tiler_options << std::endl;
    std::cout << converter_options << std::endl;
    std::exit(0);
  }

  bpo::variables_map generic_variables;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).allow_unregistered().run(),
               generic_variables);
    bpo::notify(generic_variables);
  } catch (const std::exception& ex) {
    std::cout << ex.what() << std::endl;
    std::exit(1);
  }

  if (generic_variables.count("help")) {
    std::cout << options << std::endl;
    std::cout << tiler_options << std::endl;
    std::cout << converter_options << std::endl;
    std::exit(0);
  }

  // Check if tiler or converter process
  // TODO Ensure that 'tiler' or 'converter' argument is the first argument passed!
  if (generic_variables["tiler"].as<bool>()) {
    if (generic_variables["converter"].as<bool>()) {
      std::cout << "Can't specify both 'tiler' and 'converter' arguments at the same time! Please "
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
      std::exit(1);
    }

    // Handle the remaining tiler arguments
    // Parse the string-for-enum parameters
    tiler_args.outFormat = [&]() {
      const std::unordered_map<std::string, OutputFormat> supported_output_formats = {
        { "LAS", OutputFormat::LAS }, { "LAZ", OutputFormat::LAZ }
      };
      const auto& output_format_arg = tiler_variables["output-format"].as<std::string>();
      const auto matching_output_format = supported_output_formats.find(output_format_arg);
      if (matching_output_format == supported_output_formats.end()) {
        std::cout << "Output format \"" << output_format_arg << "\" not supported!" << std::endl;
        std::exit(1);
      }
      return matching_output_format->second;
    }();

    verifyOutputAttributes(tiler_args.outputAttributes);

    if (tiler_args.outdir == "") {
      tiler_args.outdir = fs::current_path().generic_string();
    }

    // If diagonal fraction and spacing are set, diagonal fraction wins! If none
    // are set, the default value of diagonal fraction kicks in
    if (tiler_args.diagonalFraction != 0) {
      tiler_args.spacing = 0;
    } else if (tiler_args.spacing == 0) {
      tiler_args.diagonalFraction = 250;
    }

    try {
      auto absolutePath = fs::canonical(fs::system_complete(argv[0]));
      tiler_args.executablePath = absolutePath.parent_path().string();
    } catch (const fs::filesystem_error& e) {
      // do nothing
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
      std::exit(1);
    }

    // Handle the remaining converter args
    converter_args.output_format = [&]() {
      const std::unordered_map<std::string, OutputFormat> supported_output_formats = {
        { "3DTILES", OutputFormat::CZM_3DTILES }
      };
      const auto& output_format_arg = converter_variables["output-format"].as<std::string>();
      const auto matching_output_format = supported_output_formats.find(output_format_arg);
      if (matching_output_format == supported_output_formats.end()) {
        std::cout << "Output format \"" << output_format_arg << "\" not recognized!" << std::endl;
        std::exit(1);
      }
      return matching_output_format->second;
    }();

    verifyOutputAttributes(converter_args.output_attributes);

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

    return { converter_args };

  } else {
    std::cout << "Please specify either 'tiler' or 'converter' to indicate which process to run!"
              << std::endl;
    std::exit(1);
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
        if constexpr (std::is_same_v<std::decay_t<decltype(typed_args)>, TilerArguments>) {
          TilerProcess tiler_process{ typed_args.outdir, typed_args.source };

          tiler_process.spacing = typed_args.spacing;
          tiler_process.diagonalFraction = typed_args.diagonalFraction;
          tiler_process.maxDepth = typed_args.levels;
          tiler_process.max_points_per_node = typed_args.max_points_per_node;
          tiler_process.outputFormat = typed_args.outFormat;
          tiler_process.outputAttributes = typed_args.outputAttributes;
          tiler_process.samplingStrategy = typed_args.samplingStrategy;
          tiler_process.max_memory_usage_MiB = typed_args.max_memory_usage_MiB;

          tiler_process.run();
        } else {
          const auto point_attributes = point_attributes_from_strings(typed_args.output_attributes);

          run_conversion(typed_args.source_folder,
                         typed_args.output_folder,
                         typed_args.output_format,
                         point_attributes,
                         typed_args.source_projection,
                         typed_args.max_depth);
        }
      },
      args);

  } catch (const std::exception& e) {
    std::cout << "ERROR: " << e.what() << std::endl;
    exit(1);
  } catch (...) {
    std::cout << "Some obscure error..." << std::endl;
    exit(1);
  }

  return 0;
}
