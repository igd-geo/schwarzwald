
#include <chrono>
#include <exception>
#include <experimental/filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "AABB.h"
#include "PotreeConverter.h"
#include "PotreeException.h"
#include "TileSetWriter.h"
#include "Tileset.h"
#include "Vector3.h"
#include "arguments.hpp"

namespace fs = std::experimental::filesystem;

using Potree::ConversionQuality;
using Potree::PotreeConverter;
using Potree::StoreOption;
using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::map;
using std::string;
using std::vector;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

#define MAX_FLOAT std::numeric_limits<float>::max()

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

struct PotreeArguments
{
  bool help = false;
  vector<string> source;
  string outdir;
  float spacing;
  int levels;
  int diagonalFraction;
  Potree::OutputFormat outFormat;
  vector<string> outputAttributes;
  string listOfFiles = "";
  ConversionQuality conversionQuality = ConversionQuality::DEFAULT;
  string conversionQualityString = "";
  string executablePath;
  std::optional<string> sourceProjection;
  uint32_t max_memory_usage_MiB = 256;
};

PotreeArguments
parseArguments(int argc, char** argv)
{
  Arguments args(argc, argv);

  args.addArgument("source,i", "input files");
  args.addArgument("help,h", "prints usage");
  args.addArgument("outdir,o", "output directory");
  args.addArgument("spacing,s",
                   "Distance between points at root level. Distance halves each level.");
  args.addArgument("spacing-by-diagonal-fraction,d",
                   "Maximum number of points on the diagonal in the first "
                   "level (sets spacing). spacing = diagonal / value");
  args.addArgument("levels,l",
                   "Number of levels that will be generated. 0: only root, 1: "
                   "root and its children, ...");
  args.addArgument("output-format", "Output format can be BINARY, LAS or LAZ. Default is BINARY");
  args.addArgument("output-attributes,a",
                   "valid values are RGB, INTENSITY, RGB_FROM_INTENSITY, "
                   "CLASSIFICATION, NORMAL. If RGB_FROM_INTENSITY is defined, "
                   "RGB must not be defined. Default is RGB.");
  args.addArgument("scale", "Scale of the X, Y, Z coordinate in LAS and LAZ files.");
  args.addArgument("source-projection",
                   "Source projection in proj4 format. Provide this if the "
                   "projection is not specified within your source file");
  args.addArgument("max-memory-usage",
                   "Maximum memory usage of the tool during conversion (in MiB). "
                   "Note that this is only an estimate, the tool might use slightly more memory "
                   "internally. The default value is 256 MiB, the minimum value is 32 MiB.");

  PotreeArguments a;

  if (args.has("help")) {
    cout << args.usage() << endl;
    exit(0);
  } else if (!args.has("source") && !args.has("list-of-files")) {
    cout << args.usage() << endl;
    exit(1);
  } else if (argc == 1) {
    cout << args.usage() << endl;
    exit(0);
  }

  a.outdir = args.get("outdir").as<string>();
  a.spacing = args.get("spacing").as<double>(0.0);
  a.diagonalFraction = args.get("d").as<double>(0.0);
  a.levels = args.get("levels").as<int>(-1);

  if (args.has("output-format")) {
    string of = args.get("output-format").as<string>("BINARY");

    if (of == "BINARY") {
      a.outFormat = Potree::OutputFormat::BINARY;
    } else if (of == "LAS") {
      a.outFormat = Potree::OutputFormat::LAS;
    } else if (of == "LAZ") {
      a.outFormat = Potree::OutputFormat::LAZ;
    } else {
      a.outFormat = Potree::OutputFormat::BINARY;
    }
  } else {
    a.outFormat = Potree::OutputFormat::BINARY;
  }

  if (args.has("output-attributes")) {
    a.outputAttributes = args.get("output-attributes").as<vector<string>>();
    verifyOutputAttributes(a.outputAttributes);
  } else {
    a.outputAttributes = { "RGB" };
  }

  if (args.has("source-projection")) {
    a.sourceProjection = std::make_optional(args.get("source-projection").as<string>());
  }

  if (args.has("max-memory-usage")) {
    a.max_memory_usage_MiB =
      static_cast<uint32_t>(std::max(32, args.get("max-memory-usage").as<int>(32)));
  }

  if (args.has("source")) {
    a.source = args.get("source").as<vector<string>>();
  }
  if (a.source.size() == 0 && args.has("list-of-files")) {
    string lof = args.get("list-of-files").as<string>();
    a.listOfFiles = lof;

    if (fs::exists(fs::path(a.listOfFiles))) {
      std::ifstream in(a.listOfFiles);
      string line;
      while (std::getline(in, line)) {
        string path;
        if (fs::path(line).is_absolute()) {
          path = line;
        } else {
          fs::path absPath = fs::canonical(fs::path(a.listOfFiles));
          fs::path lofDir = absPath.parent_path();
          path = lofDir.string() + "/" + line;
        }

        if (fs::exists(fs::path(path))) {
          a.source.push_back(path);
        } else {
          cerr << "ERROR: file not found: " << path << endl;
          exit(1);
        }
      }
      in.close();
    } else {
      cerr << "ERROR: specified list of files not found: '" << a.listOfFiles << "'" << endl;
      exit(1);
    }
  }

  // set default parameters
  fs::path pSource(a.source[0]);
  a.outdir =
    args.has("outdir") ? args.get("outdir").as<string>() : pSource.generic_string() + "_converted";

  if (a.diagonalFraction != 0) {
    a.spacing = 0;
  } else if (a.spacing == 0) {
    a.diagonalFraction = 200;
  }

  try {
    auto absolutePath = fs::canonical(fs::system_complete(argv[0]));
    a.executablePath = absolutePath.parent_path().string();
  } catch (const fs::filesystem_error& e) {
    // do nothing
  }

  return a;
}

void
printArguments(PotreeArguments& a)
{
  try {
    cout << "== params ==" << endl;
    int i = 0;
    for (const auto& s : a.source) {
      cout << "source[" << i << "]:         \t" << s << endl;
      ++i;
    }
    cout << "outdir:            \t" << a.outdir << endl;
    cout << "spacing:           \t" << a.spacing << endl;
    cout << "diagonal-fraction: \t" << a.diagonalFraction << endl;
    cout << "levels:            \t" << a.levels << endl;
    if (a.sourceProjection) {
      cout << "source projection: \t" << *a.sourceProjection << endl;
    }
    cout << "max memory usage:  \t" << a.max_memory_usage_MiB << " MiB" << endl;
    cout << endl;
  } catch (exception& e) {
    cout << "ERROR: " << e.what() << endl;

    exit(1);
  }
}

int
main(int argc, char** argv)
{
  cout.imbue(std::locale(""));

  try {
    PotreeArguments a = parseArguments(argc, argv);
    printArguments(a);
    PotreeConverter pc(a.outdir, a.source);

    // DEBUG:
    /*
    Point p1(0, 0, 0);
    Point p2(0.5, 0.5, 0.5);
    Point p3(1.0, 1.0, 1.0);

    AABB aabb(Vector3<double>(0, 0, 0), Vector3<double>(1, 1, 1));
    PNTWriter pntwriter("file", aabb, 1.0);
    pntwriter.write(p1);
    pntwriter.write(p2);
    pntwriter.write(p3);
    pntwriter.writePNT();
    */

    pc.spacing = a.spacing;
    pc.diagonalFraction = a.diagonalFraction;
    pc.maxDepth = a.levels;
    pc.outputFormat = a.outFormat;
    pc.outputAttributes = a.outputAttributes;
    pc.quality = a.conversionQuality;
    pc.sourceProjection = a.sourceProjection;
    pc.max_memory_usage_MiB = a.max_memory_usage_MiB;

    pc.convert();

  } catch (const std::exception& e) {
    cout << "ERROR: " << e.what() << endl;
    return 1;
  } catch (...) {
    std::cout << "Some obscure error..." << std::endl;
    return 1;
  }

  return 0;
}
