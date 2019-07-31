
#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <string>

namespace Potree {

enum class OutputFormat
{
  BINARY,
  LAS,
  LAZ
};

enum class StoreOption
{
  ABORT_IF_EXISTS,
  OVERWRITE,
  INCREMENTAL
};

enum class ConversionQuality
{
  FAST,
  DEFAULT,
  NICE
};
}

namespace progress {
const static std::string LOADING{ "loading" };
const static std::string INDEXING{ "indexing" };
const static std::string POSTPROCESSING{ "postprocessing" };
}

#endif