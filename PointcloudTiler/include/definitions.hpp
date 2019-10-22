#pragma once

#include <string>

enum class OutputFormat
{
  CZM_3DTILES,
  LAS,
  LAZ
};

namespace progress {
const static std::string LOADING{ "loading" };
const static std::string INDEXING{ "indexing" };
const static std::string POSTPROCESSING{ "postprocessing" };
}