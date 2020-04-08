#pragma once

#include <exception>
#include <optional>
#include <stdint.h>
#include <string>
#include <unordered_map>

#include <boost/format.hpp>
#include <expected.hpp>

#include "algorithms/Enums.h"

namespace util {

/**
 * Flags for recoverable errors that the pointcloud tiler can ignore if
 * the user wants it
 */
enum class IgnoreErrors : uint32_t {
  /**
   * Ignore no errors, i.e. fail fast
   */
  None = 0,
  /**
   * Ignore errors due to missing files
   */
  MissingFiles = 1 << 0,
  /**
   * Ignore errors due to inaccessible files
   */
  InaccessibleFiles = 1 << 1,
  /**
   * Ignore errors due to file formats that are not supported by the pointcloud
   * tiler
   */
  UnsupportedFileFormat = 1 << 2,
  /**
   * Ignore errors due to corrupted files, i.e. files that appear correct but
   * contain internal errors
   */
  CorruptedFiles = 1 << 3,
  /**
   * Ignore errors due to files that don't contain the necessary attributes as
   * specified by the '-a' option
   */
  MissingPointAttributes = 1 << 3,

  // error combinations
  AllFileErrors =
      MissingFiles | InaccessibleFiles | UnsupportedFileFormat | CorruptedFiles,
  AllErrors = 0xFFFFFFFF,
};

namespace {
using namespace std::string_literals;

static const std::unordered_map<std::string, IgnoreErrors> TOKENS_TO_LITERALS =
    {
        {"NONE"s, IgnoreErrors::None},
        {"MISSING_FILES"s, IgnoreErrors::MissingFiles},
        {"INACCESSIBLE_FILES"s, IgnoreErrors::InaccessibleFiles},
        {"UNSUPPORTED_FILE_FORMAT"s, IgnoreErrors::UnsupportedFileFormat},
        {"CORRUPTED_FILES"s, IgnoreErrors::CorruptedFiles},
        {"MISSING_POINT_ATTRIBUTES"s, IgnoreErrors::MissingPointAttributes},
        {"ALL_FILE_ERRORS"s, IgnoreErrors::AllFileErrors},
        {"ALL_ERRORS"s, IgnoreErrors::AllErrors},
};
} // namespace

template <>
inline tl::expected<IgnoreErrors, std::string>
try_parse(const std::string &str) {
  const auto iter = TOKENS_TO_LITERALS.find(str);
  if (iter == std::end(TOKENS_TO_LITERALS)) {
    return tl::make_unexpected(
        (boost::format("Unrecognized token \"%1%\"") % str).str());
  }

  return {iter->second};
}

template <> inline const std::string &to_string(IgnoreErrors err) {
  for (auto &kv : TOKENS_TO_LITERALS) {
    if (kv.second == err)
      return kv.first;
  }
  throw std::invalid_argument{"Unrecognized IgnoreErrors literal"};
}

/**
 * Checks if r is contained within l
 */
inline bool operator&(IgnoreErrors l, IgnoreErrors r) {
  return (static_cast<uint32_t>(l) & static_cast<uint32_t>(r)) != 0;
}

inline IgnoreErrors operator|(IgnoreErrors l, IgnoreErrors r) {
  return static_cast<IgnoreErrors>(static_cast<uint32_t>(l) |
                                   static_cast<uint32_t>(r));
}

struct ErrorChain : std::exception {
  explicit ErrorChain(const std::exception &reason);
  explicit ErrorChain(std::string reason);
  ErrorChain(const ErrorChain &reason, const std::string &why);

  const char *what() const noexcept override;

private:
  std::string _reasons;
};

ErrorChain chain_error(const std::exception &exception, const std::string &why);
ErrorChain chain_error(std::string why);
ErrorChain chain_error(const ErrorChain &reason, const std::string &why);

} // namespace util