#pragma once

#include <cctype>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
//#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>

#include <experimental/filesystem>

#include "datastructures/GridCell.h"
#include "datastructures/SparseGrid.h"
#include "math/AABB.h"
#include "math/Vector3.h"
#include "pointcloud/Point.h"
#include "util/Definitions.h"

AABB readAABB(std::string fIn, int numPoints);

AABB readAABB(std::string fIn);

/**
 *   y
 *   |-z
 *   |/
 *   O----x
 *
 *   3----7
 *  /|   /|
 * 2----6 |
 * | 1--|-5
 * |/   |/
 * 0----4
 *
 */
AABB childAABB(const AABB &aabb, const int &index);

/**
 *   y
 *   |-z
 *   |/
 *   O----x
 *
 *   3----7
 *  /|   /|
 * 2----6 |
 * | 1--|-5
 * |/   |/
 * 0----4
 *
 */
int nodeIndex(const AABB &aabb, const Vector3<double> &pointPosition);

/**
 * from
 * http://stackoverflow.com/questions/5840148/how-can-i-get-a-files-size-in-c
 */
long filesize(std::string filename);

/**
 * from
 * http://stackoverflow.com/questions/874134/find-if-string-endswith-another-string-in-c
 */
bool endsWith(std::string const &fullString, std::string const &ending);

/**
 * see
 * http://stackoverflow.com/questions/735204/convert-a-string-in-c-to-upper-case
 */
std::string toUpper(std::string str);

bool copyDir(fs::path source, fs::path destination);

float psign(float value);

// see
// https://stackoverflow.com/questions/23943728/case-insensitive-standard-string-comparison-in-c
bool icompare_pred(unsigned char a, unsigned char b);

// see
// https://stackoverflow.com/questions/23943728/case-insensitive-standard-string-comparison-in-c
bool icompare(std::string const &a, std::string const &b);

bool endsWith(const std::string &str, const std::string &suffix);

bool iEndsWith(const std::string &str, const std::string &suffix);

// see
// https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
std::string ltrim(std::string s);

// see
// https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
std::string rtrim(std::string s);

// see
// https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
std::string trim(std::string s);

/// <summary>
/// Converts 16-bit intensity values to greyscale using logarithmization. This
/// compresses the dynamic range so that the visualization is more pleasing
/// (otherwise low intensity values would be very dark)
/// </summary>
Vector3<uint8_t> intensityToRGB_Log(uint16_t intensity);

/// <summary>
/// Aligns the given number using the given alignment
/// </summary>
template <typename Number>
constexpr Number align(Number value, Number alignment) {
  if (!value)
    return value;
  if (!alignment)
    return value;

  const auto remainder = value % alignment;
  if (!remainder)
    return value;
  return value + alignment - remainder;
}

template <typename T>
constexpr size_t vector_byte_size(const std::vector<T> &vec) {
  return vec.size() * sizeof(T);
}

enum class Recursive { No, Yes };

std::vector<fs::path>
get_all_files_in_directory(const std::string &dir_path,
                           Recursive recursive = Recursive::No);

template <typename... Args> std::string concat(const Args &... args) {
  std::stringstream ss;
  (ss << ... << args);
  return ss.str();
}

inline uint8_t expand_bits_by_3(uint8_t val) {
  val &= 0b11; // 2 bits
  return (val | (val << 2)) & uint8_t(0b00001001);
}

inline uint16_t expand_bits_by_3(uint16_t val) {
  val &= 0b11111; // 5 bits
  val = (val | (val << 8)) & uint16_t(0b00010000'00001111);
  val = (val | (val << 4)) & uint16_t(0b00010000'11000011);
  val = (val | (val << 2)) & uint16_t(0b00010010'01001001);
  return val;
}

inline uint32_t expand_bits_by_3(uint32_t val) {
  val &= 0b1111111111; // 10 bits
  val = (val | (val << 16)) & uint32_t(0x030000FF);
  val = (val | (val << 8)) & uint32_t(0x0300F00F);
  val = (val | (val << 4)) & uint32_t(0x030C30C3);
  val = (val | (val << 2)) & uint32_t(0x09249249);
  return val;
}

inline uint64_t expand_bits_by_3(uint64_t val) {
  val &= 0b111111111111111111111; // 21 bits
  val = (val | (val << 32)) & uint64_t(0x00FF0000'0000FFFF);
  val = (val | (val << 16)) & uint64_t(0x00FF0000'FF0000FF);
  val = (val | (val << 8)) & uint64_t(0xF00F00F0'0F00F00F);
  val = (val | (val << 4)) & uint64_t(0303030303030303030303);
  val =
      (val | (val << 2)) &
      uint64_t(
          0b00010010'01001001'00100100'10010010'01001001'00100100'10010010'01001001ULL); // Every third
                                                                                         // bit set
  return val;
}

template <typename Key, typename Value, typename Compare, typename Alloc>
std::vector<Key> keys(std::map<Key, Value, Compare, Alloc> const &map) {
  std::vector<Key> _keys;
  _keys.reserve(map.size());
  std::transform(std::begin(map), std::end(map), std::back_inserter(_keys),
                 [](const auto &kv) { return kv.first; });
  return _keys;
}

template <typename Key, typename Value, typename Hash, typename Pred,
          typename Alloc>
std::vector<Key>
keys(std::unordered_map<Key, Value, Hash, Pred, Alloc> const &map) {
  std::vector<Key> _keys;
  _keys.reserve(map.size());
  std::transform(std::begin(map), std::end(map), std::back_inserter(_keys),
                 [](const auto &kv) { return kv.first; });
  return _keys;
}

template <typename Key, typename Value, typename Compare, typename Alloc>
std::vector<Value> values(std::map<Key, Value, Compare, Alloc> const &map) {
  std::vector<Value> _values;
  _values.reserve(map.size());
  std::transform(std::begin(map), std::end(map), std::back_inserter(_values),
                 [](const auto &kv) { return kv.second; });
  return _values;
}

template <typename Key, typename Value, typename Hash, typename Pred,
          typename Alloc>
std::vector<Value>
values(std::unordered_map<Key, Value, Hash, Pred, Alloc> const &map) {
  std::vector<Value> _values;
  _values.reserve(map.size());
  std::transform(std::begin(map), std::end(map), std::back_inserter(_values),
                 [](const auto &kv) { return kv.second; });
  return _values;
}