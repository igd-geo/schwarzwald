#pragma once

#include "datastructures/PointBuffer.h"
#include "math/AABB.h"
#include "pointcloud/PointAttributes.h"
#include "types/type_util.h"

#include <type_traits>
#include <variant>

namespace pc {

template <typename File>
typename File::metadata const &metadata(File const &file) {
  return file.get_metadata();
}

template <typename File> AABB get_bounds(File const &f);

template <typename File> Vector3<double> get_offset(File const &f);

template <typename File> size_t get_point_count(File const &f);

template <typename File>
bool has_attribute(File const &f, PointAttribute const &attribute);

template <typename File, typename OutIter>
bool has_all_attributes(File const &f, PointAttributes const &attributes,
                        OutIter missing_attributes_begin) {
  auto attribute_is_missing = false;
  for (const auto &attribute : attributes) {
    if (has_attribute(f, attribute))
      continue;
    attribute_is_missing = true;
    *missing_attributes_begin++ = attribute;
  }
  return !attribute_is_missing;
}

template <typename Iter, typename Meta>
Iter read_points(Iter begin, size_t count, Meta const &metadata,
                 PointAttributes const &attributes, PointBuffer &points);

// Variant-style functions
template <typename... FileTypes>
AABB get_bounds(std::variant<FileTypes...> const &f) {
  return std::visit([](auto &typed_file) { return get_bounds(typed_file); }, f);
}

template <typename... FileTypes>
Vector3<double> get_offset(std::variant<FileTypes...> const &f) {
  return std::visit([](auto &typed_file) { return get_offset(typed_file); }, f);
}

template <typename... FileTypes>
size_t get_point_count(std::variant<FileTypes...> const &f) {
  return std::visit(
      [](auto &typed_file) { return get_point_count(typed_file); }, f);
}

template <typename... FileTypes>
bool has_attribute(std::variant<FileTypes...> const &f,
                   PointAttribute const &attribute) {
  return std::visit(
      [&attribute](auto &typed_file) {
        return has_attribute(typed_file, attribute);
      },
      f);
}

template <typename... FileTypes, typename OutIter>
bool has_all_attributes(std::variant<FileTypes...> const &f,
                        PointAttributes const &attributes,
                        OutIter missing_attributes_begin) {
  return std::visit(
      [&attributes, missing_attributes_begin](auto &typed_file) {
        return has_all_attributes(typed_file, attributes,
                                  missing_attributes_begin);
      },
      f);
}
} // namespace pc