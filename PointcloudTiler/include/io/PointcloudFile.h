#pragma once

#include "AABB.h"
#include "PointAttributes.hpp"
#include "PointBuffer.h"

#include <type_traits>
#include <variant>

namespace {
template<typename T>
struct AlwaysFalse : std::false_type
{};
}

namespace pc {

template<typename File>
typename File::metadata const&
metadata(File const& file)
{
  return file.get_metadata();
}

template<typename File>
AABB
get_bounds(File const& f);

template<typename File>
Vector3<double>
get_offset(File const& f);

template<typename File>
size_t
get_point_count(File const& f);

template<typename File>
bool
has_attribute(File const& f, PointAttribute const& attribute);

template<typename File>
bool
has_all_attributes(File const& f,
                   PointAttributes const& attributes,
                   PointAttributes* missing_attributes = nullptr)
{
  if (!missing_attributes) {
    return std::all_of(
      std::begin(attributes), std::end(attributes), [&f](PointAttribute const& attribute) {
        return has_attribute(f, attribute);
      });
  }

  std::copy_if(std::begin(attributes),
               std::end(attributes),
               std::back_inserter(*missing_attributes),
               [&f](PointAttribute const& attribute) { return !has_attribute(f, attribute); });
  return !missing_attributes->empty();
}

template<typename Iter, typename Meta>
Iter
read_points(Iter begin,
            size_t count,
            Meta const& metadata,
            PointAttributes const& attributes,
            PointBuffer& points);

// Variant-style functions
template<typename... FileTypes>
AABB
get_bounds(std::variant<FileTypes...> const& f)
{
  return std::visit([](auto& typed_file) { return get_bounds(typed_file); }, f);
}

template<typename... FileTypes>
Vector3<double>
get_offset(std::variant<FileTypes...> const& f)
{
  return std::visit([](auto& typed_file) { return get_offset(typed_file); }, f);
}

template<typename... FileTypes>
size_t
get_point_count(std::variant<FileTypes...> const& f)
{
  return std::visit([](auto& typed_file) { return get_point_count(typed_file); }, f);
}

template<typename... FileTypes>
bool
has_attribute(std::variant<FileTypes...> const& f, PointAttribute const& attribute)
{
  return std::visit(
    [&attribute](auto& typed_file) { return has_attribute(typed_file, attribute); });
}

template<typename... FileTypes>
bool
has_all_attributes(std::variant<FileTypes...> const& f,
                   PointAttributes const& attributes,
                   PointAttributes* missing_attributes = nullptr)
{
  return std::visit([&attributes, missing_attributes](auto& typed_file) {
    return has_all_attributes(typed_file, attributes, missing_attributes);
  });
}
}