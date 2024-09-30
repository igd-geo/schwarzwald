#include "reflection/StaticReflection.h"

#include <ostream>
#include <cstdint>
#include <utility>

refl::TypeDescriptor::TypeDescriptor(const char* name, size_t size)
  : name(name)
  , size(size)
{}
refl::TypeDescriptor::~TypeDescriptor() {}

std::string
refl::TypeDescriptor::full_name() const
{
  return name;
}

#define _TYPE_DESCRIPTOR_NAME_FOR_PRIMITIVE(primitive_type) TypeDescriptor_##primitive_type
#define TYPE_DESCRIPTOR_NAME_FOR_PRIMITIVE(primitive_type)                                         \
  _TYPE_DESCRIPTOR_NAME_FOR_PRIMITIVE(primitive_type)

#define DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(primitive_type)                                      \
  namespace refl {                                                                                 \
  struct TYPE_DESCRIPTOR_NAME_FOR_PRIMITIVE(primitive_type)                                        \
    : TypeDescriptor                                                                               \
  {                                                                                                \
    TYPE_DESCRIPTOR_NAME_FOR_PRIMITIVE(primitive_type)                                             \
    ()                                                                                             \
      : TypeDescriptor(#primitive_type, sizeof(primitive_type))                                    \
    {}                                                                                             \
    void print_value(const void* obj, std::ostream& ostream) const override                        \
    {                                                                                              \
      ostream << *static_cast<const primitive_type*>(obj);                                         \
    }                                                                                              \
  };                                                                                               \
  template<>                                                                                       \
  TypeDescriptor const* detail::describe_basic_type<primitive_type>()                              \
  {                                                                                                \
    static TYPE_DESCRIPTOR_NAME_FOR_PRIMITIVE(primitive_type) s_descriptor;                        \
    return &s_descriptor;                                                                          \
  }                                                                                                \
  }

DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(int8_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(uint8_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(int16_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(uint16_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(int32_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(uint32_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(int64_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(uint64_t)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(float)
DECLARE_TYPE_DESCRIPTOR_FOR_PRIMITIVE(double)

// Have to declare type descriptor manually for bool because we want bool to be printed as
// 'false'/'true' instead of '0'/'1'
namespace refl {
struct TypeDescriptor_bool : TypeDescriptor
{
  TypeDescriptor_bool()
    : TypeDescriptor("bool", sizeof(bool))
  {}
  void print_value(const void* obj, std::ostream& ostream) const override
  {
    ostream << std::boolalpha << *static_cast<const bool*>(obj);
  }
};

template<>
TypeDescriptor const*
detail::describe_basic_type<bool>()
{
  static TypeDescriptor_bool s_descriptor;
  return &s_descriptor;
}
}

refl::TypeDescriptorStruct::TypeDescriptorStruct(void (*init)(TypeDescriptorStruct*))
  : TypeDescriptor(nullptr, 0)
{
  init(this);
}

void
refl::TypeDescriptorStruct::print_value(const void* obj, std::ostream& ostream) const
{
  const auto base_ptr = reinterpret_cast<const char*>(this);
  for (auto& member : members) {
    member.type->print_value(static_cast<const void*>(base_ptr + member.offset), ostream);
  }
}

// std types

refl::TypeDescriptorString::TypeDescriptorString()
  : TypeDescriptor("std::string", sizeof(std::string))
{}

void
refl::TypeDescriptorString::print_value(const void* obj, std::ostream& ostream) const
{
  ostream << *static_cast<const std::string*>(obj);
}

template<>
refl::TypeDescriptor const*
refl::detail::describe_basic_type<std::string>()
{
  static TypeDescriptorString s_descriptor;
  return &s_descriptor;
}

namespace refl {
template<typename T>
struct TypeDescriptorVector : TypeDescriptor
{
  using UnderlyingType = T;

  TypeDescriptor const* underlying_type;

  TypeDescriptorVector()
    : TypeDescriptor("std::vector", sizeof(std::vector<T>))
  {
    // TODO Would be nicer to have the type name as 'std::vector<int>' instead of just 'std::vector'
    underlying_type = Describe<T>::get();
  }

  void print_value(const void* obj, std::ostream& ostream) const override
  {
    const auto& vector = *static_cast<const std::vector<T>*>(obj);
    ostream << "[";
    for (size_t idx = 0; idx < vector.size(); ++idx) {
      ostream << vector[idx];
      if (idx < vector.size() - 1) {
        ostream << ";";
      }
    }
    ostream << "]";
  }
};

template<typename T>
struct Describe<std::vector<T>>
{
  static TypeDescriptor const* get()
  {
    static TypeDescriptorVector<T> s_descriptor;
    return &s_descriptor;
  }
};

template<typename First, typename Second>
struct TypeDescriptorPair : TypeDescriptor
{
  using FirstType = First;
  using SecondType = Second;

  TypeDescriptor const* first_type;
  TypeDescriptor const* second_type;

  TypeDescriptorPair()
    : TypeDescriptor("std::pair", sizeof(std::pair<First, Second>))
  {
    first_type = Describe<First>::get();
    second_type = Describe<Second>::get();
  }

  void print_value(const void* obj, std::ostream& ostream) const override
  {
    const auto& pair = *static_cast<const std::pair<First, Second>*>(obj);
    ostream << "[";
    ostream << pair.first << ";" << pair.second;
    ostream << "]";
  }
};

template<typename First, typename Second>
struct Describe<std::pair<First, Second>>
{
  static TypeDescriptor const* get()
  {
    static TypeDescriptorPair<First, Second> s_descriptor;
    return &s_descriptor;
  }
};

}