#pragma once

#include "types/type_util.h"

#include <string>
#include <vector>

#include <boost/hana.hpp>

/**
 * This is heavily inspired by the following blog post by Jeff Preshing:
 *
 * https://preshing.com/20180116/a-primitive-reflection-system-in-cpp-part-1/
 */

namespace refl {

/**
 * Reflection information about a type. Abstract base class that other
 * TypeDescriptors will inherit from
 */
struct TypeDescriptor
{
  // Have to be public so that init function of REFLECT macro has access to the members
  const char* name;
  size_t size;

  TypeDescriptor(const char* name, size_t size);
  virtual ~TypeDescriptor();

  virtual std::string full_name() const;

  virtual void print_value(const void* obj, std::ostream& ostream) const = 0;
};

struct TypeDescriptorStruct : TypeDescriptor
{
  struct MemberInfo
  {
    const char* name;
    size_t offset;
    TypeDescriptor const* type;
  };
  std::vector<MemberInfo> members;

  TypeDescriptorStruct(void (*init)(TypeDescriptorStruct*));

  void print_value(const void* obj, std::ostream& ostream) const override;
};

// TypeDescriptors for std types
struct TypeDescriptorString : TypeDescriptor
{
  TypeDescriptorString();
  void print_value(const void* obj, std::ostream& ostream) const override;
};

namespace detail {
template<typename T>
struct IsReflected
{
  constexpr bool operator()() const { return decltype(s_has_reflect_member(T{}))::value; }

private:
  // TODO How do I also check that the type of refl_type_descriptor is correct? If I do:
  //  decltype(std::is_same_v<refl::TypeDescriptorStruct, decltype(T::refl_type_descriptor)>)
  // then this will fail only if refl_type_descriptor doesn't exist, but not if the is_same_v
  // returns false...
  constexpr static const auto s_has_reflect_member =
    boost::hana::is_valid([](auto x) -> decltype(decltype(x)::refl_type_descriptor) {});
};
}

/**
 * Gets the TypeDescriptor for the given type. Returns nullptr if there is no
 * reflection information about the given type
 */
// template<typename T>
// std::enable_if_t<!detail::IsReflected<T>{}(), TypeDescriptor const*>
// describe()
// {
//   return nullptr;
// }

namespace detail {
template<typename T>
TypeDescriptor const*
describe_basic_type();
// {
//   static_assert(AlwaysFalse<T>::value,
//                 "No reflection information present for type T. Consider adding reflection info to
//                 " "custom types through the REFLECT...() macro family");
//   return nullptr;
// }
}

struct TypeDescriptorResolver
{
  template<typename T, typename std::enable_if_t<detail::IsReflected<T>{}(), int> = 0>
  static TypeDescriptor const* get()
  {
    return &T::refl_type_descriptor;
  }

  template<typename T, typename std::enable_if_t<!detail::IsReflected<T>{}(), int> = 0>
  static TypeDescriptor const* get()
  {
    return detail::describe_basic_type<T>();
  }
};

/**
 * Describe type T and retrieve it's TypeDescriptor
 */
template<typename T>
struct Describe
{
  static TypeDescriptor const* get() { return TypeDescriptorResolver::get<T>(); }
};

#define REFLECT()                                                                                  \
  static refl::TypeDescriptorStruct refl_type_descriptor;                                          \
  static void init_refl(refl::TypeDescriptorStruct* type_descriptor);

#define REFLECT_STRUCT_BEGIN(struct_name)                                                          \
  refl::TypeDescriptorStruct struct_name::refl_type_descriptor{ struct_name::init_refl };          \
  void struct_name::init_refl(refl::TypeDescriptorStruct* type_descriptor)                         \
  {                                                                                                \
    using T = struct_name;                                                                         \
    type_descriptor->name = #struct_name;                                                          \
    type_descriptor->size = sizeof(T);                                                             \
    type_descriptor->members = {

#define REFLECT_STRUCT_MEMBER(member_name)                                                         \
  { #member_name, offsetof(T, member_name), refl::Describe<decltype(T::member_name)>::get() },

#define REFLECT_STRUCT_END()                                                                       \
  }                                                                                                \
  ;                                                                                                \
  }

}