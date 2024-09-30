#include <catch2/catch_all.hpp>
#include "concepts/MemoryIntrospection.h"

#include <iostream>

namespace bui = boost::units::information;

using namespace std;

static size_t MemoryAllocated = 0;

void*
operator new(size_t size)
{
  MemoryAllocated += size;
  void* p = malloc(size);
  return p;
}

SCENARIO("MemoryIntrospection std::string")
{
  WHEN("A string is default-constructed")
  {
    std::string empty;
    THEN("The string has no dynamic memory")
    {
      REQUIRE(concepts::size_in_memory(empty) == (sizeof(std::string) * bui::byte));
    }
  }

  WHEN("A short string is created")
  {
    std::string i_am_short{ "short" };
    THEN("The string has no dynamic memory")
    {
      REQUIRE(concepts::size_in_memory(i_am_short) == (sizeof(std::string) * bui::byte));
    }
  }

  WHEN("A long string is created")
  {
    const auto memory_before = MemoryAllocated;
    std::string so_long{ "so long wow very string such length wow allocation is large so large" };
    const auto dynamic_memory_allocated = MemoryAllocated - memory_before;
    THEN("The string has dynamic memory")
    {
      REQUIRE(concepts::size_in_memory(so_long) ==
              (sizeof(std::string) + dynamic_memory_allocated) * bui::byte);
    }
  }
}