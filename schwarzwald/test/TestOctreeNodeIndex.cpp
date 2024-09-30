#include <catch2/catch_all.hpp>

#include "datastructures/OctreeNodeIndex.h"

#include <string>
#include <unordered_map>

SCENARIO("OctreeNodeIndex - Constructors", "[OctreeNodeIndex]")
{
  WHEN("An OctreeNodeIndex is default constructed")
  {
    OctreeNodeIndex64 default_constructed;
    THEN("It has depth zero and index zero")
    {
      REQUIRE(default_constructed.levels() == 0);
      REQUIRE(default_constructed.index() == 0);
      REQUIRE(default_constructed.octant_at_level(0) == 0);
    }
    THEN("Calling parent() and on it raises an exception")
    {
      REQUIRE_THROWS_AS(default_constructed.parent(), std::exception);
    }
    THEN("Calling parent_at_level(0) does nothing")
    {
      REQUIRE(default_constructed == default_constructed.parent_at_level(0));
    }
  }

  WHEN("An OctreeNodeIndex is constructed directly from an index and levels")
  {
    uint64_t index = 2 | (3 << 3) | (7 << 6); //'732'
    uint32_t levels = 3;
    const auto from_index_and_levels =
      OctreeNodeIndex64::unchecked_from_index_and_levels(index, levels);
    THEN("Index and level are stored correctly")
    {
      REQUIRE(from_index_and_levels.index() == index);
      REQUIRE(from_index_and_levels.levels() == levels);
    }
    THEN("Octants are stored in little-endian order")
    {
      REQUIRE(from_index_and_levels.octant_at_level(0) == 0);
      REQUIRE(from_index_and_levels.octant_at_level(1) == 7);
      REQUIRE(from_index_and_levels.octant_at_level(2) == 3);
      REQUIRE(from_index_and_levels.octant_at_level(3) == 2);
    }
  }

  WHEN("An OctreeNodeIndex is constructed from a range of octants")
  {
    OctreeNodeIndex64 list_initialized{
      uint8_t{ 2 }, uint8_t{ 3 }, uint8_t{ 7 }, uint8_t{ 5 }
    };
    uint64_t expected_index = (2 << 9) | (3 << 6) | (7 << 3) | 5;
    THEN("Index and levels are set accordingly")
    {
      REQUIRE(list_initialized.index() == expected_index);
      REQUIRE(list_initialized.levels() == 4);
    }
    THEN("Octants are set correctly")
    {
      REQUIRE(list_initialized.octant_at_level(0) == 0);
      REQUIRE(list_initialized.octant_at_level(1) == 2);
      REQUIRE(list_initialized.octant_at_level(2) == 3);
      REQUIRE(list_initialized.octant_at_level(3) == 7);
      REQUIRE(list_initialized.octant_at_level(4) == 5);
    }
  }

  WHEN("An OctreeNodeIndex is constructed from a MortonIndex")
  {
    std::array<uint8_t, 21> expected_octants{ 5, 3, 7, 4, 0, 1, 6, 4, 3, 5, 3,
                                              6, 7, 3, 2, 1, 4, 0, 2, 5, 6 };
    MortonIndex64 static_morton_index{ expected_octants };

    WHEN("By using all levels")
    {
      OctreeNodeIndex64 from_static_morton_index{ static_morton_index };
      THEN("Index and levels are inherited from the MortonIndex")
      {
        REQUIRE(from_static_morton_index.index() == static_morton_index.get());
        REQUIRE(from_static_morton_index.levels() == 21);
      }
    }

    WHEN("By using the first N levels")
    {
      const uint32_t levels = 5;
      OctreeNodeIndex64 from_static_morton_index_with_levels{
        static_morton_index, levels
      };
      THEN("Index and levels are truncated accordingly")
      {
        uint64_t expected_index =
          (static_morton_index.get() >> (3 * (21 - levels)));
        REQUIRE(from_static_morton_index_with_levels.index() == expected_index);
        REQUIRE(from_static_morton_index_with_levels.levels() == levels);

        REQUIRE(from_static_morton_index_with_levels.octant_at_level(0) == 0);
        for (uint32_t level = 1; level <= levels; ++level) {
          REQUIRE(from_static_morton_index_with_levels.octant_at_level(level) ==
                  expected_octants[level - 1]);
        }
      }
    }
  }

  WHEN("An OctreeNodeIndex is copy-constructed")
  {
    OctreeNodeIndex64 original{
      uint8_t{ 2 }, uint8_t{ 3 }, uint8_t{ 7 }, uint8_t{ 5 }
    };
    OctreeNodeIndex64 copy{ original };

    THEN("Index and levels match")
    {
      REQUIRE(original.index() == copy.index());
      REQUIRE(original.levels() == copy.levels());
    }
  }
}

SCENARIO("OctreeNodeIndex - Comparisons", "[OctreeNodeIndex]")
{
  WHEN("Two OctreeNodeIndices have the same depth")
  {
    WHEN("They have the exact same octants")
    {
      OctreeNodeIndex64 a{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
      OctreeNodeIndex64 b = a;

      THEN("Comparisons hold")
      {
        // a==b
        REQUIRE(a == b);

        REQUIRE(a <= b);
        REQUIRE(a >= b);
        REQUIRE(b <= a);
        REQUIRE(b >= a);

        REQUIRE(!(a < b));
        REQUIRE(!(a > b));
        REQUIRE(!(b < a));
        REQUIRE(!(b > a));
      }
    }

    WHEN("They they differ only in the last octant")
    {
      OctreeNodeIndex64 a{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
      OctreeNodeIndex64 b{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 3 };

      THEN("Comparisons hold")
      {
        // a > b
        REQUIRE(a != b);

        REQUIRE(a >= b);
        REQUIRE(!(a <= b));
        REQUIRE(!(b >= a));
        REQUIRE(b <= a);

        REQUIRE(a > b);
        REQUIRE(!(a < b));
        REQUIRE(!(b > a));
        REQUIRE(b < a);
      }
    }

    WHEN("They differ in multiple octants")
    {
      OctreeNodeIndex64 a{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
      OctreeNodeIndex64 b{ 3, 2, 2, 4, 7, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };

      THEN("Comparisons hold")
      {
        // a < b
        REQUIRE(a != b);

        REQUIRE(!(a >= b));
        REQUIRE(a <= b);
        REQUIRE(b >= a);
        REQUIRE(!(b <= a));

        REQUIRE(!(a > b));
        REQUIRE(a < b);
        REQUIRE(b > a);
        REQUIRE(!(b < a));
      }
    }
  }

  WHEN("Two OctreeNodeIndices have different depth")
  {
    WHEN("One OctreeNodeIndex is a child of the other")
    {
      OctreeNodeIndex64 a{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
      OctreeNodeIndex64 b{ 1, 2, 3, 4, 5 };

      THEN("Comparisons hold")
      {
        // a is a subnode of b. This means that 'a!=b', but also !(a<b) and
        // !(b<a)
        REQUIRE(a != b);

        REQUIRE(a <= b);
        REQUIRE(a >= b);
        REQUIRE(b <= a);
        REQUIRE(b >= a);

        REQUIRE(!(a < b));
        REQUIRE(!(a > b));
        REQUIRE(!(b < a));
        REQUIRE(!(b > a));
      }
    }

    WHEN("The two indices are unrelated")
    {
      OctreeNodeIndex64 a{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                           2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
      OctreeNodeIndex64 b{ 6, 3, 0, 2, 7 };

      THEN("Comparisons hold")
      {
        // a < b
        REQUIRE(a != b);

        REQUIRE(!(a >= b));
        REQUIRE(a <= b);
        REQUIRE(b >= a);
        REQUIRE(!(b <= a));

        REQUIRE(!(a > b));
        REQUIRE(a < b);
        REQUIRE(b > a);
        REQUIRE(!(b < a));
      }
    }
  }
}

SCENARIO("OctreeNodeIndex - Parent index", "[OctreeNodeIndex]")
{
  GIVEN("An OctreeNodeIndex with many levels")
  {
    OctreeNodeIndex64 index{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                             2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };

    WHEN("parent() is called")
    {
      const auto parent = index.parent();
      THEN("The index of the parent node is obtained")
      {
        OctreeNodeIndex64 expected_parent_index{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5,
                                                 1, 2, 3, 4, 5, 1, 2, 3, 4, 5 };
        REQUIRE(parent == expected_parent_index);
      }
    }

    WHEN("parent_at_level() is called")
    {
      const auto parent = index.parent_at_level(3);
      THEN("The index of the parent node at the specific level is obtained")
      {
        OctreeNodeIndex64 expected_parent_index{ 1, 2, 3 };
        REQUIRE(parent == expected_parent_index);
      }
    }

    WHEN("parent_at_level(0) is called")
    {
      const auto should_be_root = index.parent_at_level(0);
      THEN("The root node index is returned")
      {
        OctreeNodeIndex64 root_index;
        REQUIRE(should_be_root == root_index);
      }
    }
  }
}

SCENARIO("OctreeNodeIndex - Siblings", "[OctreeNodeIndex]")
{
  GIVEN("An OctreeNodeIndex somewhere in the tree")
  {
    OctreeNodeIndex64 index{ 1, 2, 3 };

    WHEN("sibling(0) is called")
    {
      const auto sibling = index.sibling(0);
      THEN("The sibling at octant 0 is returned")
      {
        OctreeNodeIndex64 expected_sibling{ 1, 2, 0 };
        REQUIRE(sibling == expected_sibling);
      }
    }
  }

  GIVEN("The root node index")
  {
    OctreeNodeIndex64 root;

    WHEN("sibling() is called")
    {
      THEN("An error is thrown")
      {
        REQUIRE_THROWS_AS(root.sibling(0), std::exception);
      }
    }
  }
}

SCENARIO("OctreeNodeIndex - Children", "[OctreeNodeIndex]")
{
  GIVEN("An OctreeNodeIndex somewhere in the tree")
  {
    OctreeNodeIndex64 index{ 1, 2, 3 };

    WHEN("child(4) is called")
    {
      const auto child = index.child(4);

      THEN("The child index is correct")
      {
        OctreeNodeIndex64 expected_index{ 1, 2, 3, 4 };
        REQUIRE(child == expected_index);
      }
    }
  }

  GIVEN("An OctreeNodeIndex that has MAX_LEVELS")
  {
    OctreeNodeIndex64 index{
      1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1
    }; // 21 levels == MAX_LEVELS

    WHEN("child() is called for any child octant")
    {
      THEN("An exception is thrown")
      {
        REQUIRE_THROWS_AS(index.child(0), std::exception);
      }
    }
  }
}

SCENARIO("OctreeNodeIndex - Hashing", "[OctreeNodeIndex]")
{
  WHEN("An OctreeNodeIndex is stored inside an unordered_map")
  {
    std::unordered_map<OctreeNodeIndex64, std::string> map;
    OctreeNodeIndex64 index{ 1, 2, 3 };
  }
}

SCENARIO("OctreeNodeIndex - String conversions", "[OctreeNodeIndex]")
{
  WHEN("An OctreeNodeIndex with zero levels is converted to a string")
  {
    OctreeNodeIndex64 root;
    const auto as_str = OctreeNodeIndex64::to_string(root);

    THEN("The string is empty") { REQUIRE(as_str == ""); }
  }

  WHEN("An OctreeNodeIndex with multiple levels is converted to a string")
  {
    OctreeNodeIndex64 index{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                             2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
    const auto as_str = OctreeNodeIndex64::to_string(index);
    std::string expected_str{ "123451234512345123456" };

    THEN("The string is in little-endian order")
    {
      REQUIRE(as_str == expected_str);
    }
  }

  WHEN("An empty string is converted to an OctreeNodeIndex")
  {
    std::string root_str{ "" };
    const auto index = OctreeNodeIndex64::from_string(root_str);

    THEN("The conversion was successful") { REQUIRE(index.has_value()); }

    THEN("The index represents the root node")
    {
      REQUIRE(index->index() == 0);
      REQUIRE(index->levels() == 0);
    }
  }

  WHEN("A valid string is converted to an OctreeNodeIndex")
  {
    std::string index_str{ "123451234512345123456" };
    const auto index = OctreeNodeIndex64::from_string(index_str);

    THEN("The conversion was successful") { REQUIRE(index.has_value()); }

    THEN("The index is correct")
    {
      OctreeNodeIndex64 expected_index{ 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1,
                                        2, 3, 4, 5, 1, 2, 3, 4, 5, 6 };
      REQUIRE((*index) == expected_index);
    }
  }

  WHEN("An invalid string is converted to an OctreeNodeIndex")
  {
    std::string invalid{ "invalid" };
    const auto index = OctreeNodeIndex64::from_string(invalid);

    THEN("The conversion fails") { REQUIRE(!index.has_value()); }
  }

  WHEN("A valid, but too large string is converted to an OctreeNodeIndex")
  {
    // Trying to convert string with 21 levels into index that can only store 10
    // levels
    std::string index_str{ "123451234512345123456" };
    const auto index = OctreeNodeIndex32::from_string(index_str);

    THEN("The conversion fails") { REQUIRE(!index.has_value()); }
  }

  WHEN("A valid string in Entwine format is converted to an OctreeNodeIndex")
  {
    std::string index_str{ "13-410-7041-4059" };
    const auto index = OctreeNodeIndex64::from_string(
      index_str, MortonIndexNamingConvention::Entwine);

    THEN("The conversion was successful") { REQUIRE(index.has_value()); }

    THEN("The index is correct")
    {
      OctreeNodeIndex64 expected_index{ 2, 3, 1, 3, 7, 7, 1, 0, 5, 5, 0, 5, 3 };
      REQUIRE((*index) == expected_index);
    }
  }

  WHEN("A valid string in Entwine format that is too large is converted to an "
       "OctreeNodeIndex")
  {
    std::string index_str{ "22-0-0-0" };
    const auto index = OctreeNodeIndex64::from_string(
      index_str, MortonIndexNamingConvention::Entwine);

    THEN("The conversion fails") { REQUIRE(!index.has_value()); }
  }
}