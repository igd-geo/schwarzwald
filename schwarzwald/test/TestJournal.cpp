#include <catch2/catch_all.hpp>

#include "logging/Journal.h"
#include "reflection/StaticReflection.h"

#include <experimental/filesystem>
#include <fstream>
#include <streambuf>

namespace fs = std::experimental::filesystem;

[[maybe_unused]] static std::string
read_file_to_string(const fs::path& path)
{
  std::ifstream fs(path);
  return { (std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>() };
}

struct FlatTrivialType
{
  int value;
  bool condition;
  std::string label;

  REFLECT()
};

REFLECT_STRUCT_BEGIN(FlatTrivialType)
REFLECT_STRUCT_MEMBER(value)
REFLECT_STRUCT_MEMBER(condition)
REFLECT_STRUCT_MEMBER(label)
REFLECT_STRUCT_END()

struct NestedTrivialType
{
  int value;
  FlatTrivialType nested_type;
};

struct NonTrivialType
{
  std::vector<int> vec;
};

struct NastyNonTrivialType
{
  int val;
  NonTrivialType non_trivial;
};

SCENARIO("Journal with simple type")
{
  // Run everything within a scope. At scope exit, the Journal gets destroyed, thus writing
  // its contents to disk
  {
    logging::JournalStore journal_storage;
    // Or logging::JournalStore::global()

    // Builder pattern makes the most sense with such a complex type
    auto test_journal = journal_storage.new_journal("test-journal")
                          .with_flat_type<FlatTrivialType>()
                          .as_csv("./test-journal-base-path")
                          .into_single_file()
                          .build();

    FlatTrivialType test_record{ 42, false, "test" };

    test_journal->add_record(test_record);

    {
      // Try to get the journal dynamically (by name) and add a record
      auto journal = journal_storage.get_journal("test-journal");
      REQUIRE(journal != nullptr);

      // For this to work, we have to store some typeids inside the Journal
      journal->add_record_untyped(test_record);

      std::string invalid_record = "invalid";
      REQUIRE_THROWS_AS(journal->add_record_untyped(invalid_record), std::invalid_argument);
    }
  }

  fs::path expected_file_path = "./test-journal-base-path/test-journal.csv";
  REQUIRE(fs::exists(expected_file_path));

  std::string expected_file_contents = "value;condition;label\n42;false;test\n42;false;test\n";
  const auto actual_file_contents = read_file_to_string(expected_file_path);

  REQUIRE(actual_file_contents == expected_file_contents);
}

TEST_CASE("IsJSONPrimitiveType", "Journal")
{
  // Should probably be a separate test file

  REQUIRE(logging::IsJSONPrimitiveType<uint8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<int8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<uint16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<int16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<uint32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<int32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<uint64_t>);
  REQUIRE(logging::IsJSONPrimitiveType<int64_t>);

  REQUIRE(logging::IsJSONPrimitiveType<const uint8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const int8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const uint16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const int16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const uint32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const int32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const uint64_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const int64_t>);

  REQUIRE(logging::IsJSONPrimitiveType<volatile uint8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile int8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile uint16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile int16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile uint32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile int32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile uint64_t>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile int64_t>);

  REQUIRE(logging::IsJSONPrimitiveType<const volatile uint8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile int8_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile uint16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile int16_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile uint32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile int32_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile uint64_t>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile int64_t>);

  REQUIRE(!logging::IsJSONPrimitiveType<uint8_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<int8_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<uint16_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<int16_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<uint32_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<int32_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<uint64_t&>);
  REQUIRE(!logging::IsJSONPrimitiveType<int64_t&>);

  REQUIRE(!logging::IsJSONPrimitiveType<uint8_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<int8_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<uint16_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<int16_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<uint32_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<int32_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<uint64_t*>);
  REQUIRE(!logging::IsJSONPrimitiveType<int64_t*>);

  REQUIRE(logging::IsJSONPrimitiveType<float>);
  REQUIRE(logging::IsJSONPrimitiveType<double>);
  REQUIRE(logging::IsJSONPrimitiveType<const float>);
  REQUIRE(logging::IsJSONPrimitiveType<const double>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile float>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile double>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile float>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile double>);

  REQUIRE(logging::IsJSONPrimitiveType<std::string>);
  REQUIRE(logging::IsJSONPrimitiveType<const std::string>);
  REQUIRE(logging::IsJSONPrimitiveType<volatile std::string>);
  REQUIRE(logging::IsJSONPrimitiveType<const volatile std::string>);

  REQUIRE(!logging::IsJSONPrimitiveType<std::string&>);
  REQUIRE(!logging::IsJSONPrimitiveType<const std::string&>);
  REQUIRE(!logging::IsJSONPrimitiveType<volatile std::string&>);
  REQUIRE(!logging::IsJSONPrimitiveType<const volatile std::string&>);

  REQUIRE(!logging::IsJSONPrimitiveType<std::string*>);
  REQUIRE(!logging::IsJSONPrimitiveType<const std::string*>);
  REQUIRE(!logging::IsJSONPrimitiveType<volatile std::string*>);
  REQUIRE(!logging::IsJSONPrimitiveType<const volatile std::string*>);

  REQUIRE(logging::IsJSONPrimitiveType<std::vector<uint8_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<int8_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<uint16_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<int16_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<uint32_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<int32_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<uint64_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<int64_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<float>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<double>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::vector<std::string>>);

  REQUIRE(logging::IsJSONPrimitiveType<std::optional<uint8_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<int8_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<uint16_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<int16_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<uint32_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<int32_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<uint64_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<int64_t>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<float>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<double>>);
  REQUIRE(logging::IsJSONPrimitiveType<std::optional<std::string>>);

  struct SimpleJSONAggregate
  {
    uint8_t m1;
    int16_t m2;
    uint32_t m3;
    int64_t m4;
    std::string m5;
    std::vector<int32_t> m6;
    std::optional<std::string> m7;
  };

  struct SimpleNonJSONAggregate
  {
    const std::string& m1;
  };

  struct NestedJSONAggregate
  {
    uint8_t m1;
    SimpleJSONAggregate m2;
  };

  struct NestedNonJSONAggregate
  {
    uint8_t m1;
    SimpleNonJSONAggregate m2;
  };

  REQUIRE(logging::IsJSONPrimitiveType<SimpleJSONAggregate>);
  REQUIRE(logging::IsJSONPrimitiveType<NestedJSONAggregate>);
  // Won't compile because these types are not default constructible!
  // REQUIRE(!logging::IsJSONPrimitiveType<SimpleNonJSONAggregate>);
  // REQUIRE(!logging::IsJSONPrimitiveType<NestedNonJSONAggregate>);
}

TEST_CASE("IsConvertibleToString", "Journal")
{
  REQUIRE(logging::IsConvertibleToString<uint8_t>);
  REQUIRE(logging::IsConvertibleToString<int8_t>);
  REQUIRE(logging::IsConvertibleToString<uint16_t>);
  REQUIRE(logging::IsConvertibleToString<int16_t>);
  REQUIRE(logging::IsConvertibleToString<uint32_t>);
  REQUIRE(logging::IsConvertibleToString<int32_t>);
  REQUIRE(logging::IsConvertibleToString<uint64_t>);
  REQUIRE(logging::IsConvertibleToString<int64_t>);

  REQUIRE(logging::IsConvertibleToString<float>);
  REQUIRE(logging::IsConvertibleToString<double>);

  REQUIRE(logging::IsConvertibleToString<bool>);
  REQUIRE(logging::IsConvertibleToString<std::string>);

  REQUIRE(!logging::IsConvertibleToString<FlatTrivialType>);
  REQUIRE(!logging::IsConvertibleToString<NestedTrivialType>);
}

TEST_CASE("IsFlatPrimitiveType", "Journal")
{
  REQUIRE(logging::IsFlatTrivialType<FlatTrivialType>);
  REQUIRE(!logging::IsFlatTrivialType<NestedTrivialType>);
  REQUIRE(!logging::IsFlatTrivialType<NonTrivialType>);
  REQUIRE(!logging::IsFlatTrivialType<NastyNonTrivialType>);
}

TEST_CASE("IsNestedPrimitiveType", "Journal")
{
  // Any flat trivial type is - by definition - also a nested trivial type
  REQUIRE(logging::IsNestedTrivialType<FlatTrivialType>);
  REQUIRE(logging::IsNestedTrivialType<NestedTrivialType>);
  REQUIRE(!logging::IsNestedTrivialType<NonTrivialType>);
  REQUIRE(!logging::IsNestedTrivialType<NastyNonTrivialType>);
}