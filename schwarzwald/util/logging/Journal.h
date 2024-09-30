#pragma once

#include "containers/Range.h"
#include "reflection/StaticReflection.h"
#include "types/MembersOf.h"

#include <any>
#include <experimental/filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>

#include <boost/format.hpp>
#include <boost/hana.hpp>

namespace logging {

struct JournalBase
{
  virtual ~JournalBase();

  template<typename T>
  void add_record_untyped(T&& record);

private:
  virtual void do_add_record(refl::TypeDescriptor const* record_type, std::any record) = 0;
};

/**
 * A Journal is a mechanism for storing and persisting information from the execution
 * of a program that is interesting for analysis, introspection, or debugging.
 *
 * A Journal stores records of some specified type and provides functionality to write
 * those records to disk in a specific format. The Journal also enables partitioning of
 * records, which controls into which file(s) individual records are written.
 */
template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
struct Journal : JournalBase
{
  Journal(std::string name,
          RecordStorage storage,
          RecordWriter writer,
          RecordPartitioner partitioner);
  ~Journal();

  template<typename T>
  void add_record(T&& record);

  const std::string& name() const { return _name; }

private:
  void do_add_record(refl::TypeDescriptor const* type, std::any record) override;
  void flush_records();

  std::string _name;
  RecordStorage _storage;
  RecordWriter _writer;
  RecordPartitioner _partitioner;
};

struct JournalBuilder;

template<typename RecordStorage>
struct JournalBuilderWithType;

template<typename RecordStorage, typename RecordWriter>
struct JournalBuilderWithTypeAndFormat;

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
struct JournalBuilderWithTypeAndFormatAndPartitioner;

/**
 * Stores all Journal instances for the runtime of the program
 */
struct JournalStore
{
  template<typename, typename, typename>
  friend struct JournalBuilderWithTypeAndFormatAndPartitioner;

  JournalStore() = default;

  static JournalStore& global();

  JournalBase* get_journal(const std::string& journal_name);

  JournalBuilder new_journal(const std::string& journal_name);

private:
  std::unordered_map<std::string, std::unique_ptr<JournalBase>> _journals;

  template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
  Journal<RecordStorage, RecordWriter, RecordPartitioner>* add_journal(
    Journal<RecordStorage, RecordWriter, RecordPartitioner> journal);
};

#pragma region Storage

template<typename T>
struct VectorStorage
{
  using RecordType = T;

  void add_record(const T& record);
  void add_record(T&& record);

  auto& records() { return _records; }
  void clear() { _records.clear(); }

private:
  std::vector<T> _records;
};

struct TupleStorage
{
  using RecordType = std::any;

  template<typename T>
  void add_record(T&& record);

  auto& records() { return _records; }
  void clear() { _records.clear(); }

private:
  std::vector<std::any> _records;
};

/**
 * Similar to VectorStorage, but with weaker type bounds
 */
template<typename T>
struct TreeStorage
{
  using RecordType = T;

  void add_record(const T& record);
  void add_record(T&& record);

  auto& records() { return _records; }
  void clear() { _records.clear(); }

private:
  std::vector<T> _records;
};

template<typename T>
struct DictionaryVectorStorage
{
  using RecordType = std::pair<const std::string, T>;

  void add_record(std::string key, const T& record);
  void add_record(std::string key, T&& record);

  auto& records() { return _records; }
  void clear() { _records.clear(); }

private:
  std::unordered_map<std::string, T> _records;
};

struct DictionaryTupleStorage
{
  using RecordType = std::pair<const std::string, std::any>;

  template<typename T>
  void add_record(std::string key, T&& record);

  auto& records() { return _records; }
  void clear() { _records.clear(); }

private:
  std::unordered_map<std::string, std::any> _records;
};

template<typename T>
struct DictionaryTreeStorage
{
  using RecordType = std::pair<const std::string, T>;

  void add_record(std::string key, const T& record);
  void add_record(std::string key, T&& record);

  auto& records() { return _records; }
  void clear() { _records.clear(); }

private:
  std::unordered_map<std::string, T> _records;
};

#pragma endregion

#pragma region Writer

struct BinaryWriter
{
  explicit BinaryWriter(const std::experimental::filesystem::path& base_path);

  template<typename RecordIterator>
  void write_records(util::Range<RecordIterator> records, const std::string& file_name);

private:
  std::experimental::filesystem::path _base_path;
};

struct CSVWriter
{
  template<typename T>
  static CSVWriter make_writer_for_type(const std::experimental::filesystem::path& base_path);

  template<typename RecordIterator>
  void write_records(util::Range<RecordIterator> records, const std::string& file_name);

private:
  template<typename T>
  explicit CSVWriter(const std::experimental::filesystem::path& base_path, T* dummy);

  std::experimental::filesystem::path _base_path;
  std::function<void(std::ostream&)> _header_writer;
  std::function<void(const std::any&, std::ostream&)> _record_writer;
};

struct JSONWriter
{
  explicit JSONWriter(const std::experimental::filesystem::path& base_path);

  template<typename RecordIterator>
  void write_records(util::Range<RecordIterator> records, const std::string& file_name);

private:
  std::experimental::filesystem::path _base_path;
};

struct TextWriter
{
  template<typename T>
  static TextWriter make_writer_for_type(const std::experimental::filesystem::path& base_path);

  template<typename RecordIterator>
  void write_records(util::Range<RecordIterator> records, const std::string& file_name);

private:
  template<typename T>
  explicit TextWriter(const std::experimental::filesystem::path& base_path, T*);

  std::experimental::filesystem::path _base_path;
  std::function<void(const std::any&, std::ostream&)> _record_writer;
};

#pragma endregion

#pragma region Partitioner

/**
 * Assigns all records of a Journal to a single file
 */
struct SingleFilePartitioner
{
  /**
   * Partitions the records into subsets based on the type of this Partitioner,
   * then writes each subset using the RecordWriter
   */
  template<typename RecordIterator, typename RecordWriter>
  void partition_and_write_records(util::Range<RecordIterator> records,
                                   RecordWriter& writer,
                                   const std::string& journal_name);

  /**
   * Checks if flushing to disk is required based on the given number of records
   */
  bool requires_flush(size_t number_of_records) const;
};

/**
 * Partition records of a single Journal into chunks of a fixed size, assigning
 * each chunk to a unique numbered file. This is useful if you have either very
 * large records or a very large number of records, for example in a large log
 * file. Instead of writing a single file with a million records, using a
 * ChunkedFilePartitioner, one hundred files with 10k records each can be written.
 */
struct ChunkedFilePartitioner
{
  /**
   * Partitions the records into subsets based on the type of this Partitioner,
   * then writes each subset using the RecordWriter
   */
  explicit ChunkedFilePartitioner(size_t chunk_size);

  template<typename RecordIterator, typename RecordWriter>
  void partition_and_write_records(util::Range<RecordIterator> records,
                                   RecordWriter& writer,
                                   const std::string& journal_name);
  /**
   * Checks if flushing to disk is required based on the given number of records
   */
  bool requires_flush(size_t number_of_records) const;

private:
  size_t _chunk_size;
};

/**
 * Partitions records of a single Journal so that each record gets written into
 * a separate file
 */
struct UniqueFilePartitioner
{
  UniqueFilePartitioner();
  /**
   * Partitions the records into subsets based on the type of this Partitioner,
   * then writes each subset using the RecordWriter
   */
  template<typename RecordIterator, typename RecordWriter>
  void partition_and_write_records(util::Range<RecordIterator> records,
                                   RecordWriter& writer,
                                   const std::string& journal_name);
  /**
   * Checks if flushing to disk is required based on the given number of records
   */
  bool requires_flush(size_t number_of_records) const;

private:
  size_t _next_file_index;
};

#pragma endregion

#pragma region Types

namespace detail {

template<typename T>
struct IsVector : std::false_type
{};

template<typename T>
struct IsVector<std::vector<T>> : std::true_type
{};

template<typename T>
struct IsOptional : std::false_type
{};

template<typename T>
struct IsOptional<std::optional<T>> : std::true_type
{};

template<typename T>
struct IsPair : std::false_type
{};

template<typename First, typename Second>
struct IsPair<std::pair<First, Second>> : std::true_type
{};

template<typename T>
struct IsJSONPrimitiveType
{
  constexpr bool operator()() const
  {
    if constexpr (std::is_arithmetic_v<T>) {
      return true;
    } else if constexpr (std::is_same_v<std::string, std::remove_cv_t<T>>) {
      return true;
    } else if constexpr (IsVector<std::decay_t<T>>::value || IsOptional<std::decay_t<T>>::value) {
      using ElementType = typename std::decay_t<T>::value_type;
      return IsJSONPrimitiveType<ElementType>{}();
    } else if constexpr (std::is_aggregate_v<T>) {
      // All members of the aggregate must be JSON primitive types!
      constexpr auto const members = refl::members_of<T>();
      return meta::all_of<IsJSONPrimitiveType>(members);
      // for (int idx = 0; idx < meta::size(members); ++idx) {
      //   if (!is_json_primitive_type<decltype(meta::type_at<idx>(members))>())
      //     return false;
      // }
      // return true;
    } else {
      return false;
    }
  }
};

// template<typename, typename = std::void_t<>>
// struct IsConvertibleToString : std::false_type
// {};

// template<typename T>
// struct IsConvertibleToString<
//   T,
//   std::void_t<decltype(std::declval<std::ostream&>().operator<<(std::declval<T>()))>>
//   : std::true_type
// {};

template<typename T>
struct IsConvertibleToString
{
  constexpr bool operator()() const
  {
    // typename decltype(std::declval<std::ostream>() << T{})::wrong x;
    return decltype(s_is_printable(T{}))::value;
  }

private:
  constexpr static const auto s_is_printable =
    boost::hana::is_valid([](auto&& x) -> decltype(std::declval<std::ostream>() << x) {});
};

template<typename T>
inline constexpr bool
is_flat_trivial_type()
{
  // TODO Not sure what this type constraint REALLY means. Is a flat type a type that contains only
  // primitive types? This would exclude std::string, which I want to allow. Does it instead mean
  // 'only members that can be converted to string'? This would allow some complex types, given that
  // they have operator<< for std::ostream. For now, I'm sticking to 'only members that have
  // operator<< for std::ostream'. For this, I have to explicitly check if the type T itself has
  // operator<<

  if constexpr (IsConvertibleToString<T>{}()) {
    return true;
  } else if constexpr (!std::is_aggregate_v<T>) {
    return false;
  } else {
    constexpr auto const members = refl::members_of<T>();
    return meta::all_of<IsConvertibleToString>(members);
  }
}

/**
 * IsNestedTrivialType works by first ensuring that the actual type passed
 * is an aggregate (the reasoning being that single values make poor records because they have no
 * names, whereas aggregates have members with names)
 *
 * Then, it recursively checks all members for either:
 *   a) IsConvertibleToString (to cover all basic members)
 *   b) IsNestedTrivialType (the recursive case where a member is itself an aggregate of types
 * convertible to string)
 *
 * This catches arbitrarily deeply nested types
 */

template<typename T>
struct IsNestedTrivialTypeImpl
{
  constexpr bool operator()() const
  {
    if constexpr (IsConvertibleToString<T>{}()) {
      return true;
    } else {
      if constexpr (!refl::is_reflectable<T>()) {
        // T is not a trivial type of any sorts because we can't provide reflection on it
        // One common case for this would be std::vector, which of course has private members,
        // non-trivial constructors etc.
        // Unless users define operator<< for std::vector<T> - in which case IsConvertibleToString
        // passes - any type which has a std::vector member is not trivial!
        return false;
      } else {
        constexpr auto const members = refl::members_of<T>();
        return meta::all_of<IsNestedTrivialTypeImpl>(members);
      }
    }
  }
};

template<typename T>
inline constexpr bool
is_nested_trivial_type()
{
  if constexpr (!std::is_aggregate_v<T>)
    return false;

  constexpr auto const members = refl::members_of<T>();
  return meta::all_of<IsNestedTrivialTypeImpl>(members);
}

}

/**
 * Is the given type one of the primitive types allowed in JSON? The primitive types are:
 * - std::string
 * - all integral types
 * - the bool type
 * - std::vector<U> where IsJSONPrimitiveType<U> == true
 * - any type U that is a trivial type and has only members for which IsJSONPrimitiveType holds
 * (corresponds to object type in JSON)
 * - std::optional<U> where IsJSONPrimitiveType<U> == true (corresponds to nullable types in JSON)
 */
template<typename T>
inline constexpr bool IsJSONPrimitiveType = detail::IsJSONPrimitiveType<T>{}();

/**
 * Is the given type convertible to std::string through std::ostream::operator<<?
 */
template<typename T>
inline constexpr bool IsConvertibleToString = detail::IsConvertibleToString<T>{}();

/**
 * Does the given type only consist of members that are trivial and convertible to std::string?
 */
template<typename T>
inline constexpr bool IsFlatTrivialType = detail::is_flat_trivial_type<T>();

/**
 * Does the given type consist of nested types that are all convertible to std::string?
 */
template<typename T>
inline constexpr bool IsNestedTrivialType = detail::is_nested_trivial_type<T>(); // TODO Implement

template<typename T>
inline constexpr bool IsPair = detail::IsPair<T>::value;

#pragma endregion

#pragma region SupportMatrix

/**
 * Which types of storage, writer and partitioner combine with which other types?
 */

namespace detail {

template<typename RecordStorage, typename RecordWriter>
struct StorageWriterMatch : std::false_type
{};

template<typename RecordStorage, typename RecordPartitioner>
struct StoragePartitionerMatch : std::false_type
{};

template<typename RecordWriter, typename RecordPartitioner>
struct WriterPartitionerMatch : std::false_type
{};

// Specializations for all types that match. This is a safety net for adding new types (even though
// there are more types that match than those that don't)

template<typename T>
struct StorageWriterMatch<VectorStorage<T>, BinaryWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<VectorStorage<T>, JSONWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<VectorStorage<T>, CSVWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<VectorStorage<T>, TextWriter> : std::true_type
{};

template<>
struct StorageWriterMatch<TupleStorage, JSONWriter> : std::true_type
{};
template<>
struct StorageWriterMatch<TupleStorage, TextWriter> : std::true_type
{};

template<typename T>
struct StorageWriterMatch<TreeStorage<T>, BinaryWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<TreeStorage<T>, JSONWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<TreeStorage<T>, TextWriter> : std::true_type
{}; // But probably not very useful...

template<typename T>
struct StorageWriterMatch<DictionaryVectorStorage<T>, BinaryWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<DictionaryVectorStorage<T>, JSONWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<DictionaryVectorStorage<T>, CSVWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<DictionaryVectorStorage<T>, TextWriter> : std::true_type
{};

template<>
struct StorageWriterMatch<DictionaryTupleStorage, JSONWriter> : std::true_type
{};
template<>
struct StorageWriterMatch<DictionaryTupleStorage, TextWriter> : std::true_type
{};

template<typename T>
struct StorageWriterMatch<DictionaryTreeStorage<T>, BinaryWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<DictionaryTreeStorage<T>, JSONWriter> : std::true_type
{};
template<typename T>
struct StorageWriterMatch<DictionaryTreeStorage<T>, TextWriter> : std::true_type
{}; // Again, probably not super useful

template<typename T>
struct StoragePartitionerMatch<VectorStorage<T>, SingleFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<VectorStorage<T>, ChunkedFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<VectorStorage<T>, UniqueFilePartitioner> : std::true_type
{};

template<>
struct StoragePartitionerMatch<TupleStorage, SingleFilePartitioner> : std::true_type
{};
template<>
struct StoragePartitionerMatch<TupleStorage, ChunkedFilePartitioner> : std::true_type
{};
template<>
struct StoragePartitionerMatch<TupleStorage, UniqueFilePartitioner> : std::true_type
{};

template<typename T>
struct StoragePartitionerMatch<TreeStorage<T>, SingleFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<TreeStorage<T>, ChunkedFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<TreeStorage<T>, UniqueFilePartitioner> : std::true_type
{};

template<typename T>
struct StoragePartitionerMatch<DictionaryVectorStorage<T>, SingleFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<DictionaryVectorStorage<T>, ChunkedFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<DictionaryVectorStorage<T>, UniqueFilePartitioner> : std::true_type
{};

template<>
struct StoragePartitionerMatch<DictionaryTupleStorage, SingleFilePartitioner> : std::true_type
{};
template<>
struct StoragePartitionerMatch<DictionaryTupleStorage, ChunkedFilePartitioner> : std::true_type
{};
template<>
struct StoragePartitionerMatch<DictionaryTupleStorage, UniqueFilePartitioner> : std::true_type
{};

template<typename T>
struct StoragePartitionerMatch<DictionaryTreeStorage<T>, SingleFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<DictionaryTreeStorage<T>, ChunkedFilePartitioner> : std::true_type
{};
template<typename T>
struct StoragePartitionerMatch<DictionaryTreeStorage<T>, UniqueFilePartitioner> : std::true_type
{};

template<>
struct WriterPartitionerMatch<BinaryWriter, SingleFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<BinaryWriter, ChunkedFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<BinaryWriter, UniqueFilePartitioner> : std::true_type
{};

template<>
struct WriterPartitionerMatch<CSVWriter, SingleFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<CSVWriter, ChunkedFilePartitioner> : std::true_type
{};
// UniqueFilePartitioner WOULD be compatible with CSV files, but then you end up with lots of CSV
// files with just one entry...

template<>
struct WriterPartitionerMatch<JSONWriter, SingleFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<JSONWriter, ChunkedFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<JSONWriter, UniqueFilePartitioner> : std::true_type
{};

template<>
struct WriterPartitionerMatch<TextWriter, SingleFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<TextWriter, ChunkedFilePartitioner> : std::true_type
{};
template<>
struct WriterPartitionerMatch<TextWriter, UniqueFilePartitioner> : std::true_type
{};

}

template<typename RecordStorage, typename RecordWriter>
inline constexpr bool StorageAndWriterMatch =
  detail::StorageWriterMatch<RecordStorage, RecordWriter>::value;

template<typename RecordStorage, typename RecordPartitioner>
inline constexpr bool StorageAndPartitionerMatch =
  detail::StoragePartitionerMatch<RecordStorage, RecordPartitioner>::value;

template<typename RecordWriter, typename RecordPartitioner>
inline constexpr bool WriterAndPartitionerMatch =
  detail::WriterPartitionerMatch<RecordWriter, RecordPartitioner>::value;

#pragma endregion

#pragma region Builder

/**
 * First-stage builder for Journal objects
 */
struct JournalBuilder
{
  friend struct JournalStore;

  /**
   * The Journal will store records of the fixed type T, where T is interpreted as a set of members
   * where each member is convertible to std::string. In contrast to 'with_nested_type', a Journal
   * that stores flat types will not expand members into multiple entries upon writing.
   *
   * As an example, consider the following types:
   *
   * struct TypeA {
   *   std::string str1;
   *   std::string str2;
   * };
   *
   * struct TypeB {
   *   TypeA nested;
   *   std::string str3;
   * };
   *
   * TypeB test{
   *  {"a", "b"},
   *  "c"
   * };
   *
   * Writing the object 'test' as a record and printing it as a CSV file, this is the result
   * (assuming that TypeA is convertible to std::string):
   *
   * with_flat_type:
   *    "nested;str3"
   *    "ab;c"
   *
   * with_nested_type:
   *    "str1;str2;str3"
   *    "a;b;c"
   *
   */
  template<typename T>
  JournalBuilderWithType<VectorStorage<T>> with_flat_type();
  /**
   * The Journal will store arbitrary types. The allowed types will depend on the type of
   * RecordWriter of the Journal
   */
  JournalBuilderWithType<TupleStorage> with_dynamic_type();
  /**
   * The Journal will store records of the fixed type T, where T can be a flat or a nested type. See
   * the documentation of 'with_flat_type' for an example that illustrates the difference between
   * the two.
   */
  template<typename T>
  JournalBuilderWithType<TreeStorage<T>> with_nested_type();

  template<typename T>
  JournalBuilderWithType<DictionaryVectorStorage<T>> with_indexed_fixed_type();
  JournalBuilderWithType<DictionaryTupleStorage> with_indexed_dynamic_type();
  template<typename T>
  JournalBuilderWithType<DictionaryTreeStorage<T>> with_indexed_nested_type();

private:
  JournalBuilder(JournalStore* store, std::string journal_name)
    : _store(store)
    , _journal_name(std::move(journal_name))
  {}

  JournalStore* _store;
  std::string _journal_name;
};

/**
 * Second-stage builder for Journal objects. Already knows the RecordStorage type
 */
template<typename RecordStorage>
struct JournalBuilderWithType
{
  friend struct JournalBuilder;

  /**
   * Stores Records in binary form
   */
  JournalBuilderWithTypeAndFormat<RecordStorage, BinaryWriter> as_binary(
    const std::experimental::filesystem::path& base_path);
  /**
   * Store Records as CSV file(s)
   */
  JournalBuilderWithTypeAndFormat<RecordStorage, CSVWriter> as_csv(
    const std::experimental::filesystem::path& base_path);
  /**
   * Store Records in JSON notation
   */
  JournalBuilderWithTypeAndFormat<RecordStorage, JSONWriter> as_json(
    const std::experimental::filesystem::path& base_path);
  /**
   * Store records in text form
   */
  JournalBuilderWithTypeAndFormat<RecordStorage, TextWriter> as_text(
    const std::experimental::filesystem::path& base_path);

private:
  JournalBuilderWithType(JournalStore* store,
                         std::string journal_name,
                         RecordStorage record_storage)
    : _store(store)
    , _journal_name(std::move(journal_name))
    , _record_storage(std::move(record_storage))
  {}

  JournalStore* _store;
  std::string _journal_name;
  RecordStorage _record_storage;
};

template<typename RecordStorage, typename RecordWriter>
struct JournalBuilderWithTypeAndFormat
{
  friend struct JournalBuilderWithType<RecordStorage>;

  JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage, RecordWriter, SingleFilePartitioner>
  into_single_file();
  JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage, RecordWriter, ChunkedFilePartitioner>
  into_chunked_files(size_t chunk_size);
  JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage, RecordWriter, UniqueFilePartitioner>
  into_unique_files();

private:
  JournalBuilderWithTypeAndFormat(JournalStore* store,
                                  std::string journal_name,
                                  RecordStorage record_storage,
                                  RecordWriter record_writer)
    : _store(store)
    , _journal_name(std::move(journal_name))
    , _record_storage(std::move(record_storage))
    , _record_writer(std::move(record_writer))
  {}

  JournalStore* _store;
  std::string _journal_name;
  RecordStorage _record_storage;
  RecordWriter _record_writer;
};

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
struct JournalBuilderWithTypeAndFormatAndPartitioner
{
  friend struct JournalBuilderWithTypeAndFormat<RecordStorage, RecordWriter>;

  Journal<RecordStorage, RecordWriter, RecordPartitioner>* build();

private:
  JournalBuilderWithTypeAndFormatAndPartitioner(JournalStore* store,
                                                std::string journal_name,
                                                RecordStorage record_storage,
                                                RecordWriter record_writer,
                                                RecordPartitioner record_partitioner)
    : _store(store)
    , _journal_name(std::move(journal_name))
    , _record_storage(std::move(record_storage))
    , _record_writer(std::move(record_writer))
    , _record_partitioner(std::move(record_partitioner))
  {}

  JournalStore* _store;
  std::string _journal_name;
  RecordStorage _record_storage;
  RecordWriter _record_writer;
  RecordPartitioner _record_partitioner;
};

// Impl
template<typename T>
inline JournalBuilderWithType<VectorStorage<T>>
JournalBuilder::with_flat_type()
{
  static_assert(IsFlatTrivialType<T>,
                "Journal with flat type is only valid for types T for which IsFlatTrivialType<T> "
                "equals true!");
  return { _store, std::move(_journal_name), {} };
}

inline JournalBuilderWithType<TupleStorage>
JournalBuilder::with_dynamic_type()
{
  return { _store, std::move(_journal_name), {} };
}

template<typename T>
inline JournalBuilderWithType<TreeStorage<T>>
JournalBuilder::with_nested_type()
{
  // TODO Technically, IsFlatTrivialType<T> || IsNestedTrivialType<T>, but this way forces users
  // to select the exact storage type that makes sense
  static_assert(IsNestedTrivialType<T>,
                "Journal with nested type is only valid for types T for which "
                "IsNestedTrivialType<T> equals true!");
  return { _store, std::move(_journal_name), {} };
}

template<typename T>
inline JournalBuilderWithType<DictionaryVectorStorage<T>>
JournalBuilder::with_indexed_fixed_type()
{
  static_assert(
    IsFlatTrivialType<T>,
    "Journal with indexed flat type is only valid for types T for which IsFlatTrivialType<T> "
    "equals true!");
  return { _store, std::move(_journal_name), {} };
}

inline JournalBuilderWithType<DictionaryTupleStorage>
JournalBuilder::with_indexed_dynamic_type()
{
  return { _store, std::move(_journal_name), {} };
}

template<typename T>
inline JournalBuilderWithType<DictionaryTreeStorage<T>>
JournalBuilder::with_indexed_nested_type()
{
  static_assert(IsNestedTrivialType<T>,
                "Journal with indexed nested type is only valid for types T for which "
                "IsNestedTrivialType<T> equals true!");
  return { _store, std::move(_journal_name), {} };
}

template<typename RecordStorage>
inline JournalBuilderWithTypeAndFormat<RecordStorage, BinaryWriter>
JournalBuilderWithType<RecordStorage>::as_binary(
  const std::experimental::filesystem::path& base_path)
{
  static_assert(StorageAndWriterMatch<RecordStorage, BinaryWriter>,
                "Can't use BinaryWriter with this RecordStorage type!");
  return {
    _store, std::move(_journal_name), std::move(_record_storage), BinaryWriter{ base_path }
  };
}

template<typename RecordStorage>
inline JournalBuilderWithTypeAndFormat<RecordStorage, CSVWriter>
JournalBuilderWithType<RecordStorage>::as_csv(const std::experimental::filesystem::path& base_path)
{
  static_assert(StorageAndWriterMatch<RecordStorage, CSVWriter>,
                "Can't use CSVWriter with this RecordStorage type!");
  // CSVWriter constructor needs the type of Records that it will write, so we have to create the
  // CSVWriter with this indirection
  using RecordType = typename RecordStorage::RecordType;
  return { _store,
           std::move(_journal_name),
           std::move(_record_storage),
           CSVWriter::make_writer_for_type<RecordType>(base_path) };
}

template<typename RecordStorage>
inline JournalBuilderWithTypeAndFormat<RecordStorage, JSONWriter>
JournalBuilderWithType<RecordStorage>::as_json(const std::experimental::filesystem::path& base_path)
{
  static_assert(StorageAndWriterMatch<RecordStorage, JSONWriter>,
                "Can't use JSONWriter with this RecordStorage type!");
  return { _store, std::move(_journal_name), std::move(_record_storage), JSONWriter{ base_path } };
}

template<typename RecordStorage>
inline JournalBuilderWithTypeAndFormat<RecordStorage, TextWriter>
JournalBuilderWithType<RecordStorage>::as_text(const std::experimental::filesystem::path& base_path)
{
  static_assert(StorageAndWriterMatch<RecordStorage, TextWriter>,
                "Can't use TextWriter with this RecordStorage type!");
  using RecordType = typename RecordStorage::RecordType;
  return { _store,
           std::move(_journal_name),
           std::move(_record_storage),
           TextWriter::make_writer_for_type<RecordType>(base_path) };
}

template<typename RecordStorage, typename RecordWriter>
inline JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage,
                                                     RecordWriter,
                                                     SingleFilePartitioner>
JournalBuilderWithTypeAndFormat<RecordStorage, RecordWriter>::into_single_file()
{
  static_assert(StorageAndPartitionerMatch<RecordStorage, SingleFilePartitioner>,
                "Can't use SingleFilePartitioner with this RecordStorage type!");
  static_assert(WriterAndPartitionerMatch<RecordWriter, SingleFilePartitioner>,
                "Can't use SingleFilePartitioner with this RecordWriter type!");
  return { _store,
           std::move(_journal_name),
           std::move(_record_storage),
           std::move(_record_writer),
           SingleFilePartitioner{} };
}

template<typename RecordStorage, typename RecordWriter>
inline JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage,
                                                     RecordWriter,
                                                     ChunkedFilePartitioner>
JournalBuilderWithTypeAndFormat<RecordStorage, RecordWriter>::into_chunked_files(size_t chunk_size)
{
  static_assert(StorageAndPartitionerMatch<RecordStorage, ChunkedFilePartitioner>,
                "Can't use ChunkedFilePartitioner with this RecordStorage type!");
  static_assert(WriterAndPartitionerMatch<RecordWriter, ChunkedFilePartitioner>,
                "Can't use ChunkedFilePartitioner with this RecordWriter type!");
  return { _store,
           std::move(_journal_name),
           std::move(_record_storage),
           std::move(_record_writer),
           ChunkedFilePartitioner{ chunk_size } };
}

template<typename RecordStorage, typename RecordWriter>
inline JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage,
                                                     RecordWriter,
                                                     UniqueFilePartitioner>
JournalBuilderWithTypeAndFormat<RecordStorage, RecordWriter>::into_unique_files()
{
  static_assert(StorageAndPartitionerMatch<RecordStorage, UniqueFilePartitioner>,
                "Can't use UniqueFilePartitioner with this RecordStorage type!");
  static_assert(WriterAndPartitionerMatch<RecordWriter, UniqueFilePartitioner>,
                "Can't use UniqueFilePartitioner with this RecordWriter type!");
  return { _store,
           std::move(_journal_name),
           std::move(_record_storage),
           std::move(_record_writer),
           UniqueFilePartitioner{} };
}

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
inline Journal<RecordStorage, RecordWriter, RecordPartitioner>*
JournalBuilderWithTypeAndFormatAndPartitioner<RecordStorage, RecordWriter, RecordPartitioner>::
  build()
{
  return _store->add_journal(Journal{ std::move(_journal_name),
                                      std::move(_record_storage),
                                      std::move(_record_writer),
                                      std::move(_record_partitioner) });
}

#pragma endregion

#pragma region JournalImpl

template<typename T>
void
JournalBase::add_record_untyped(T&& record)
{
  using NakedType = std::decay_t<T>;
  const auto type_info = refl::Describe<NakedType>::get();
  do_add_record(type_info, { std::forward<T>(record) });
}

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
inline Journal<RecordStorage, RecordWriter, RecordPartitioner>::Journal(
  std::string name,
  RecordStorage storage,
  RecordWriter writer,
  RecordPartitioner partitioner)
  : _name(std::move(name))
  , _storage(std::move(storage))
  , _writer(std::move(writer))
  , _partitioner(std::move(partitioner))
{}

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
inline Journal<RecordStorage, RecordWriter, RecordPartitioner>::~Journal()
{
  flush_records();
}

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
template<typename T>
inline void
Journal<RecordStorage, RecordWriter, RecordPartitioner>::add_record(T&& record)
{
  static_assert(std::is_convertible_v<T, typename RecordStorage::RecordType>,
                "Journal::add_record: Invalid Record type!");

  _storage.add_record(std::forward<T>(record));
  if (!_partitioner.requires_flush(_storage.records().size()))
    return;

  flush_records();
}

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
inline void
Journal<RecordStorage, RecordWriter, RecordPartitioner>::flush_records()
{
  _partitioner.partition_and_write_records(util::range(_storage.records()), _writer, _name);
  _storage.clear();
}

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
void
Journal<RecordStorage, RecordWriter, RecordPartitioner>::do_add_record(
  refl::TypeDescriptor const* type,
  std::any record)
{
  using AcceptableRecordType = typename RecordStorage::RecordType;
  const auto acceptable_type = refl::Describe<AcceptableRecordType>::get();
  if (type != acceptable_type) {
    throw std::invalid_argument{
      (boost::format(
         "Journal::do_add_record: Expected Record of type %1% but got type %2% instead") %
       acceptable_type->name % type->name)
        .str()
    };
  }

  auto typed_record = std::any_cast<AcceptableRecordType>(std::move(record));
  add_record(std::move(typed_record));
}

#pragma endregion

#pragma region JournalStoreImpl

template<typename RecordStorage, typename RecordWriter, typename RecordPartitioner>
Journal<RecordStorage, RecordWriter, RecordPartitioner>*
JournalStore::add_journal(Journal<RecordStorage, RecordWriter, RecordPartitioner> journal)
{
  const auto journal_name = journal.name();
  const auto iter = _journals.insert_or_assign(
    journal_name,
    std::make_unique<Journal<RecordStorage, RecordWriter, RecordPartitioner>>(std::move(journal)));
  return static_cast<Journal<RecordStorage, RecordWriter, RecordPartitioner>*>(
    iter.first->second.get());
}

#pragma endregion

#pragma region StorageImpl

template<typename T>
void
VectorStorage<T>::add_record(const T& record)
{
  _records.push_back(record);
}

template<typename T>
void
VectorStorage<T>::add_record(T&& record)
{
  _records.push_back(std::move(record));
}

template<typename T>
void
TupleStorage::add_record(T&& record)
{
  _records.emplace_back(std::forward<T>(record));
}

template<typename T>
void
TreeStorage<T>::add_record(const T& record)
{
  _records.push_back(record);
}

template<typename T>
void
TreeStorage<T>::add_record(T&& record)
{
  _records.push_back(std::move(record));
}

template<typename T>
void
DictionaryVectorStorage<T>::add_record(std::string key, const T& record)
{
  _records.insert_or_assign(std::move(key), record);
}

template<typename T>
void
DictionaryVectorStorage<T>::add_record(std::string key, T&& record)
{
  _records.insert_or_assign(std::move(key), std::move(record));
}

template<typename T>
void
DictionaryTupleStorage::add_record(std::string key, T&& record)
{
  _records.insert_or_assign(std::move(key), { std::forward<T>(record) });
}

template<typename T>
void
DictionaryTreeStorage<T>::add_record(std::string key, const T& record)
{
  _records.insert_or_assign(std::move(key), record);
}

template<typename T>
void
DictionaryTreeStorage<T>::add_record(std::string key, T&& record)
{
  _records.insert_or_assign(std::move(key), std::move(record));
}

#pragma endregion

#pragma region WriterImpl

template<typename RecordIterator>
void
BinaryWriter::write_records(util::Range<RecordIterator> records, const std::string& file_name)
{}

template<typename T>
logging::CSVWriter
logging::CSVWriter::make_writer_for_type(const std::experimental::filesystem::path& base_path)
{
  return CSVWriter{ base_path, reinterpret_cast<T*>(0) };
}

template<typename T>
logging::CSVWriter::CSVWriter(const std::experimental::filesystem::path& base_path, T*)
  : _base_path(base_path)
{
  if (!std::experimental::filesystem::exists(_base_path)) {
    if (!std::experimental::filesystem::create_directories(_base_path)) {
      throw std::runtime_error{
        (boost::format("CSVWriter::CSVWriter: Could not create directory for base path %1%") %
         _base_path)
          .str()
      };
    }
  }
  // TODO We have to initialize the two functions for writing header and records here, depending on
  // the type T. This type is defined by the JournalBuilderWithType, which knows what kind of
  // object(s) will be passed to the Writer. There, we have to distinguish between several cases:
  //
  //  a) It is a fixed type that is not std::any -> In this case we reflect upon the type, get the
  //  names of it members and use this for writing the header. The Record itself is written by
  //  accessing all of the members in order and passing them to std::ostream using operator<<
  //
  //  b) It is any other type. In the case of CSVWriter, only fixed types are supported, so this is
  //  easy (we static_assert). For other writers however, we might have to do more here. See notes
  //  in Evernote on this

  const auto type_info = refl::Describe<T>::get();
  if (!type_info) {
    // TODO Some function 'refl::can_reflect<T>' that returns true or the reason why T can't be
    // reflected upon
    throw std::runtime_error{ "Could not reflect type T" };
  }
  const auto type_info_for_struct = dynamic_cast<const refl::TypeDescriptorStruct*>(type_info);
  if (!type_info_for_struct) {
    // T is standard type (like int, float, or even std::string)
    // Standard types are print to CSV with the header being the type name
    _header_writer = [type_info](std::ostream& ostream) { ostream << type_info->name; };

    _record_writer = [type_info](const std::any& record, std::ostream& ostream) {
      const auto& typed_record = std::any_cast<const T&>(record);
      type_info->print_value(&typed_record, ostream);
      ostream << "\n";
    };

  } else {
    // T is an aggregate type
    _header_writer = [type_info_for_struct](std::ostream& ostream) {
      if (!type_info_for_struct->members.size())
        return;

      const auto idx_of_last_member = type_info_for_struct->members.size() - 1;
      for (size_t member_idx = 0; member_idx < type_info_for_struct->members.size(); ++member_idx) {
        const auto& member_info = type_info_for_struct->members[member_idx];
        ostream << member_info.name;
        if (member_idx != idx_of_last_member) {
          ostream << ";";
        }
      }
      ostream << "\n";
    };

    _record_writer = [type_info_for_struct](const std::any& record, std::ostream& ostream) {
      if (!type_info_for_struct->members.size())
        return;

      const auto& typed_record = std::any_cast<const T&>(record);

      const auto idx_of_last_member = type_info_for_struct->members.size() - 1;
      for (size_t member_idx = 0; member_idx < type_info_for_struct->members.size(); ++member_idx) {
        const auto& member_info = type_info_for_struct->members[member_idx];
        const auto member_ptr = reinterpret_cast<const char*>(&typed_record) + member_info.offset;
        member_info.type->print_value(member_ptr, ostream);
        if (member_idx != idx_of_last_member) {
          ostream << ";";
        }
      }
      ostream << "\n";
    };
  }
}

template<typename RecordIterator>
void
CSVWriter::write_records(util::Range<RecordIterator> records, const std::string& file_name)
{
  // We assume that RecordIterator dereferences to std::any
  const auto full_path = _base_path / (file_name + ".csv");
  std::ofstream fs{ full_path };
  if (!fs.is_open()) {
    throw std::runtime_error{
      (boost::format("CSVWriter::write_records: Could not open file %1% for writing") % full_path)
        .str()
    };
  }

  _header_writer(fs);
  for (auto& record : records) {
    _record_writer(record, fs);
  }
}

template<typename RecordIterator>
void
JSONWriter::write_records(util::Range<RecordIterator> records, const std::string& file_name)
{}

template<typename T>
logging::TextWriter
logging::TextWriter::make_writer_for_type(const std::experimental::filesystem::path& base_path)
{
  return TextWriter{ base_path, reinterpret_cast<T*>(0) };
}

template<typename T>
logging::TextWriter::TextWriter(const std::experimental::filesystem::path& base_path, T*)
  : _base_path(base_path)
{
  if (!std::experimental::filesystem::exists(_base_path)) {
    if (!std::experimental::filesystem::create_directories(_base_path)) {
      throw std::runtime_error{
        (boost::format("TextWriter::TextWriter: Could not create directory for base path %1%") %
         _base_path)
          .str()
      };
    }
  }

  const auto type_info = refl::Describe<T>::get();
  if (!type_info) {
    throw std::runtime_error{ "TextWriter::TextWriter: Could not reflect type T" };
  }

  // Several cases:
  // a) T is flat trivial type
  // b) T is std::any
  // c) T is nested trivial type
  // d) T is std::pair<std::string, U> with U being one of the above

  if constexpr (std::is_same_v<T, std::any>) {
    static_assert(AlwaysFalse<T>::value, "TextWriter with std::any not supported at the moment");
  } else if constexpr (std::is_same_v<T, std::pair<std::string, std::any>>) {
    static_assert(AlwaysFalse<T>::value, "TextWriter with std::any not supported at the moment");
  } else if constexpr (IsPair<T>) {
    // Case d)
    _record_writer = [type_info](const std::any& record, std::ostream& ostream) {
      const auto& typed_record = std::any_cast<const T&>(record);
      using SecondType = std::remove_cv_t<decltype(typed_record.second)>;
      // TODO Check for SecondType being a standard type. Maybe there is a more general scheme to
      // prevent this type checking here and instead use the TypeDescriptor class to handle value
      // extraction completely?
      const auto second_type_info =
        dynamic_cast<const refl::TypeDescriptorStruct*>(refl::Describe<SecondType>::get());
      if (!second_type_info) {
        throw std::runtime_error{ "TextWriter::_record_writer: Could not reflect type SecondType" };
      }

      ostream << typed_record.first << ": ";

      for (auto& member : second_type_info->members) {
        const auto member_ptr = reinterpret_cast<const char*>(&typed_record.second) + member.offset;
        member.type->print_value(member_ptr, ostream);
        ostream << " ";
      }

      ostream << "\n";
    };
  } else {
    const auto type_info_for_struct = dynamic_cast<const refl::TypeDescriptorStruct*>(type_info);
    if (type_info_for_struct) {
      // Case a) / c) with aggregate type
      _record_writer = [type_info_for_struct](const std::any& record, std::ostream& ostream) {
        const auto& typed_record = std::any_cast<const T&>(record);
        for (auto& member : type_info_for_struct->members) {
          const auto member_ptr = reinterpret_cast<const char*>(&typed_record) + member.offset;
          member.type->print_value(member_ptr, ostream);
          ostream << " ";
        }
        ostream << "\n";
      };
    } else {
      // Case a) / c) with primitive type
      _record_writer = [](const std::any& record, std::ostream& ostream) {
        const auto& typed_record = std::any_cast<const T&>(record);
        ostream << typed_record << "\n";
      };
    }
  }
}

template<typename RecordIterator>
void
TextWriter::write_records(util::Range<RecordIterator> records, const std::string& file_name)
{
  const auto full_path = _base_path / (file_name + ".txt");
  std::ofstream fs{ full_path };
  if (!fs.is_open()) {
    throw std::runtime_error{
      (boost::format("TextWriter::write_records: Could not open file %1% for writing") % full_path)
        .str()
    };
  }

  for (auto& record : records) {
    _record_writer(record, fs);
  }
}

#pragma endregion

#pragma region PartitonerImpl

template<typename RecordIterator, typename RecordWriter>
void
SingleFilePartitioner::partition_and_write_records(util::Range<RecordIterator> records,
                                                   RecordWriter& writer,
                                                   const std::string& journal_name)
{
  writer.write_records(records, journal_name);
}

template<typename RecordIterator, typename RecordWriter>
void
ChunkedFilePartitioner::partition_and_write_records(util::Range<RecordIterator> records,
                                                    RecordWriter& writer,
                                                    const std::string& journal_name)
{
  throw std::runtime_error{ "not implemented" };
}

template<typename RecordIterator, typename RecordWriter>
void
UniqueFilePartitioner::partition_and_write_records(util::Range<RecordIterator> records,
                                                   RecordWriter& writer,
                                                   const std::string& journal_name)
{
  for (auto begin = std::begin(records); begin != std::end(records); ++begin) {
    util::Range<RecordIterator> single_record_range{ begin, std::next(begin) };
    const auto file_name = (boost::format("%1%_%2%") % journal_name % _next_file_index++).str();
    writer.write_records(single_record_range, file_name);
  }
}

#pragma endregion

}