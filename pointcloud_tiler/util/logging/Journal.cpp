#include "logging/Journal.h"

#pragma region JournalBase

logging::JournalBase::~JournalBase() {}

#pragma endregion

#pragma region JournalStore

logging::JournalStore&
logging::JournalStore::global()
{
  static JournalStore s_instance;
  return s_instance;
}

logging::JournalBase*
logging::JournalStore::get_journal(const std::string& journal_name)
{
  const auto iter_to_journal = _journals.find(journal_name);
  if (iter_to_journal == std::end(_journals))
    return nullptr;

  return iter_to_journal->second.get();
}

logging::JournalBuilder
logging::JournalStore::new_journal(const std::string& journal_name)
{
  return { this, journal_name };
}

#pragma endregion

#pragma region Writers

logging::BinaryWriter::BinaryWriter(const std::experimental::filesystem::path& base_path)
  : _base_path(base_path)
{}

logging::JSONWriter::JSONWriter(const std::experimental::filesystem::path& base_path)
  : _base_path(base_path)
{}

#pragma endregion

#pragma region Partitioners

bool
logging::SingleFilePartitioner::requires_flush(size_t number_of_records) const
{
  return false;
}

bool
logging::ChunkedFilePartitioner::requires_flush(size_t number_of_records) const
{
  return number_of_records >= _chunk_size;
}

logging::UniqueFilePartitioner::UniqueFilePartitioner() : _next_file_index(0) {}

bool
logging::UniqueFilePartitioner::requires_flush(size_t number_of_records) const
{
  return number_of_records > 0;
}

#pragma endregion