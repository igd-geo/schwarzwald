#include "debug/Journal.h"

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/format.hpp>

namespace fs = std::experimental::filesystem;

constexpr static size_t MaxJournalEntries = 10'000'000;

debug::Journal&
debug::Journal::instance()
{
  static debug::Journal s_instance;
  return s_instance;
}

debug::Journal::Journal()
  : _root_folder(".")
  , _sub_journal_number(0)
  , _enabled(false)
{}

debug::Journal::~Journal()
{
  dump_and_clear_journal();
}

void
debug::Journal::enable(bool enable)
{
  _enabled = enable;
}

void
debug::Journal::set_root_folder(const std::experimental::filesystem::path& root_folder)
{
  _root_folder = root_folder;

  if (!fs::exists(_root_folder)) {
    if (!fs::create_directories(_root_folder)) {
      throw std::runtime_error{ "Could not create output directory for journal" };
    }
  }
}

const std::experimental::filesystem::path&
debug::Journal::get_root_folder() const
{
  return _root_folder;
}

void
debug::Journal::add_entry(std::string entry)
{
  if (!_enabled)
    return;

  std::lock_guard guard{ _lock };
  _entries.push_back(std::move(entry));

  if (_entries.size() == MaxJournalEntries) {
    dump_and_clear_journal();
  }
}

void
debug::Journal::dump_and_clear_journal()
{
  if (_entries.empty())
    return;

  std::stringstream ss;
  ss << "journal_" << _sub_journal_number++ << ".log";
  const auto file_name = _root_folder / ss.str();

  std::ofstream fs{ file_name };
  if (!fs.is_open()) {
    throw std::runtime_error{
      (boost::format("Could not write journal: Could not open output file %1%") % file_name).str()
    };
  }

  for (auto& entry : _entries) {
    fs << entry << "\n";
  }

  _entries.clear();
}