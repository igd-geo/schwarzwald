#include "debug/Journal.h"

#include <fstream>
#include <iostream>
#include <sstream>

constexpr static size_t MaxJournalEntries = 10'000'000;

debug::Journal &debug::Journal::instance() {
  static debug::Journal s_instance;
  return s_instance;
}

debug::Journal::Journal() : _sub_journal_number(0), _enabled(false) {}

debug::Journal::~Journal() { dump_and_clear_journal(); }

void debug::Journal::enable(bool enable) { _enabled = enable; }

void debug::Journal::set_root_folder(std::string root_folder) {
  _root_folder = root_folder;
}

void debug::Journal::add_entry(std::string entry) {
  if (!_enabled)
    return;

  std::lock_guard guard{_lock};
  _entries.push_back(std::move(entry));

  if (_entries.size() == MaxJournalEntries) {
    dump_and_clear_journal();
  }
}

void debug::Journal::dump_and_clear_journal() {
  if (_entries.empty())
    return;

  std::stringstream ss;
  ss << _root_folder << "/journal_" << _sub_journal_number++ << ".log";
  const auto file_name = ss.str();

  std::ofstream fs{file_name};
  if (!fs.is_open()) {
    std::cerr << "Could not write journal..." << std::endl;
    return;
  }

  for (auto &entry : _entries) {
    fs << entry << "\n";
  }

  _entries.clear();
}