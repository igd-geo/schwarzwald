#pragma once

#include <mutex>
#include <string>
#include <vector>

namespace debug {

struct Journal {
  static Journal &instance();

  void set_root_folder(std::string root_folder);
  void add_entry(std::string entry);

  void enable(bool enable);
  bool is_enabled() const { return _enabled; }

private:
  Journal();
  ~Journal();

  void dump_and_clear_journal();

  std::vector<std::string> _entries;
  std::mutex _lock;
  std::string _root_folder;
  uint32_t _sub_journal_number;
  bool _enabled;
};

} // namespace debug
