#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <experimental/filesystem>

namespace debug {

struct Journal
{
  static Journal& instance();

  void set_root_folder(const std::experimental::filesystem::path& root_folder);
  const std::experimental::filesystem::path& get_root_folder() const;

  void add_entry(std::string entry);

  void enable(bool enable);
  bool is_enabled() const { return _enabled; }

private:
  Journal();
  ~Journal();

  void dump_and_clear_journal();

  std::vector<std::string> _entries;
  std::mutex _lock;
  std::experimental::filesystem::path _root_folder;
  uint32_t _sub_journal_number;
  bool _enabled;
};

} // namespace debug
