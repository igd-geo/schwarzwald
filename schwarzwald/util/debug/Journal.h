#pragma once

#include <mutex>
#include <string>
#include <vector>

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

namespace debug {

struct Journal
{
  static Journal& instance();

#ifdef _WIN32
  void set_root_folder(const std::filesystem::path& root_folder);
  const std::filesystem::path& get_root_folder() const;
#else
  void set_root_folder(const std::experimental::filesystem::path& root_folder);
  const std::experimental::filesystem::path& get_root_folder() const;
#endif

  void add_entry(std::string entry);

  void enable(bool enable);
  bool is_enabled() const { return _enabled; }

private:
  Journal();
  ~Journal();

  void dump_and_clear_journal();

  std::vector<std::string> _entries;
  std::mutex _lock;
#ifdef _WIN32
  std::filesystem::path _root_folder;
#else
  std::experimental::filesystem::path _root_folder;
#endif
  uint32_t _sub_journal_number;
  bool _enabled;
};

} // namespace debug
