#include "io/stdout_helper.h"

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <numeric>
#include <vector>

constexpr static auto ANSI_ClearLine = "\u001b[2K";

static bool is_newline(char c) { return c == '\n'; }

std::mutex &util::print_lock() {
  static std::mutex s_lock;
  return s_lock;
}

void util::write_log(const std::string &log) {
  // Split by newlines and make sure that we clear each console line that we
  // print to. This ensures that we correctly overwrite the terminal UI
  // (progress bars etc.)

  std::vector<std::string> lines;
  boost::algorithm::split(lines, log, is_newline);

  print_lock().lock();

  for (size_t idx = 0; idx < lines.size(); ++idx) {
    std::cout << ANSI_ClearLine << lines[idx];
    if (idx < (lines.size() - 1)) {
      std::cout << '\n';
    }
  }

  print_lock().unlock();
}