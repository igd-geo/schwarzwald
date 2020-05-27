#include "terminal/stdout_helper.h"

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <numeric>
#include <vector>

#if _WIN32
#include <io.h>
#define ISATTY _isatty
#define FILENO _fileno
#else
#include <unistd.h>
#define ISATTY isatty
#define FILENO fileno
#endif

constexpr static auto ANSI_ClearLine = "\u001b[2K";

static bool
is_newline(char c)
{
  return c == '\n';
}

std::mutex&
util::print_lock()
{
  static std::mutex s_lock;
  return s_lock;
}

void
util::write_log(const std::string& log)
{
  if (!terminal_is_tty()) {
    std::cout << log;
    return;
  }

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

bool
util::terminal_is_tty()
{
  return false;
  // static bool s_is_tty = ISATTY(FILENO(stdout)) != 0;
  // return s_is_tty;
}