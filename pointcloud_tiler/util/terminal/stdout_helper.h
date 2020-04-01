#pragma once

#include <mutex>
#include <string>

namespace util {

std::mutex &print_lock();

void write_log(const std::string &log);

/**
 * Returns true if stdout is a TTY
 */
bool terminal_is_tty();

} // namespace util