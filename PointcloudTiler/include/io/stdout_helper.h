#pragma once

#include <mutex>
#include <string>

namespace util {

std::mutex &print_lock();

void write_log(const std::string &log);

} // namespace util