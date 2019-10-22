#pragma once

#include <chrono>
#include <string>

struct PerformanceStats
{
  std::chrono::milliseconds prepare_duration;
  std::chrono::milliseconds indexing_duration;

  size_t files_written;
  size_t points_processed;
};

void
dump_perf_stats(const PerformanceStats& stats, const std::string& work_dir);