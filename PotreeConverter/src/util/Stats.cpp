#include "util/Stats.h"

#include "stuff.h"

#include <fstream>

void
dump_perf_stats(const PerformanceStats& stats, const std::string& work_dir)
{
  const auto stats_file_name = concat(work_dir, "/", "perf.stats");
  std::ofstream fs{ stats_file_name };
  if (!fs.is_open()) {
    std::cerr << "Could not write stats file " << stats_file_name << std::endl;
    return;
  }

  fs << "Prepare duration: " << (stats.prepare_duration.count() / 1000.f) << std::endl;
  fs << "Indexing duration: " << (stats.indexing_duration.count() / 1000.f) << std::endl;
  fs << "Postprocessing duration: " << (stats.postprocessing_duration.count() / 1000.f)
     << std::endl;
  fs << "Files written: " << stats.files_written << std::endl;
  fs << "Points processed: " << stats.points_processed << std::endl;

  fs.flush();
  fs.close();
}