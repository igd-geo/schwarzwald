#include "debug/ThroughputCounter.h"

#include <numeric>

void
ThroughputCounter::push_entry(
  size_t count,
  std::chrono::high_resolution_clock::time_point timepoint)
{
  _entries.push_back({ count, timepoint });
  if (_entries.size() > MAX_SAMPLES) {
    _entries.pop_front();
  }
}

double
ThroughputCounter::get_throughput_per_second() const
{
  if (!_entries.size())
    return 0;

  const auto t_start = _entries.front().timepoint;
  const auto t_end = _entries.back().timepoint;
  const auto timespan_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start);

  const auto total_count = std::accumulate(
    _entries.begin(),
    _entries.end(),
    size_t{ 0 },
    [](auto accum, const auto& entry) { return accum + entry.count; });

  return 1e9 * (total_count / static_cast<double>(timespan_ns.count()));
}

ThroughputSampler::ThroughputSampler(size_t max_samples)
  : _max_samples(max_samples)
{}

void
ThroughputSampler::push_entry(size_t count, std::chrono::nanoseconds duration)
{
  _entries.push_back({ count, duration });
  if (_entries.size() > _max_samples) {
    _entries.pop_front();
  }
}

double
ThroughputSampler::get_throughput_per_second() const
{
  if (!_entries.size())
    return 0;

  using Accumulator = std::pair<size_t, std::chrono::nanoseconds>;

  const auto [total_count, total_time] =
    std::accumulate(_entries.begin(),
                    _entries.end(),
                    Accumulator{ 0, 0 },
                    [](auto accum, const auto& entry) {
                      return Accumulator{ accum.first + entry.count,
                                          accum.second + entry.duration };
                    });

  return 1e9 * (total_count / static_cast<double>(total_time.count()));
}