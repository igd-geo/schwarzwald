#pragma once

#include <chrono>
#include <list>

struct ThroughputCounter
{

  void push_entry(size_t count,
                  std::chrono::high_resolution_clock::time_point timepoint =
                    std::chrono::high_resolution_clock::now());
  double get_throughput_per_second() const;

private:
  struct Entry
  {
    size_t count;
    std::chrono::high_resolution_clock::time_point timepoint;
  };

  constexpr static auto MAX_SAMPLES = 1024;
  std::list<Entry> _entries;
};

/**
 * Estimates throughput based on a series of count/duration samples. This allows
 * sampling throughput over discontinuous time intervals, as opposed to the
 * ThroughputCounter, which counts continuous time intervals
 */
struct ThroughputSampler
{
  constexpr static auto DEFAULT_MAX_SAMPLES = 1024;

  explicit ThroughputSampler(size_t max_samples = DEFAULT_MAX_SAMPLES);

  void push_entry(size_t count, std::chrono::nanoseconds time);
  double get_throughput_per_second() const;

private:
  struct Entry
  {
    size_t count;
    std::chrono::nanoseconds duration;
  };

  std::list<Entry> _entries;
  size_t _max_samples;
};