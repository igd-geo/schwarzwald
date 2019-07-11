#include "ThroughputCounter.hpp"

#include <numeric>

void Potree::ThroughputCounter::push_entry(size_t count) {
    const auto now = std::chrono::high_resolution_clock::now();
    _entries.push_back({count, now});
    if(_entries.size() > MAX_SAMPLES) {
        _entries.pop_front();
    }
}

double Potree::ThroughputCounter::get_throughput_per_second() const {
    if(!_entries.size()) return 0;

    const auto t_start = _entries.front().timepoint;
    const auto t_end = _entries.back().timepoint;
    const auto timespan_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start);

    const auto total_count = std::accumulate(_entries.begin(), _entries.end(), size_t{0}, [](auto accum, const auto& entry) {
        return accum + entry.count;
    });

    return 1e9 * (total_count / static_cast<double>(timespan_ns.count()));
}