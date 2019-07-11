#pragma once

#include <list>
#include <chrono>

namespace Potree {

    struct ThroughputCounter {

        void push_entry(size_t count);
        double get_throughput_per_second() const;

    private:
        struct Entry {
            size_t count;
            std::chrono::high_resolution_clock::time_point timepoint;
        };

        constexpr static auto MAX_SAMPLES = 1024;
        std::list<Entry> _entries; 
    };

}