#pragma once

#include <mutex>
#include <taskflow/taskflow.hpp>
#include <vector>

/**
 * This is basically tf::ExecutorObserver, but instead of storing tf::TaskView objects
 * it stores task names internally.
 */
class ExecutorObserver : public tf::ExecutorObserverInterface
{

  friend class tf::Executor;

  // data structure to record each task execution
  struct Execution
  {

    std::string task_name;

    std::chrono::time_point<std::chrono::steady_clock> beg;
    std::chrono::time_point<std::chrono::steady_clock> end;

    Execution(tf::TaskView tv, std::chrono::time_point<std::chrono::steady_clock> b)
      : task_name{ tv.name() }
      , beg{ b }
    {}

    Execution(tf::TaskView tv,
              std::chrono::time_point<std::chrono::steady_clock> b,
              std::chrono::time_point<std::chrono::steady_clock> e)
      : task_name{ tv.name() }
      , beg{ b }
      , end{ e }
    {}
  };

  // data structure to store the entire execution timeline
  struct Timeline
  {
    std::chrono::time_point<std::chrono::steady_clock> origin;
    std::vector<std::vector<Execution>> executions;
  };

public:
  /**
  @brief dump the timelines in JSON format to an ostream
  @param ostream the target std::ostream to dump
  */
  inline void dump(std::ostream& ostream) const;

  /**
  @brief dump the timelines in JSON to a std::string
  @return a JSON string
  */
  inline std::string dump() const;

  /**
  @brief clear the timeline data
  */
  inline void clear();

  /**
  @brief get the number of total tasks in the observer
  @return number of total tasks
  */
  inline size_t num_tasks() const;

private:
  inline void set_up(unsigned num_workers) override final;
  inline void on_entry(unsigned worker_id, tf::TaskView task_view) override final;
  inline void on_exit(unsigned worker_id, tf::TaskView task_view) override final;

  Timeline _timeline;
};

// Procedure: set_up
inline void
ExecutorObserver::set_up(unsigned num_workers)
{

  _timeline.executions.resize(num_workers);

  for (unsigned w = 0; w < num_workers; ++w) {
    _timeline.executions[w].reserve(1024);
  }

  _timeline.origin = std::chrono::steady_clock::now();
}

// Procedure: on_entry
inline void
ExecutorObserver::on_entry(unsigned w, tf::TaskView tv)
{
  _timeline.executions[w].emplace_back(tv, std::chrono::steady_clock::now());
}

// Procedure: on_exit
inline void
ExecutorObserver::on_exit(unsigned w, tf::TaskView tv)
{
  assert(_timeline.executions[w].size() > 0);
  _timeline.executions[w].back().end = std::chrono::steady_clock::now();
}

// Function: clear
inline void
ExecutorObserver::clear()
{
  for (size_t w = 0; w < _timeline.executions.size(); ++w) {
    _timeline.executions[w].clear();
  }
}

// Procedure: dump
inline void
ExecutorObserver::dump(std::ostream& os) const
{

  os << '[';

  for (size_t w = 0; w < _timeline.executions.size(); w++) {

    if (w != 0 && _timeline.executions[w].size() > 0 && _timeline.executions[w - 1].size() > 0) {
      os << ',';
    }

    for (size_t i = 0; i < _timeline.executions[w].size(); i++) {

      os << '{' << "\"cat\":\"ExecutorObserver\","
         << "\"name\":\"" << _timeline.executions[w][i].task_name << "\","
         << "\"ph\":\"X\","
         << "\"pid\":1,"
         << "\"tid\":" << w << ',' << "\"ts\":"
         << std::chrono::duration_cast<std::chrono::microseconds>(_timeline.executions[w][i].beg -
                                                                  _timeline.origin)
              .count()
         << ',' << "\"dur\":"
         << std::chrono::duration_cast<std::chrono::microseconds>(_timeline.executions[w][i].end -
                                                                  _timeline.executions[w][i].beg)
              .count();

      if (i != _timeline.executions[w].size() - 1) {
        os << "},";
      } else {
        os << '}';
      }
    }
  }
  os << "]\n";
}

// Function: dump
inline std::string
ExecutorObserver::dump() const
{
  std::ostringstream oss;
  dump(oss);
  return oss.str();
}

// Function: num_tasks
inline size_t
ExecutorObserver::num_tasks() const
{
  return std::accumulate(_timeline.executions.begin(),
                         _timeline.executions.end(),
                         size_t{ 0 },
                         [](size_t sum, const auto& exe) { return sum + exe.size(); });
}