#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <variant>

template<typename T>
struct ProgressCounter
{
  explicit ProgressCounter(T max_progress)
    : _max_progress(max_progress)
    , _current_progress(T{ 0 })
  {}

  void increment_by(T increment) { _current_progress.fetch_add(increment); }

  T get_current_progress() const { return _current_progress.load(); }

  T get_max_progress() const { return _max_progress; }

private:
  T _max_progress;
  std::atomic<T> _current_progress;
};

using ProgressCounter_t = std::variant<ProgressCounter<size_t>, ProgressCounter<double>>;

/**
 * Helper class for tracking progress of various processes in a general manner
 */
struct ProgressReporter
{

  template<typename T>
  void register_progress_counter(const std::string& name, T max_progress)
  {
    std::lock_guard<std::mutex> lock{ _lock };
    _progress_counters[name] =
      std::make_unique<ProgressCounter_t>(std::in_place_type<ProgressCounter<T>>, max_progress);
  }

  template<typename T>
  void increment_progress(const std::string& name, T increment)
  {
    auto& counter = get_progress_counter<T>(name);
    counter.increment_by(increment);
  }

  template<typename T>
  T get_progress(const std::string& name)
  {
    auto& counter = get_progress_counter<T>(name);
    return counter.get_current_progress();
  }

  template<typename T>
  float get_progress_as_percentage(const std::string& name)
  {
    auto& counter = get_progress_counter<T>(name);
    return static_cast<float>(counter.get_current_progress()) /
           static_cast<float>(counter.get_max_progress());
  }

  template<typename T>
  T get_max_progress(const std::string& name)
  {
    auto& counter = get_progress_counter<T>(name);
    return counter.get_max_progress();
  }

  bool has_progress_counter(const std::string& name)
  {
    std::lock_guard<std::mutex> lock{ _lock };
    return _progress_counters.find(name) != _progress_counters.end();
  }

private:
  template<typename T>
  ProgressCounter<T>& get_progress_counter(const std::string& name)
  {
    std::lock_guard<std::mutex> lock{ _lock };
    auto& variant = *_progress_counters[name];
    return std::get<ProgressCounter<T>>(variant);
  }

  std::unordered_map<std::string, std::unique_ptr<ProgressCounter_t>> _progress_counters;
  std::mutex _lock;
};