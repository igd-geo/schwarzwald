#pragma once

#include <debug/ThroughputCounter.h>
#include <future>
#include <taskflow/taskflow.hpp>

struct TilingScheduler
{
  virtual ~TilingScheduler() = 0;

  virtual std::future<void> execute_tiling_iteration(
    tf::Taskflow& read_graph,
    tf::Taskflow& index_graph) = 0;
  virtual std::pair<uint32_t, uint32_t> get_read_and_index_concurrency(
    uint32_t remaining_files) = 0;
};

struct FixedThreadsSchedulerArgs
{
  uint32_t read_threads;
  uint32_t indexing_threads;
};

struct FixedThreadsScheduler : TilingScheduler
{
  explicit FixedThreadsScheduler(FixedThreadsSchedulerArgs args);

  ~FixedThreadsScheduler() override;
  std::future<void> execute_tiling_iteration(
    tf::Taskflow& read_graph,
    tf::Taskflow& index_graph) override;

  std::pair<uint32_t, uint32_t> get_read_and_index_concurrency(
    uint32_t remaining_files) override;

private:
  tf::Executor _read_executor, _indexing_executor;
  std::shared_ptr<tf::ChromeObserver> _read_executor_observer;
  std::shared_ptr<tf::ChromeObserver> _indexing_executor_observer;
};

struct AdaptiveScheduler : TilingScheduler
{
  AdaptiveScheduler(uint32_t total_threads,
                    ThroughputSampler& read_throughput_sampler,
                    ThroughputSampler& indexing_throughput_sampler);
  ~AdaptiveScheduler() override;

  std::future<void> execute_tiling_iteration(
    tf::Taskflow& read_graph,
    tf::Taskflow& index_graph) override;

  std::pair<uint32_t, uint32_t> get_read_and_index_concurrency(
    uint32_t remaining_files) override;

private:
  uint32_t _num_read_threads;
  uint32_t _num_index_threads;

  tf::Executor _executor;
  std::shared_ptr<tf::ChromeObserver> _executor_observer;

  ThroughputSampler& _read_throughput_sampler;
  ThroughputSampler& _indexing_throughput_sampler;
};