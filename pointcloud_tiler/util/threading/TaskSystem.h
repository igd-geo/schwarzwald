#pragma once

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "threading/Async.h"
#include "threading/Semaphore.h"

struct TaskSystem {
  TaskSystem();

  template <typename Task> decltype(auto) push(Task task) {
    using Ret_t = std::decay_t<std::result_of_t<Task()>>;

    auto task_helper = std::make_unique<TaskHelper<Ret_t>>();
    auto future = task_helper->promise.get_future();
    task_helper->task = std::move(task);

    {
      std::lock_guard<std::mutex> guard{_tasks_lock};
      _tasks.push(std::move(task_helper));
    }

    _tasks_semaphore.notify();

    return async::Awaitable<Ret_t>{future.share()};
  }

  void run(uint32_t concurrency = std::thread::hardware_concurrency());
  void stop_and_join();

  size_t concurrency() const { return _workers.size(); }

private:
  struct TaskHelperBase {
    virtual ~TaskHelperBase() {}
    virtual void execute_task() = 0;
  };

  template <typename Ret> struct TaskHelper : TaskHelperBase {
    virtual ~TaskHelper() {}

    virtual void execute_task() override {
      if constexpr (std::is_void_v<Ret>) {
        task();
        promise.set_value();
      } else {
        promise.set_value(task());
      }
    }

    std::promise<Ret> promise;
    std::function<Ret()> task;
  };

  void run_worker();

  std::vector<std::unique_ptr<std::thread>> _workers;
  std::queue<std::unique_ptr<TaskHelperBase>> _tasks;
  Semaphore _tasks_semaphore;
  std::mutex _tasks_lock;
  std::atomic_bool _run;
};

template <typename Ret>
void await_all(const std::vector<std::future<Ret>> &futures) {
  for (auto &future : futures)
    future.wait();
}