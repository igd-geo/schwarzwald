#include "threading/TaskSystem.h"

#include <exception>

// std::future<void>
// TaskSystem::push(Task task)
// {
//   std::lock_guard<std::mutex> guard{ _tasks_lock };
//   _tasks.push(std::move(task));
//   _tasks_semaphore.notify();
// }

TaskSystem::TaskSystem() : _run(false) {}

void TaskSystem::run(uint32_t concurrency) {
  if (!_workers.empty() || _run) {
    throw std::runtime_error{"Task system already running!"};
  }

  _run = true;
  _workers.reserve(concurrency);
  for (uint32_t idx = 0; idx < concurrency; ++idx) {
    _workers.push_back(
        std::make_unique<std::thread>([this]() { run_worker(); }));
  }
}

void TaskSystem::stop_and_join() {
  _run = false;
  // Notify all workers to prevent hangs
  for (size_t idx = 0; idx < _workers.size(); ++idx) {
    _tasks_semaphore.notify();
  }
  for (auto &worker : _workers) {
    worker->join();
  }
  _workers.clear();
}

void TaskSystem::run_worker() {
  while (_run) {
    _tasks_semaphore.wait();
    if (!_run)
      break;

    _tasks_lock.lock();
    auto task = std::move(_tasks.front());
    _tasks.pop();
    _tasks_lock.unlock();

    task->execute_task();
  }
}