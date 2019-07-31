#pragma once

#include <condition_variable>
#include <mutex>

struct Semaphore
{
  Semaphore() = default;
  explicit Semaphore(size_t initial_count);

  void notify();
  void wait();
  bool try_wait();

private:
  std::mutex _lock;
  std::condition_variable _cond;
  size_t _count;
};