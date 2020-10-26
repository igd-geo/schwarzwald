#pragma once

#include <condition_variable>
#include <mutex>

struct Semaphore
{
  explicit Semaphore(size_t initial_count = 0);

  void notify();
  void wait();
  bool try_wait();

private:
  std::mutex _lock;
  std::condition_variable _cond;
  size_t _count;
};