#include "threading/Semaphore.h"

Semaphore::Semaphore(size_t initial_count) : _count(initial_count) {}

// Implementation taken from https://stackoverflow.com/a/4793662

void Semaphore::notify() {
  std::lock_guard<std::mutex> lock(_lock);
  ++_count;
  _cond.notify_one();
}

void Semaphore::wait() {
  std::unique_lock<std::mutex> lock(_lock);
  while (!_count) // Handle spurious wake-ups.
    _cond.wait(lock);
  --_count;
}

bool Semaphore::try_wait() {
  std::lock_guard<std::mutex> lock(_lock);
  if (_count) {
    --_count;
    return true;
  }
  return false;
}