#pragma once

#include <future>
#include <vector>

namespace async {

template <typename Ret> struct Awaitable {
  explicit Awaitable(std::function<Ret()> &&call) : _call(std::move(call)) {}
  Awaitable(std::shared_future<Ret> future)
      : _call([future]() mutable { return future.get(); }) {}

  Ret await() { return _call(); }

private:
  std::function<Ret()> _call;
};

template <> struct Awaitable<void> {
  explicit Awaitable(std::function<void()> &&call) : _call(std::move(call)) {}
  Awaitable(std::shared_future<void> future)
      : _call([future]() mutable { future.get(); }) {}

  void await() { _call(); }

private:
  std::function<void()> _call;
};

template <typename Ret>
std::enable_if_t<!std::is_void_v<Ret>, Awaitable<std::vector<Ret>>>
all(std::vector<std::shared_future<Ret>> &&futures) {
  return Awaitable<std::vector<Ret>>{[futures{std::move(futures)}]() mutable {
    std::vector<Ret> results;
    for (auto &future : futures)
      results.push_back(future.get());
    return results;
  }};
}

template <typename Ret>
std::enable_if_t<!std::is_void_v<Ret>, Awaitable<std::vector<Ret>>>
all(std::vector<Awaitable<Ret>> &&awaitables) {
  return Awaitable<std::vector<Ret>>{
      [awaitables{std::move(awaitables)}]() mutable {
        std::vector<Ret> results;
        for (auto &awaitable : awaitables)
          results.push_back(awaitable.await());
        return results;
      }};
}

Awaitable<void> all(std::vector<std::shared_future<void>> &&futures);
Awaitable<void> all(std::vector<Awaitable<void>> &&awaitables);

template <typename Ret>
Awaitable<Ret> flatten(Awaitable<Awaitable<Ret>> &&nested_awaitable) {
  return Awaitable<Ret>{
      [nested_awaitable{std::move(nested_awaitable)}]() mutable {
        auto inner_awaitable = nested_awaitable.await();
        if constexpr (std::is_void_v<Ret>) {
          inner_awaitable.await();
        } else {
          return inner_awaitable.await();
        }
      }};
}

} // namespace async