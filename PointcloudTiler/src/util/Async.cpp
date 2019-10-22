#include "util/Async.h"

async::Awaitable<void>
async::all(std::vector<std::shared_future<void>> &&futures) {
  return Awaitable<void>{[futures{std::move(futures)}]() mutable {
    for (auto &future : futures)
      future.wait();
  }};
}

async::Awaitable<void> async::all(std::vector<Awaitable<void>> &&awaitables) {
  return Awaitable<void>{[awaitables{std::move(awaitables)}]() mutable {
    for (auto &awaitable : awaitables)
      awaitable.await();
  }};
}