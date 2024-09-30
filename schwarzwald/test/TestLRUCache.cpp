#include <catch2/catch_all.hpp>

#include <datastructures/LRUCache.h>

#include <random>

using namespace boost::units::information;

struct FixedSize28 // 28 bytes because the key adds another 4
{
  int32_t id;
  char buf[24];
};

struct Size24 {
  char buf[24];
};

using ComplexType28 = std::pair<int32_t, Size24>;

SCENARIO("LRUCache (simple type)") {
  constexpr unit::byte capacity = 64 * byte;

  FixedSize28 a, b, c, d;
  a.id = 1;
  b.id = 2;
  c.id = 3;
  d.id = 4;

  WHEN("The cache is created") {
    LRUCache<int32_t, FixedSize28> cache{capacity};

    THEN("The cache is empty and has its full capacity") {
      REQUIRE(cache.size() == 0);
      REQUIRE(cache.capacity() == capacity);
    }

    WHEN("An item is added to the cache") {
      cache.put(a.id, a);

      THEN("Size increases and capacity decreases accordingly") {
        REQUIRE(cache.size() == 1);
        REQUIRE(cache.capacity() == (capacity - (32 * byte)));
      }
      THEN("The item can be retrieved") {
        FixedSize28 retrieved_item{};
        REQUIRE(cache.try_get(a.id, retrieved_item));
        REQUIRE(retrieved_item.id == a.id);
      }

      WHEN("The item is updated") {
        cache.put(a.id, d);

        THEN("Size and capacity don't change") {
          REQUIRE(cache.size() == 1);
          REQUIRE(cache.capacity() == (capacity - (32 * byte)));
        }
        THEN("The new item can be retrieved") {
          FixedSize28 retrieved_item{};
          REQUIRE(cache.try_get(a.id, retrieved_item));
          REQUIRE(retrieved_item.id == d.id); // !We did change to d after all!
        }
      }
    }

    WHEN("Three items are added to a cache with capacity 2") {
      int32_t evicted_count = 0, last_evicted = 0;
      cache.add_evict_handler([&](int32_t id, FixedSize28 const &element) {
        ++evicted_count;
        last_evicted = id;
      });

      cache.put(a.id, a);
      cache.put(b.id, b);
      cache.put(c.id, c);

      THEN("The cache is full") {
        REQUIRE(cache.size() == 2);
        REQUIRE(cache.capacity() == 0 * byte);
      }
      THEN("The first added item has been evicted") {
        REQUIRE(evicted_count == 1);
        REQUIRE(last_evicted == 1);
        FixedSize28 tmp;
        REQUIRE(!cache.try_get(a.id, tmp));
      }
      THEN("The second and third items can be retrieved") {
        FixedSize28 retrieved{};
        REQUIRE(cache.try_get(c.id, retrieved));
        REQUIRE(retrieved.id == c.id);

        REQUIRE(cache.try_get(b.id, retrieved));
        REQUIRE(retrieved.id == b.id);
      }

      WHEN("The cache is cleared") {
        cache.clear();

        THEN("The size is zero and the capacity is restored") {
          REQUIRE(cache.size() == 0);
          REQUIRE(cache.capacity() == capacity);
        }
        THEN("All items have been evicted") {
          REQUIRE(evicted_count == 3); // 2 items in the cache, plus the one
                                       // that was evicted when c was added
        }
      }
    }
  }
}

SCENARIO("LRUCache (complex type)") {
  constexpr unit::byte capacity = 64 * byte;

  ComplexType28 a, b, c, d;
  a.first = 1;
  b.first = 2;
  c.first = 3;
  d.first = 4;

  WHEN("The cache is created") {
    LRUCache<int32_t, ComplexType28> cache{capacity};

    THEN("The cache is empty and has its full capacity") {
      REQUIRE(cache.size() == 0);
      REQUIRE(cache.capacity() == capacity);
    }

    WHEN("An item is added to the cache") {
      cache.put(a.first, a);

      THEN("Size increases and capacity decreases accordingly") {
        REQUIRE(cache.size() == 1);
        REQUIRE(cache.capacity() == (capacity - (32 * byte)));
      }
      THEN("The item can be retrieved") {
        ComplexType28 retrieved_item;
        REQUIRE(cache.try_get(a.first, retrieved_item));
        REQUIRE(retrieved_item.first == a.first);
      }

      WHEN("The item is updated") {
        cache.put(a.first, d);

        THEN("Size and capacity don't change") {
          REQUIRE(cache.size() == 1);
          REQUIRE(cache.capacity() == (capacity - (32 * byte)));
        }
        THEN("The new item can be retrieved") {
          ComplexType28 retrieved_item;
          REQUIRE(cache.try_get(a.first, retrieved_item));
          REQUIRE(retrieved_item.first ==
                  d.first); // !We did change to d after all!
        }
      }
    }

    WHEN("Three items are added to a cache with capacity 2") {
      int32_t evicted_count = 0, last_evicted = 0;
      cache.add_evict_handler([&](int32_t id, ComplexType28 const &element) {
        ++evicted_count;
        last_evicted = id;
      });

      cache.put(a.first, a);
      cache.put(b.first, b);
      cache.put(c.first, c);

      THEN("The cache is full") {
        REQUIRE(cache.size() == 2);
        REQUIRE(cache.capacity() == 0 * byte);
      }
      THEN("The first added item has been evicted") {
        REQUIRE(evicted_count == 1);
        REQUIRE(last_evicted == 1);
        ComplexType28 tmp;
        REQUIRE(!cache.try_get(a.first, tmp));
      }
      THEN("The second and third items can be retrieved") {
        ComplexType28 retrieved;
        REQUIRE(cache.try_get(c.first, retrieved));
        REQUIRE(retrieved.first == c.first);

        REQUIRE(cache.try_get(b.first, retrieved));
        REQUIRE(retrieved.first == b.first);
      }

      WHEN("The cache is cleared") {
        cache.clear();

        THEN("The size is zero and the capacity is restored") {
          REQUIRE(cache.size() == 0);
          REQUIRE(cache.capacity() == capacity);
        }
        THEN("All items have been evicted") {
          REQUIRE(evicted_count == 3); // 2 items in the cache, plus the one
                                       // that was evicted when c was added
        }
      }
    }
  }
}

SCENARIO("LRUCache stress-test") {
  constexpr unit::byte capacity = 4096 * byte;

  std::mt19937 rnd_engine;
  rnd_engine.seed(static_cast<uint32_t>(time(nullptr)));
  std::uniform_int_distribution<int32_t> id_distribution{0, 256};
  std::uniform_int_distribution<int32_t> iterations_distribution{1 << 15,
                                                                 1 << 16};

  WHEN("The cache is created and used heavily") {
    LRUCache<int32_t, ComplexType28> cache{capacity};
    const auto iterations = iterations_distribution(rnd_engine);
    for (auto iteration = 0; iteration < iterations; ++iteration) {
      ComplexType28 element;
      element.first = id_distribution(rnd_engine);
      cache.put(element.first, element);
    }
    cache.clear();

    THEN("The capacity is correct") { REQUIRE(cache.capacity() == capacity); }
  }
}