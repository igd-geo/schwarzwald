#pragma once

#include "Units.h"
#include "concepts/MemoryIntrospection.h"

#include <deque>
#include <functional>

/**
 * Cache using least-recently-used caching scheme. It supports lookup by key in
 * linear time and is bounded in memory. The key and value types must implement
 * the MemoryIntrospectable concept. A callback function can be registered that
 * gets invoked for every entry that gets evicted from the cache.
 */
template<typename Key, typename Value>
struct LRUCache
{
  BOOST_CONCEPT_ASSERT((concepts::MemoryIntrospectable<Key>));
  BOOST_CONCEPT_ASSERT((concepts::MemoryIntrospectable<Value>));

  using EvictHandler = std::function<void(Key const&, Value const&)>;

  explicit LRUCache(unit::byte memory_capacity)
    : _memory_capacity(memory_capacity)
    , _free_capacity(memory_capacity)
  {}

  void put(Key const& key, Value value)
  {
    assert((concepts::size_in_memory(key) + concepts::size_in_memory(value)) <= _memory_capacity);
    const auto entry_iter = find(key);
    if (entry_iter == std::end(_entries)) {
      insert_new(key, std::move(value));
    } else {
      update_entry(entry_iter, std::move(value));
    }
  }

  bool try_get(Key const& key, Value& value)
  {
    const auto entry_iter = find(key);
    if (entry_iter == std::end(_entries))
      return false;

    // Move entry to front
    auto tmp_entry = std::move(*entry_iter);
    _entries.erase(entry_iter);
    _entries.push_front(std::move(tmp_entry));

    value = _entries.front().second;
    return true;
  }

  void clear()
  {
    while (!_entries.empty()) {
      evict(std::prev(_entries.end()));
    }
  }

  void add_evict_handler(EvictHandler handler) { _evict_handlers.push_back(std::move(handler)); }

  size_t size() const { return _entries.size(); }
  unit::byte capacity() const { return _free_capacity; }

private:
  using Entry = std::pair<Key, Value>;

  unit::byte const _memory_capacity;
  unit::byte _free_capacity;
  std::deque<Entry> _entries;
  std::vector<EvictHandler> _evict_handlers;

  void insert_new(Key const& key, Value value)
  {
    const auto required_size = concepts::size_in_memory(key) + concepts::size_in_memory(value);

    // Evict oldest elements until we have enough free memory for the new entry
    while (_free_capacity < required_size) {
      assert(!_entries.empty());
      evict(std::prev(_entries.end()));
    }

    _entries.emplace_front(key, std::move(value));
    _free_capacity -= required_size;
  }

  void update_entry(typename std::deque<Entry>::iterator entry, Value new_value)
  {
    // Remove old entry temporarily
    auto tmp_entry = std::move(*entry);
    _entries.erase(entry);

    // Remove enough entries to accomodate possibly larger size of new_value
    const auto size_difference =
      concepts::size_in_memory(new_value) - concepts::size_in_memory(tmp_entry.second);
    while (size_difference > _free_capacity) {
      assert(!_entries.empty());
      evict(std::prev(_entries.end()));
    }

    // Push entry to front with new value
    _entries.emplace_front(tmp_entry.first, std::move(new_value));
    _free_capacity -= size_difference;
  }

  typename std::deque<Entry>::iterator find(Key const& key)
  {
    return std::find_if(std::begin(_entries), std::end(_entries), [&key](Entry const& entry) {
      return entry.first == key;
    });
  }

  void evict(typename std::deque<Entry>::const_iterator iter)
  {
    for (auto& handler : _evict_handlers) {
      handler(iter->first, iter->second);
    }
    _free_capacity +=
      concepts::size_in_memory(iter->first) + concepts::size_in_memory(iter->second);
    _entries.erase(iter);
  }
};