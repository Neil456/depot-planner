// A small open-addressing hash map from a packed state key to the per-state
// bookkeeping the space-time search needs.
//
// The Python reference keeps four dictionaries and a set (g_score, movement,
// parent, and closed) keyed on the same (x, y, t) tuple. One record per state
// replaces all of them with a single lookup, and linear probing over a flat
// vector keeps the search's inner loop free of allocations.

#ifndef DEPOT_STATE_TABLE_HPP_
#define DEPOT_STATE_TABLE_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace depot {

class StateTable {
 public:
  struct Record {
    double g = std::numeric_limits<double>::infinity();
    double movement = std::numeric_limits<double>::infinity();
    std::int64_t parent = -1;
    bool closed = false;
  };

  explicit StateTable(std::size_t expected = 1024) {
    std::size_t capacity = 16;
    while (capacity < expected * 2) capacity <<= 1;
    keys_.assign(capacity, 0);
    used_.assign(capacity, 0);
    records_.assign(capacity, Record{});
    mask_ = capacity - 1;
  }

  /// The record for `key`, or nullptr when the state has never been reached.
  const Record* Find(std::int64_t key) const {
    std::size_t slot = Slot(key);
    while (used_[slot]) {
      if (keys_[slot] == key) return &records_[slot];
      slot = (slot + 1) & mask_;
    }
    return nullptr;
  }

  /// The record for `key`, default-constructed on first use.
  Record& Emplace(std::int64_t key) {
    std::size_t slot = Slot(key);
    while (used_[slot]) {
      if (keys_[slot] == key) return records_[slot];
      slot = (slot + 1) & mask_;
    }
    used_[slot] = 1;
    keys_[slot] = key;
    records_[slot] = Record{};
    ++size_;
    if (size_ * 10 >= (mask_ + 1) * 7) {
      Grow();
      return Emplace(key);
    }
    return records_[slot];
  }

  std::size_t size() const { return size_; }

 private:
  static std::uint64_t Mix(std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
  }

  std::size_t Slot(std::int64_t key) const {
    return static_cast<std::size_t>(Mix(static_cast<std::uint64_t>(key))) & mask_;
  }

  void Grow() {
    std::vector<std::int64_t> old_keys = std::move(keys_);
    std::vector<char> old_used = std::move(used_);
    std::vector<Record> old_records = std::move(records_);
    const std::size_t capacity = (mask_ + 1) * 2;
    keys_.assign(capacity, 0);
    used_.assign(capacity, 0);
    records_.assign(capacity, Record{});
    mask_ = capacity - 1;
    size_ = 0;
    for (std::size_t i = 0; i < old_used.size(); ++i) {
      if (!old_used[i]) continue;
      Emplace(old_keys[i]) = old_records[i];
    }
  }

  std::vector<std::int64_t> keys_;
  std::vector<char> used_;
  std::vector<Record> records_;
  std::size_t mask_ = 0;
  std::size_t size_ = 0;
};

}  // namespace depot

#endif  // DEPOT_STATE_TABLE_HPP_
