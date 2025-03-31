/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "pricing/finish_time_penalties.h"
#include "pricing/interval.h"
#include "problem/map.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/memory_pool.h"

struct IntervalList
{
    Interval* list;
    Bool is_shared;
};

class SharedIntervals
{
    // Problem
    const Map& map_;

    // Intervals
    Bool is_shared_;
    MemoryPool interval_storage_;
    Interval default_interval_;
    HashMap<Edge, IntervalList> interval_set_;
    Vector<Cost> finish_time_h_;

  public:
    // Constructors and destructor
    SharedIntervals() = delete;
    SharedIntervals(const Map& map, const Bool is_shared);
    ~SharedIntervals() = default;
    SharedIntervals(const SharedIntervals&) noexcept = delete;
    SharedIntervals(SharedIntervals&&) noexcept = default;
    SharedIntervals& operator=(const SharedIntervals&) noexcept = delete;
    SharedIntervals& operator=(SharedIntervals&&) noexcept = delete;

    // Clear existing intervals
    void clear();

    // Getters
    const Interval* get_intervals(const Node n, const Direction d) const
    {
        debug_assert(n == -1 || map_[map_.get_destination(Edge{n, d})]);
        const Interval* intervals = &default_interval_;
        if (auto it = interval_set_.find(Edge{n, d}); it != interval_set_.end())
        {
            intervals = it->second.list;
        }
        return intervals;
    }
    const Interval* get_end_intervals() const { return get_intervals(-1, static_cast<Direction>(4)); }
    Cost get_finish_time_h(const Time t) const
    {
        debug_assert(!finish_time_h_.empty());
        const auto index = std::min<Time>(finish_time_h_.size() - 1, t);
        return finish_time_h_[index];
    }

    // Duplicating shared intervals to agent-specific intervals
    inline void copy_intervals_set(const SharedIntervals& intervals) { interval_set_ = intervals.interval_set_; }
    Interval* duplicate_edge_intervals(const Node n, const Direction d);
    Interval* duplicate_end_intervals();

    // Insert intervals
    void add_penalty(const Node n, const Direction d, const Time earliest, const Time latest, const Cost cost);
    void add_end_penalty(const Time earliest, const Time latest, const Cost cost);
    void finalise();

    // Debug
    void print() const;

  private:
    // Insert intervals
    void insert_interval(const Node n, const Direction d, const Time start, const Time end, const Cost cost);
    void insert_consecutive_intervals(const Node n, const Direction d, const Time start, const Time end);

    // Debug
#ifdef DEBUG
    void check_invariants(const Node n, const Direction d);
#endif
};
