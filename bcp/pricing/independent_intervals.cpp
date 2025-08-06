/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "output/formatting.h"
#include "pricing/independent_intervals.h"
#include "problem/debug.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::SteelBlue

IndependentIntervals::IndependentIntervals(const Map& map) :
    map_(map),

    interval_storage_(),
    default_interval_(),
    interval_set_(),
    finish_time_h_()
{
    ZoneScopedC(TRACY_COLOUR);

    // Reset data structures.
    clear();

    // Create default interval.
    default_interval_.start = 0;
    default_interval_.end = TIME_MAX;
    default_interval_.cost = 0.0;
    default_interval_.next = nullptr;
}

void IndependentIntervals::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    interval_storage_.reset(sizeof(Interval));
    interval_set_.clear();
}

void IndependentIntervals::add_penalty(const Node n,
                                       const Direction d,
                                       const Time earliest,
                                       const Time latest,
                                       const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("    Adding penalty at {} in the {} direction on interval [{},{}) of cost {} to agent {}",
            format_node(n, map_), d, earliest, latest, cost, a_);

    // Sum the penality.
    debug_assert(is_ge(cost, 0.0));
    insert_interval(n, d, earliest, latest, cost);
}

void IndependentIntervals::add_end_penalty(const Time earliest, const Time latest, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("    Adding end penalty on interval [{},{}) of cost {} to agent {}", earliest, latest, cost, a_);

    // Sum the penality.
    debug_assert(is_gt(cost, 0.0));
    insert_interval(-1, static_cast<Direction>(4), earliest, latest, cost);
}

void IndependentIntervals::finalise()
{
    ZoneScopedC(TRACY_COLOUR);

    // Compute the size of the vector for storing the finish time lower bound.
    const auto end_intervals = get_end_intervals();
    Time last_end_interval = 0;
    {
        auto interval = end_intervals;
        for (; interval->next; interval = interval->next) {}
        last_end_interval = interval->start;
    }
    const auto size = last_end_interval + 1;

    // Calculate the finish time lower bounds.
    finish_time_h_.resize(size);
    auto interval = end_intervals;
    do
    {
        const auto t_bound = std::min(interval->end, size);
        for (Time t = interval->start; t < t_bound; ++t)
        {
            finish_time_h_[t] = INF;
            auto it = interval;
            do
            {
                const auto arrival_time = std::max(t, it->start);
                const auto arrival_cost = arrival_time - t;
                const auto finish_cost = arrival_cost + it->cost;
                finish_time_h_[t] = std::min(finish_time_h_[t], finish_cost);
            }
            while ((it = it->next));
        }
    }
    while ((interval = interval->next));
}

void IndependentIntervals::insert_interval(const Node n,
                                           const Direction d,
                                           const Time start,
                                           const Time end,
                                           const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(n == -1 || map_[n]);
    debug_assert(0 <= start && start < end);
    debug_assert(end <= TIME_MAX);

    // Output whether new intervals are created or the penalty is summed on existing intervals.
    Bool created_interval = false;

    // Get the list of intervals at node n going in direction d.
    debug_assert(n >= 0 || d == 4);
    debug_assert(n * 5 + d >= -1);
    auto& intervals = interval_set_[Edge{n, d}];

    // Create the default interval if empty.
    if (!intervals)
    {
        ZoneScopedNC("Create default interval", TRACY_COLOUR);

        intervals = static_cast<Interval*>(interval_storage_.get_buffer<true, false>());
        intervals->start = 0;
        intervals->end = TIME_MAX;
        intervals->cost = 0.0;
        intervals->next = nullptr;
    }

    // Check invariants.
#ifdef DEBUG
    check_invariants(n, d);
#endif

    // Find or create (split) the earliest interval.
    Interval* current = intervals;
    Interval* prev = nullptr;
    {
        ZoneScopedNC("Insert earliest interval", TRACY_COLOUR);

        for (; current && current->start < start; current = current->next)
        {
            prev = current;
        }
        if (!current || current->start > start)
        {
            auto inserted = static_cast<Interval*>(interval_storage_.get_buffer<true, false>());
            inserted->start = start;
            inserted->end = current ? current->start : TIME_MAX;
            inserted->cost = prev->cost;
            inserted->next = current;

            debug_assert(prev);
            prev->end = start;
            prev->next = inserted;

            current = inserted;
            created_interval = true;
        }
    }

    // Accumulate the cost on the intermediate intervals.
    {
        ZoneScopedNC("Accumulate cost", TRACY_COLOUR);

        debug_assert(current->start == start);
        for (; current && current->end <= end; current = current->next)
        {
            current->cost += cost;
            prev = current;
        }
    }

    // Find or create (split) the latest interval.
    {
        ZoneScopedNC("Split latest interval", TRACY_COLOUR);

        if (!current)
        {
            debug_assert(prev->end == TIME_MAX);
            debug_assert(!prev->next);
        }
        if (current && current->start < end)
        {
            auto inserted = static_cast<Interval*>(interval_storage_.get_buffer<true, false>());
            inserted->start = end;
            inserted->end = current->end;
            inserted->cost = current->cost;
            inserted->next = current->next;

            current->end = end;
            current->cost += cost;
            current->next = inserted;

            created_interval = true;
        }
    }

    // Check invariants.
#ifdef DEBUG
    check_invariants(n, d);
#endif

    // Insert wait bypass intervals.
    if (n >= 0 && d == Direction::WAIT && cost != 0.0 && end < TIME_MAX && created_interval)
    {
        ZoneScopedNC("Creating wait bypass intervals", TRACY_COLOUR);

        if (const auto from_n = map_.get_north(n); map_[from_n])
        {
            insert_consecutive_intervals(from_n, Direction::SOUTH, std::max(start - 1, 0), end);
        }
        if (const auto from_n = map_.get_south(n); map_[from_n])
        {
            insert_consecutive_intervals(from_n, Direction::NORTH, std::max(start - 1, 0), end);
        }
        if (const auto from_n = map_.get_west(n); map_[from_n])
        {
            insert_consecutive_intervals(from_n, Direction::EAST, std::max(start - 1, 0), end);
        }
        if (const auto from_n = map_.get_east(n); map_[from_n])
        {
            insert_consecutive_intervals(from_n, Direction::WEST, std::max(start - 1, 0), end);
        }
    }
}

void IndependentIntervals::insert_consecutive_intervals(const Node n,
                                                        const Direction d,
                                                        const Time start,
                                                        const Time end)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(n == -1 || map_[n]);
    debug_assert(0 <= start && start < end);
    debug_assert(end < TIME_MAX);

    // Get the list of intervals at node n going in direction d.
    debug_assert(n >= 0 || d == 4);
    debug_assert(n * 5 + d >= -1);
    auto& intervals = interval_set_[Edge{n, d}];

    // Create the default interval if empty.
    if (!intervals)
    {
        ZoneScopedNC("Create default interval", TRACY_COLOUR);

        intervals = static_cast<Interval*>(interval_storage_.get_buffer<true, false>());
        intervals->start = 0;
        intervals->end = TIME_MAX;
        intervals->cost = 0.0;
        intervals->next = nullptr;
    }

    // Check invariants.
#ifdef DEBUG
    check_invariants(n, d);
#endif

    // Find or create (split) the earliest interval.
    Interval* current = intervals;
    Interval* prev = nullptr;
    {
        ZoneScopedNC("Insert earliest interval", TRACY_COLOUR);

        for (; current && current->start < start; current = current->next)
        {
            prev = current;
        }
        if (!current || current->start > start)
        {
            auto inserted = static_cast<Interval*>(interval_storage_.get_buffer<true, false>());
            inserted->start = start;
            inserted->end = current ? current->start : TIME_MAX;
            inserted->cost = prev->cost;
            inserted->next = current;

            debug_assert(prev);
            prev->end = start;
            prev->next = inserted;

            current = inserted;
        }
    }

    // Insert the intermediate intervals.
    {
        ZoneScopedNC("Insert intermediate intervals", TRACY_COLOUR);

        for (Time t = start; t < end; ++t)
        {
            debug_assert(current);
            debug_assert(current->start == t);
            if (current->end > t + 1)
            {
                auto inserted = static_cast<Interval*>(interval_storage_.get_buffer<true, false>());
                inserted->start = t + 1;
                inserted->end = current->end;
                inserted->cost = current->cost;
                inserted->next = current->next;

                current->end = t + 1;
                current->next = inserted;
            }
            debug_assert(current->end == t + 1);

            prev = current;
            current = current->next;
        }
    }

    // Check invariants.
#ifdef DEBUG
    check_invariants(n, d);
#endif
}

#ifdef DEBUG
void IndependentIntervals::check_invariants(const Node n, const Direction d)
{
    const Interval* it = get_intervals(n, d);
    debug_assert(it->start == 0);
    while (it && it->next)
    {
        debug_assert(it->end == it->next->start);
        it = it->next;
    }
    debug_assert(it->end == TIME_MAX);
    debug_assert(!it->next);
}
#endif

void IndependentIntervals::print() const
{
    ZoneScopedC(TRACY_COLOUR);

    Node last_n = -2;
    Size last_d = -1;
    println("----------------------------------------------------------------------------------------------------------------");
    println("{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}",
            "From N", "To N", "From XY", "To XY", "Start", "End", "Penalty");
    for (Node n = -1; n < map_.size(); ++n)
        for (Size d = (n < 0 ? 4 : 0); d <= Direction::WAIT; ++d)
            if (n == -1 || (map_[n] && map_[map_.get_destination(n, static_cast<Direction>(d))]))
            {
                auto interval = get_intervals(n, static_cast<Direction>(d));
                if (interval && (interval->next || interval->cost != 0.0))
                {
                    const auto dest = n >= 0 ? map_.get_destination(n, static_cast<Direction>(d)) : -1;
                    if (n != last_n || d != last_d)
                    {
                        println("----------------------------------------------------------------------------------------------------------------");
                    }
                    do
                    {
                        println("{:>16d}{:>16d}{:>16s}{:>16s}{:>16d}{:>16d}{:>16.4f}",
                                n, dest,
                                format_node(n, map_), format_node(dest, map_),
                                interval->start, interval->end, interval->cost);
                    }
                    while ((interval = interval->next));
                }
            }
   println("----------------------------------------------------------------------------------------------------------------");
}
