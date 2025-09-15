/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "pricing/shared_intervals.h"
#include "output/formatting.h"
#include "types/debug.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::Turquoise

SharedIntervals::SharedIntervals(const Map& map, const Bool is_shared) :
    map_(map),

    is_shared_(is_shared),
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

void SharedIntervals::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    interval_storage_.reset(sizeof(Interval));
    interval_set_.clear();
}

void SharedIntervals::add_penalty(const Node n, const Direction d, const Time earliest,
                                  const Time latest, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN(
        "    Adding penalty at {} in the {} direction on interval [{},{}) of cost {} to agent {}",
        format_node(n, map_),
        d,
        earliest,
        latest,
        cost,
        a_);

    // Sum the penality.
    DEBUG_ASSERT(is_ge(cost, 0.0));
    insert_interval(n, d, earliest, latest, cost);
}

void SharedIntervals::add_end_penalty(const Time earliest, const Time latest, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN("    Adding end penalty on interval [{},{}) of cost {} to agent {}",
            earliest,
            latest,
            cost,
            a_);

    // Sum the penality.
    DEBUG_ASSERT(is_gt(cost, 0.0));
    insert_interval(-1, static_cast<Direction>(4), earliest, latest, cost);
}

void SharedIntervals::finalise()
{
    ZoneScopedC(TRACY_COLOUR);

    // Compute the size of the vector for storing the finish time lower bound.
    const auto end_intervals = get_end_intervals();
    Time last_end_interval = 0;
    {
        auto interval = end_intervals;
        for (; interval->next; interval = interval->next)
        {
        }
        last_end_interval = interval->start;
    }
    const auto size = last_end_interval + 1;

    // Calculate the finish time lower bounds.
    finish_time_h_.resize(size);
#ifdef DEBUG
    std::fill(finish_time_h_.begin(), finish_time_h_.end(), COST_NAN);
#endif
    auto interval = end_intervals;
    do
    {
        const auto t_bound = std::min(interval->end, size);
        for (Time t = interval->start; t < t_bound; ++t)
        {
            finish_time_h_[t] = COST_INF;
            auto it = interval;
            do
            {
                const auto arrival_time = std::max(t, it->start);
                const auto arrival_cost = arrival_time - t;
                const auto finish_cost = arrival_cost + it->cost;
                finish_time_h_[t] = std::min(finish_time_h_[t], finish_cost);
            } while ((it = it->next));
        }
    } while ((interval = interval->next));
#ifdef DEBUG
    for (const auto x : finish_time_h_)
    {
        DEBUG_ASSERT(!std::isnan(x));
    }
#endif
}

Interval* SharedIntervals::duplicate_edge_intervals(const Node n, const Direction d)
{
    // Clone the default interval if not yet created. Otherwise clone the existing intervals if it
    // belongs to the shared set.
    auto [it, created] = interval_set_.try_emplace(Edge{n, d});
    auto& [intervals, intervals_are_shared] = it->second;
    if (created)
    {
        ZoneScopedNC("Create default interval", TRACY_COLOUR);

        intervals = new (interval_storage_.get_buffer<true, false>()) Interval;
        intervals->start = 0;
        intervals->end = TIME_MAX;
        intervals->cost = 0.0;
        intervals->next = nullptr;
    }
    else if (intervals_are_shared && !is_shared_)
    {
        auto agent_interval = &intervals;
        for (auto shared_interval = intervals; shared_interval;
             shared_interval = shared_interval->next)
        {
            *agent_interval = new (interval_storage_.get_buffer<true, false>()) Interval;
            std::memcpy(*agent_interval, shared_interval, sizeof(Interval));
            agent_interval = &(*agent_interval)->next;
        }
        DEBUG_ASSERT(!*agent_interval);
    }
    intervals_are_shared = is_shared_;
    return intervals;
}

Interval* SharedIntervals::duplicate_end_intervals()
{
    return duplicate_edge_intervals(-1, static_cast<Direction>(4));
}

void SharedIntervals::insert_interval(const Node n, const Direction d, const Time start,
                                      const Time end, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(n == -1 || map_[n]);
    DEBUG_ASSERT(0 <= start && start < end);
    DEBUG_ASSERT(end <= TIME_MAX);
    DEBUG_ASSERT(n >= 0 || d == 4);
    DEBUG_ASSERT(n * 5 + d >= -1);

    // Duplicate the intervals on the edge for the agent if the intervals are shared.
    const auto intervals = duplicate_edge_intervals(n, d);

    // Check invariants.
#ifdef DEBUG
    check_invariants(n, d);
#endif

    // Find or create (split) the earliest interval.
    Bool created_interval = false;
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
            auto inserted = new (interval_storage_.get_buffer<true, false>()) Interval;
            inserted->start = start;
            inserted->end = current ? current->start : TIME_MAX;
            inserted->cost = prev->cost;
            inserted->next = current;

            DEBUG_ASSERT(prev);
            prev->end = start;
            prev->next = inserted;

            current = inserted;
            created_interval = true;
        }
    }

    // Accumulate the cost on the intermediate intervals.
    {
        ZoneScopedNC("Accumulate cost", TRACY_COLOUR);

        DEBUG_ASSERT(current->start == start);
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
            DEBUG_ASSERT(prev->end == TIME_MAX);
            DEBUG_ASSERT(!prev->next);
        }
        if (current && current->start < end)
        {
            auto inserted = new (interval_storage_.get_buffer<true, false>()) Interval;
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

void SharedIntervals::insert_consecutive_intervals(const Node n, const Direction d,
                                                   const Time start, const Time end)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(n == -1 || map_[n]);
    DEBUG_ASSERT(0 <= start && start < end);
    DEBUG_ASSERT(end < TIME_MAX);
    DEBUG_ASSERT(n >= 0 || d == 4);
    DEBUG_ASSERT(n * 5 + d >= -1);

    // Duplicate the intervals on the edge for the agent if the intervals are shared.
    const auto intervals = duplicate_edge_intervals(n, d);

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
            auto inserted = new (interval_storage_.get_buffer<true, false>()) Interval;
            inserted->start = start;
            inserted->end = current ? current->start : TIME_MAX;
            inserted->cost = prev->cost;
            inserted->next = current;

            DEBUG_ASSERT(prev);
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
            DEBUG_ASSERT(current);
            DEBUG_ASSERT(current->start == t);
            if (current->end > t + 1)
            {
                auto inserted = new (interval_storage_.get_buffer<true, false>()) Interval;
                inserted->start = t + 1;
                inserted->end = current->end;
                inserted->cost = current->cost;
                inserted->next = current->next;

                current->end = t + 1;
                current->next = inserted;
            }
            DEBUG_ASSERT(current->end == t + 1);

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
void SharedIntervals::check_invariants(const Node n, const Direction d)
{
    const Interval* it = get_intervals(n, d);
    DEBUG_ASSERT(it->start == 0);
    while (it && it->next)
    {
        DEBUG_ASSERT(it->end == it->next->start);
        it = it->next;
    }
    DEBUG_ASSERT(it->end == TIME_MAX);
    DEBUG_ASSERT(!it->next);
}
#endif

void SharedIntervals::print() const
{
    ZoneScopedC(TRACY_COLOUR);

    Node last_n = -2;
    Size64 last_d = -1;
    PRINTLN("--------------------------------------------------------------------------------------"
            "------------------------------------------");
    PRINTLN("{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}{:>16s}",
            "From N",
            "To N",
            "From XY",
            "To XY",
            "Start",
            "End",
            "Penalty",
            "Shared");
    for (Node n = -1; n < map_.size(); ++n)
        for (Size64 d = (n < 0 ? 4 : 0); d <= Direction::WAIT; ++d)
            if (n == -1 || (map_[n] && map_[map_.get_destination(n, static_cast<Direction>(d))]))
            {
                auto interval = get_intervals(n, static_cast<Direction>(d));
                if (interval && (interval->next || interval->cost != 0.0))
                {
                    const auto dest =
                        n >= 0 ? map_.get_destination(n, static_cast<Direction>(d)) : -1;

                    Bool is_shared = true;
                    if (auto it = interval_set_.find(Edge{n, static_cast<Direction>(d)});
                        it != interval_set_.end())
                    {
                        is_shared = it->second.is_shared;
                    }

                    if (n != last_n || d != last_d)
                    {
                        PRINTLN("------------------------------------------------------------------"
                                "--------------------------------------------------------------");
                    }
                    do
                    {
                        PRINTLN("{:>16d}{:>16d}{:>16s}{:>16s}{:>16d}{:>16d}{:>16.4f}{:>16}",
                                n,
                                dest,
                                format_node(n, map_),
                                format_node(dest, map_),
                                interval->start,
                                interval->end,
                                interval->cost,
                                is_shared);
                    } while ((interval = interval->next));
                }
            }
    PRINTLN("--------------------------------------------------------------------------------------"
            "------------------------------------------");
}
