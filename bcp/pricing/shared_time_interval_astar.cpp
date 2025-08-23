/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "master/master.h"
#include "output/formatting.h"
#include "pricing/distance_heuristic.h"
#include "pricing/shared_time_interval_astar.h"
#include "problem/problem.h"
#include "types/bitset.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::LightSkyBlue

#ifdef DEBUG
static Bool verbose = false;
#endif
#if defined(DEBUG) || defined(TRACY_ENABLE)
static UInt64 iter = 0;
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

SharedTimeIntervalAStar::SharedTimeIntervalAStar(const Instance& instance,
                                                 Problem& problem,
                                                 DistanceHeuristic& distance_heuristic,
                                                 Size& num_added) :
    instance_(instance),
    map_(instance_.map),
    problem_(problem),
    distance_heuristic_(distance_heuristic),

    a_(),
    start_(),
    target_(),

    waypoints_(nullptr),
    h_waypoint_to_target_(),
    next_waypoint_index_(0),
    waypoint_time_(),
    h_to_waypoint_(nullptr),

    constant_(),
    intervals_(nullptr),
    once_off_penalties_(nullptr),
    rectangle_penalties_(nullptr),
    once_off_bitset_size_(0),
    rectangle_bitset_size_(0),
    earliest_target_time_(0),
    latest_target_time_(TIME_MAX),

    label_storage_(),
    closed_(nullptr),
    open_infeasible_(),
    open_feasible_(),
    obj_()
#ifdef USE_SHARED_TIME_INTERVAL_ASTAR_PRICER
  , num_added_(num_added)
#endif

// #ifdef CHECK_USING_ASTAR
//   , astar_(instance, problem, distance_heuristic, a)
// #endif
{
    ZoneScopedC(TRACY_COLOUR);

    // Allocate memory for the closed set.
    {
        ZoneScopedNC("Allocate closed set", TRACY_COLOUR);

        closed_ = static_cast<void**>(std::malloc((map_.size() + 1) * sizeof(void*)));
        ++closed_;
    }

    // Reset data structures.
    reset();
}

#pragma GCC diagnostic pop

template<Bool feasible>
Byte* SharedTimeIntervalAStar::get_once_off_bitset(Label<feasible>* const label)
{
    return label->bitset;
}

template<Bool feasible>
Byte* SharedTimeIntervalAStar::get_rectangle_bitset(Label<feasible>* const label)
{
    return label->bitset + once_off_bitset_size_;
}

SharedTimeIntervalAStar::~SharedTimeIntervalAStar()
{
    ZoneScopedC(TRACY_COLOUR);

    --closed_;
    std::free(closed_);
}

void SharedTimeIntervalAStar::reset()
{
    ZoneScopedC(TRACY_COLOUR);

    waypoints_ = nullptr;
    intervals_ = nullptr;
    once_off_penalties_ = nullptr;
    rectangle_penalties_ = nullptr;
    // earliest_target_time_ and latest_target_time_ are computed, whereas they are directly inputted in time-expanded A*
}

template<Bool feasible, Bool towards_end>
void SharedTimeIntervalAStar::generate_start()
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the input data.
    const auto& intervals = *intervals_;

    // Get the start nodetime.
    const NodeTime next_nt = start_;
    const auto& next_n = next_nt.n;
    constexpr Time next_t = 0;
    debug_assert(map_[next_n]);

    // Calculate the distance to the next waypoint.
    constexpr Size next_waypoint_index = 0;
    const auto h_to_waypoint = h_to_waypoint_[next_n];
    debug_assert(0 <= h_to_waypoint && h_to_waypoint < TIME_MAX);
    const auto waypoint_t = waypoint_time_;
    const auto h_to_waypoint_time = std::max(h_to_waypoint, waypoint_t - next_t);

    // Calculate the distance to the target.
    const auto h_waypoint_to_target = towards_end ? 0 : h_waypoint_to_target_[next_waypoint_index];
    debug_assert(0 <= h_waypoint_to_target && h_waypoint_to_target < TIME_MAX);
    const auto h_to_target = h_to_waypoint_time + h_waypoint_to_target;
    debug_assert(h_to_target >= earliest_target_time_ - next_t);

    // Calculate the earliest arrival time.
    const auto next_time_f = next_t + h_to_target;

    // Exit if time infeasible.
    if ((!towards_end && next_t + h_to_waypoint > waypoint_t) || next_time_f > latest_target_time_)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("        Time infeasible at start nt {}, n {}, xyt {}",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_));
        }
#endif

        // Exit.
        return;
    }

    // Allocate memory for the label.
    auto next = static_cast<Label<feasible>*>(label_storage_.get_buffer<false, true>());

    // Calculate the g value.
    next->g = constant_;

    // Calculate the f value.
    if constexpr (feasible)
    {
        // Calculate the h value.
        const auto h_target_to_end = intervals.get_finish_time_h(next_time_f);
        // This h for feasible master problems is stronger than in LPA* which doesn't support varying h at the moment.
        const auto next_h = h_to_target + h_target_to_end;

        // Update the f value.
        next->f = next->g + next_h;
        debug_assert(next->f == next->g || is_ge(next->f, next->g));
    }
    else
    {
        next->time_f = next_time_f;
    }

    // Exit if cost infeasible.
    Cost cost_f;
    if constexpr (feasible) { cost_f = next->f; } else { cost_f = next->g; }
    if (is_ge(cost_f, 0.0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                println("        Cost infeasible at start nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->f,
                        next->g,
                        format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                        next->reservations);
            }
            else
            {
                println("        Cost infeasible at start nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->g,
                        format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                        next->reservations,
                        next->time_f,
                        next_t);
            }
        }
#endif

        // Exit.
        return;
    }

    // Push into the priority queue.
    next->nt = next_nt;
    next = push<feasible, false>(next);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if constexpr (feasible)
        {
            println("        Generated start at nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_),
                    next->f,
                    next->g,
                    format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                    format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                    next->reservations);
        }
        else
        {
            println("        Generated start at nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_),
                    next->g,
                    format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                    format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                    next->reservations,
                    next->time_f,
                    next_t);
        }
    }
#endif
}

template<Bool feasible, Bool towards_end>
void SharedTimeIntervalAStar::generate_nodetimes(const Node next_n, const Direction d, Label<feasible>* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(map_[next_n]);

    // Get the input data.
    const auto& intervals = *intervals_;
    const auto& once_off_penalties = *once_off_penalties_;
    const auto& rectangle_penalties = *rectangle_penalties_;

    // Get the label data.
    const auto current_nt = current->nt;

    // Calculate the distance to the next waypoint.
    const auto next_waypoint_index = towards_end ? 0 : next_waypoint_index_;
    const auto h_to_waypoint = h_to_waypoint_[next_n];
    debug_assert(0 <= h_to_waypoint && h_to_waypoint < TIME_MAX);
    const auto waypoint_t = waypoint_time_;

    // Calculate the distance to the target.
    const auto h_waypoint_to_target = towards_end ? 0 : h_waypoint_to_target_[next_waypoint_index];
    debug_assert(0 <= h_waypoint_to_target && h_waypoint_to_target < TIME_MAX);

    // Skip intervals earlier than the current time.
    const Interval* move_interval;
    {
        // ZoneScopedNC("Skip to earliest move interval", TRACY_COLOUR);

        move_interval = intervals.get_intervals(current_nt.n, d);
        while (current_nt.t >= move_interval->end)
        {
            move_interval = move_interval->next;
        }
    }
    const Interval* wait_interval;
    {
        // ZoneScopedNC("Skip to earliest wait interval", TRACY_COLOUR);

        wait_interval = intervals.get_intervals(current_nt.n, Direction::WAIT);
        while (current_nt.t >= wait_interval->end)
        {
            wait_interval = wait_interval->next;
        }
    }

    // Expand using each move interval.
    Time wait_end = current_nt.t;
    Cost wait_cost = 0.0;
    do
    {
        // ZoneScopedNC("Expand one interval", TRACY_COLOUR);

        // Calculate the transition cost.
#ifdef DEBUG
        Cost check_wait_cost = 0.0;
        {
            auto wait_interval = intervals.get_intervals(current_nt.n, Direction::WAIT);
            do
            {
                auto wait_duration = std::min(wait_interval->end, std::max(current_nt.t, move_interval->start)) -
                    std::max(current_nt.t, wait_interval->start);
                wait_duration = std::max(wait_duration, 0);
                check_wait_cost += wait_duration == 0 ? 0.0 : wait_duration * (default_edge_cost<feasible>() + wait_interval->cost);
            }
            while ((wait_interval = wait_interval->next));
        }
#endif
        {
            // ZoneScopedNC("Calculate wait cost", TRACY_COLOUR);
            do
            {
                const auto wait_start = wait_end;
                debug_assert(wait_interval->start <= wait_start);
                wait_end = std::min(std::max(wait_end, move_interval->start), wait_interval->end);
                const auto duration_in_wait_interval = wait_end - wait_start;
                debug_assert(wait_start >= 0);
                debug_assert(wait_end >= 0);
                debug_assert(duration_in_wait_interval >= 0);
                if (duration_in_wait_interval > 0 && wait_interval->cost == INF)
                {
                    return;
                }
                wait_cost += duration_in_wait_interval == 0 ?
                             0.0 :
                             duration_in_wait_interval * (default_edge_cost<feasible>() + wait_interval->cost);
                debug_assert(!std::isnan(wait_cost));
            }
            while (wait_end >= wait_interval->end && (wait_interval = wait_interval->next));
            debug_assert(is_eq(wait_cost, check_wait_cost));
        }

        // Get the next nodetime.
        const auto next_t = wait_end + 1;
        const NodeTime next_nt{next_n, next_t};

        // Calculate the earliest arrival time.
        const auto h_to_waypoint_time = std::max(h_to_waypoint, waypoint_t - next_t);
        const auto h_to_target = h_to_waypoint_time + h_waypoint_to_target;
        debug_assert(h_to_target >= earliest_target_time_ - next_t);
        const auto next_time_f = next_t + h_to_target;

        // Exit if time infeasible.
        if ((!towards_end && next_t + h_to_waypoint > waypoint_t) || next_time_f > latest_target_time_)
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("        Time infeasible at nt {}, n {}, xyt {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_));
            }
#endif

            // Exit.
            return;
        }

        // Allocate memory for the label.
        auto next = static_cast<Label<feasible>*>(label_storage_.get_buffer<false, false>());
        std::memcpy(next, current, label_storage_.object_size());

        // Calculate the g value.
        next->g += wait_cost + default_edge_cost<feasible>() + move_interval->cost;
        {
            auto once_off_bitset = get_once_off_bitset<feasible>(next);
            for (Size index = 0; index < once_off_penalties.size(); ++index)
                if (!get_bitset(once_off_bitset, index))
                {
                    // Get the penalty data.
                    const auto& [cost, nt, d] = once_off_penalties[index];

                    // Accumulate the penalty if the nodetime is crossed.
                    const auto crossed = (d == OnceOffDirection::GEq) ?
                        ((nt.n == current_nt.n && nt.t <= next_t - 1) || (nt.n == next_n && nt.t <= next_t)) :
                        (nt.n == next_n && nt.t >= next_t);
                    if (crossed)
                    {
                        ++next->num_bitset_ones;
                        set_bitset(once_off_bitset, index);
                        next->g += cost;
                    }
                }
        }
        if (d != Direction::WAIT)
        {
            auto rectangle_bitset = get_rectangle_bitset<feasible>(next);
            for (Size index = 0; index < rectangle_penalties.size(); ++index)
            {
                // Get the penalty data.
                const auto& [cost, first_ets, length, n_increment] = rectangle_penalties[index];

                // Accumulate the penalty if the boundary is crossed.
                if (d == first_ets[0].d)
                {
                    const auto num_boundaries_crossed = get_bitset(rectangle_bitset, index);
                    const auto et = first_ets[num_boundaries_crossed];
                    const auto i = (next_t - 1) - et.t;
                    const auto crossed = (0 <= i && i <= length) &&
                                         (current_nt.n == et.n + i * n_increment) &&
                                         (next_t - 1 == et.t + i);
                    if (crossed)
                    {
                        ++next->num_bitset_ones;
                        const auto add_cost = !flip_bitset(rectangle_bitset, index);
                        next->g += cost * add_cost;
                    }
                }
            }
        }
        debug_assert(is_ge(next->g, current->g));

        // Calculate the f value.
        if constexpr (feasible)
        {
            // Calculate the h value.
            const auto h_target_to_end = intervals.get_finish_time_h(next_time_f);
            // This h for feasible master problems is stronger than in LPA* which doesn't support varying h at the moment.
            const auto next_h = h_to_target + h_target_to_end;

            // Update the f value.
            next->f = next->g + next_h;
            debug_assert(next->f == next->g || is_ge(next->f, next->g));
        }
        else
        {
            next->time_f = next_time_f;
        }

        // Exit if cost infeasible.
        Cost cost_f;
        if constexpr (feasible) { cost_f = next->f; } else { cost_f = next->g; }
        if (is_ge(cost_f, 0.0))
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                if constexpr (feasible)
                {
                    println("        Cost infeasible nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                            next_nt.id,
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->f,
                            next->g,
                            format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                            format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                            next->reservations);
                }
                else
                {
                    println("        Cost infeasible nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                            next_nt.id,
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                            format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                            next->reservations,
                            next->time_f,
                            next_t);
                }
            }
#endif

            // Proceed to the next interval.
            continue;
        }

        // Calculate the number of reservations.
#ifdef USE_RESERVATION_LOOKUP
        if constexpr (feasible)
        {
            // ZoneScopedNC("Calculate reservations", TRACY_COLOUR);

            for (Time t = current->nt.t + 1; t < next_t; ++t)
            {
                const NodeTime nt{current_nt.n, t};
                next->reservations += calculate_reservation(nt);
            }
            next->reservations += calculate_reservation(next_nt);
        }
#endif

        // Push into the priority queue.
        next->nt = next_nt;
        next->parent = current;
#ifdef DEBUG
        auto next_copy = next;
#endif
        next = push<feasible, false>(next);

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                println("        {} nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->f,
                        next_copy->g,
                        format_bitset(get_once_off_bitset<feasible>(next_copy), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next_copy), rectangle_penalties_->size()),
                        next_copy->reservations);
            }
            else
            {
                println("        {} nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset<feasible>(next_copy), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next_copy), rectangle_penalties_->size()),
                        next_copy->reservations,
                        next_copy->time_f,
                        next_t);
            }
        }
#endif
    }
    while ((move_interval = move_interval->next));
}

template<Bool feasible, Bool towards_end>
void SharedTimeIntervalAStar::generate_waypoint(Label<feasible>* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Check.
    static_assert(!towards_end);

    // Get the input data.
    const auto& intervals = *intervals_;
    const auto& once_off_penalties = *once_off_penalties_;

    // Get the label data.
    const auto current_nt = current->nt;
    debug_assert(current_nt.n == (*waypoints_)[next_waypoint_index_].n);
    const auto next_n = current_nt.n;

    // Calculate the distance to the next waypoint and target.
    const auto next_waypoint_index = towards_end ? 0 : next_waypoint_index_;
    debug_assert(h_to_waypoint_[next_n] == 0);
    const auto waypoint_t = waypoint_time_;

    // Calculate the distance to the target.
    const auto h_waypoint_to_target = towards_end ? 0 : h_waypoint_to_target_[next_waypoint_index];
    debug_assert(0 <= h_waypoint_to_target && h_waypoint_to_target < TIME_MAX);

    // Skip intervals earlier than the current time.
    const Interval* wait_interval;
    {
        // ZoneScopedNC("Skip to earliest wait interval", TRACY_COLOUR);

        wait_interval = intervals.get_intervals(current_nt.n, Direction::WAIT);
        while (current_nt.t >= wait_interval->end)
        {
            wait_interval = wait_interval->next;
        }
    }

    // Expand along wait intervals to the waypoint time.
    Time wait_end = current_nt.t;
    Cost wait_cost = 0.0;
    {
        // ZoneScopedNC("Expand one interval", TRACY_COLOUR);

        // Calculate the transition cost.
#ifdef DEBUG
        Cost check_wait_cost = 0.0;
        {
            auto wait_interval = intervals.get_intervals(next_n, Direction::WAIT);
            do
            {
                auto wait_duration = std::min(wait_interval->end, std::max(current_nt.t, waypoint_t)) -
                    std::max(current_nt.t, wait_interval->start);
                wait_duration = std::max(wait_duration, 0);
                check_wait_cost += wait_duration == 0 ? 0.0 : wait_duration * (default_edge_cost<feasible>() + wait_interval->cost);
            }
            while ((wait_interval = wait_interval->next));
        }
#endif
        {
            // ZoneScopedNC("Calculate wait cost", TRACY_COLOUR);
            do
            {
                const auto wait_start = wait_end;
                debug_assert(wait_interval->start <= wait_start);
                wait_end = std::min(std::max(wait_end, waypoint_t), wait_interval->end);
                const auto duration_in_wait_interval = wait_end - wait_start;
                debug_assert(wait_start >= 0);
                debug_assert(wait_end >= 0);
                debug_assert(duration_in_wait_interval >= 0);
                if (duration_in_wait_interval > 0 && wait_interval->cost == INF)
                {
                    return;
                }
                wait_cost += duration_in_wait_interval == 0 ?
                             0.0 :
                             duration_in_wait_interval * (default_edge_cost<feasible>() + wait_interval->cost);
                debug_assert(!std::isnan(wait_cost));
            }
            while (wait_end >= wait_interval->end && (wait_interval = wait_interval->next));
            debug_assert(is_eq(wait_cost, check_wait_cost));
        }

        // Get the next nodetime.
        const auto next_t = wait_end;
        debug_assert(next_t == waypoint_t);
        const NodeTime next_nt{next_n, next_t};

        // Calculate the earliest arrival time.
        constexpr auto h_to_waypoint_time = 0;
        const auto h_to_target = h_to_waypoint_time + h_waypoint_to_target;
        debug_assert(h_to_target >= earliest_target_time_ - next_t);
        const auto next_time_f = next_t + h_to_target;

        // Exit if time infeasible.
        if (next_time_f > latest_target_time_)
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("        Time infeasible at nt {}, n {}, xyt {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_));
            }
#endif

            // Exit.
            return;
        }

        // Allocate memory for the label.
        auto next = static_cast<Label<feasible>*>(label_storage_.get_buffer<false, false>());
        std::memcpy(next, current, label_storage_.object_size());

        // Calculate the g value.
        next->g += wait_cost;
        {
            auto once_off_bitset = get_once_off_bitset<feasible>(next);
            for (Size index = 0; index < once_off_penalties.size(); ++index)
                if (!get_bitset(once_off_bitset, index))
                {
                    // Get the penalty data.
                    const auto& [cost, nt, d] = once_off_penalties[index];

                    // Accumulate the penalty.
                    const auto crossed = (d == OnceOffDirection::GEq) ?
                        (nt.n == next_n && nt.t <= next_t) :
                        (nt.n == next_n && nt.t >= next_t);
                    if (crossed)
                    {
                        ++next->num_bitset_ones;
                        set_bitset(once_off_bitset, index);
                        next->g += cost;
                    }
                }
        }
        debug_assert(is_ge(next->g, current->g));

        // Calculate the f value.
        if constexpr (feasible)
        {
            // Calculate the h value.
            const auto h_target_to_end = intervals.get_finish_time_h(next_time_f);
            // This h for feasible master problems is stronger than in LPA* which doesn't support varying h at the moment.
            const auto next_h = h_to_target + h_target_to_end;

            // Update the f value.
            next->f = next->g + next_h;
            debug_assert(next->f == next->g || is_ge(next->f, next->g));
        }
        else
        {
            next->time_f = next_time_f;
        }

        // Exit if cost infeasible.
        Cost cost_f;
        if constexpr (feasible) { cost_f = next->f; } else { cost_f = next->g; }
        if (is_ge(cost_f, 0.0))
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                if constexpr (feasible)
                {
                    println("        Cost infeasible nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                            next_nt.id,
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->f,
                            next->g,
                            format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                            format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                            next->reservations);
                }
                else
                {
                    println("        Cost infeasible nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                            next_nt.id,
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                            format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                            next->reservations,
                            next->time_f,
                            next_t);
                }
            }
#endif

            // Exit.
            return;
        }

        // Calculate the number of reservations.
#ifdef USE_RESERVATION_LOOKUP
        if constexpr (feasible)
        {
            // ZoneScopedNC("Calculate reservations", TRACY_COLOUR);

            for (Time t = current->nt.t + 1; t <= next_t; ++t)
            {
                const NodeTime nt{current_nt.n, t};
                next->reservations += calculate_reservation(nt);
            }
        }
#endif

        // Push into the priority queue.
        next->nt = next_nt;
        next->parent = current;
#ifdef DEBUG
        auto next_copy = next;
#endif
        next = push<feasible, true>(next);

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                println("        {} nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->f,
                        next_copy->g,
                        format_bitset(get_once_off_bitset<feasible>(next_copy), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next_copy), rectangle_penalties_->size()),
                        next_copy->reservations);
            }
            else
            {
                println("        {} nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset<feasible>(next_copy), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next_copy), rectangle_penalties_->size()),
                        next_copy->reservations,
                        next_copy->time_f,
                        next_t);
            }
        }
#endif
    }
}

template<Bool feasible>
void SharedTimeIntervalAStar::generate_end(Label<feasible>* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the input data.
    const auto& intervals = *intervals_;

    // Get the label data.
    const auto current_nt = current->nt;

    // Skip intervals earlier than the current time.
    const Interval* move_interval;
    {
        // ZoneScopedNC("Skip to earliest move interval", TRACY_COLOUR);

        move_interval = intervals.get_end_intervals();
        while (current_nt.t >= move_interval->end)
        {
            move_interval = move_interval->next;
        }
    }
    const Interval* wait_interval;
    {
        // ZoneScopedNC("Skip to earliest wait interval", TRACY_COLOUR);

        wait_interval = intervals.get_intervals(target_, Direction::WAIT);
        while (current_nt.t >= wait_interval->end)
        {
            wait_interval = wait_interval->next;
        }
    }

    // Expand using each move interval.
    Time wait_end = current_nt.t;
    Cost wait_cost = 0.0;
    do
    {
        // ZoneScopedNC("Expand one interval", TRACY_COLOUR);

        // Calculate the transition cost.
#ifdef DEBUG
        Cost check_wait_cost = 0.0;
        {
            auto wait_interval = intervals.get_intervals(target_, Direction::WAIT);
            do
            {
                auto wait_duration = std::min(wait_interval->end, std::max(current_nt.t, move_interval->start)) -
                    std::max(current_nt.t, wait_interval->start);
                wait_duration = std::max(wait_duration, 0);
                check_wait_cost += wait_duration == 0 ? 0.0 : wait_duration * (default_edge_cost<feasible>() + wait_interval->cost);
            }
            while ((wait_interval = wait_interval->next));
        }
#endif
        {
            // ZoneScopedNC("Calculate wait cost", TRACY_COLOUR);
            do
            {
                const auto wait_start = wait_end;
                debug_assert(wait_interval->start <= wait_start);
                wait_end = std::min(std::max(wait_end, move_interval->start), wait_interval->end);
                const auto duration_in_wait_interval = wait_end - wait_start;
                debug_assert(wait_start >= 0);
                debug_assert(wait_end >= 0);
                debug_assert(duration_in_wait_interval >= 0);
                if (duration_in_wait_interval > 0 && wait_interval->cost == INF)
                {
                    return;
                }
                wait_cost += duration_in_wait_interval == 0 ?
                             0.0 :
                             duration_in_wait_interval * (default_edge_cost<feasible>() + wait_interval->cost);
                debug_assert(!std::isnan(wait_cost));
            }
            while (wait_end >= wait_interval->end && (wait_interval = wait_interval->next));
            debug_assert(is_eq(wait_cost, check_wait_cost));
        }

        // Get the next nodetime.
        const auto next_t = wait_end;
        const NodeTime next_nt{end_n(), next_t};

        // Exit if time infeasible.
        if (next_t > latest_target_time_)
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                println("        Time infeasible at nt {}, n {}, xyt {}",
                        next_nt.id,
                        next_nt.n,
                        format_nodetime(next_nt, map_));
            }
#endif

            // Exit.
            return;
        }

        // Allocate memory for the label.
        auto next = static_cast<Label<feasible>*>(label_storage_.get_buffer<false, false>());
        std::memcpy(next, current, label_storage_.object_size());

        // Calculate the g value.
        next->g += wait_cost + move_interval->cost;
        debug_assert(is_ge(next->g, current->g));

        // Exit if cost infeasible.
        if (is_ge(next->g, 0.0))
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                if constexpr (feasible)
                {
                    println("        Cost infeasible at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}",
                            next_nt.id,
                            next_nt.n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                            format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()));
                }
                else
                {
                    println("        Cost infeasible at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time g {}",
                            next_nt.id,
                            next_nt.n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset<feasible>(next), once_off_penalties_->size()),
                            format_bitset(get_rectangle_bitset<feasible>(next), rectangle_penalties_->size()),
                            next_t);
                }
            }
#endif

            // Proceed to the next interval.
            continue;
        }
        debug_assert(next_t >= earliest_target_time_);

        // Calculate the f value.
        if constexpr (feasible)
        {
            next->f = next->g;
        }
        else
        {
            next->time_f = next_t;
        }

        // Push into the priority queue.
        next->nt = next_nt;
        next->parent = current;
#ifdef DEBUG
        auto next_copy = next;
#endif
        next = push<feasible, false>(next);

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                println("        {} at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id,
                        next_nt.n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset<feasible>(next_copy), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next_copy), rectangle_penalties_->size()));
            }
            else
            {
                println("        {} at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time g {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id,
                        next_nt.n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset<feasible>(next_copy), once_off_penalties_->size()),
                        format_bitset(get_rectangle_bitset<feasible>(next_copy), rectangle_penalties_->size()),
                        next_t);
            }
        }
#endif

        // Add the new path to the master problem.
        if (next)
        {
            add_path<feasible>(next);
        }
    }
    while ((move_interval = move_interval->next));
}

template<Bool feasible, Bool towards_end>
void SharedTimeIntervalAStar::expand(Label<feasible>* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the input data.
    const auto& waypoints = *waypoints_;

    // Push to four directions.
    const auto current_nt = current->nt;
    const auto current_n = current_nt.n;
    if (const auto next_n = map_.get_north(current_n); map_[next_n])
    {
        generate_nodetimes<feasible, towards_end>(next_n, Direction::NORTH, current);
    }
    if (const auto next_n = map_.get_south(current_n); map_[next_n])
    {
        generate_nodetimes<feasible, towards_end>(next_n, Direction::SOUTH, current);
    }
    if (const auto next_n = map_.get_west(current_n); map_[next_n])
    {
        generate_nodetimes<feasible, towards_end>(next_n, Direction::WEST, current);
    }
    if (const auto next_n = map_.get_east(current_n); map_[next_n])
    {
        generate_nodetimes<feasible, towards_end>(next_n, Direction::EAST, current);
    }

    // Push to the waypoint if at the correct location.
    if constexpr (!towards_end)
        if (current_n == waypoints[next_waypoint_index_].n)
        {
            generate_waypoint<feasible, towards_end>(current);
        }

    // Push to the end if at the correct location.
    if constexpr (towards_end)
        if (current_n == target_)
        {
            generate_end<feasible>(current);
        }
}

#ifdef USE_RESERVATION_LOOKUP
Bool SharedTimeIntervalAStar::calculate_reservation(const NodeTime nt)
{
    const auto& projection = problem_.projection();
    const auto total_use = projection.find_summed_nodetime(nt);
    const auto individual_use = projection.find_agent_nodetime(a_, nt);
    const auto other_use = total_use - individual_use;
    const auto reservation = is_gt(other_use, 0.0);
    return reservation;
}
#endif

// static inline Bool approx_dominates(const Cost lhs_potential_g, const Cost rhs_potential_g,
//                                     const UInt32 lhs_reservations, const UInt32 rhs_reservations)
// {
//     return (is_lt(lhs_potential_g, rhs_potential_g)) ||
//            (is_eq(lhs_potential_g, rhs_potential_g) && is_lt(lhs_reservations, rhs_reservations));
// }
// static inline Bool dominates(const Cost lhs_potential_g, const Cost rhs_potential_g,
//                              const UInt32 lhs_reservations, const UInt32 rhs_reservations)
// {
//     return (lhs_potential_g <  rhs_potential_g) ||
//            (lhs_potential_g == rhs_potential_g && lhs_reservations < rhs_reservations);
// }

template<Bool feasible, Bool is_wait_action>
SharedTimeIntervalAStar::Label<feasible>* SharedTimeIntervalAStar::push(Label<feasible>* next)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the new label.
    const auto nt = next->nt;
    const auto next_t = nt.t;

    // Check ordering.
#ifdef DEBUG
    for (auto current = static_cast<Label<feasible>*>(closed_[nt.n]);
         current && current->next;
         current = current->next)
    {
        debug_assert(current->nt.t <= current->next->nt.t);
    }
#endif

    // Get the input data.
    const auto& intervals = *intervals_;
    const auto& once_off_penalties = *once_off_penalties_;
    const auto& rectangle_penalties = *rectangle_penalties_;

    // Get the intervals.
    const auto wait_intervals = intervals.get_intervals(nt.n, Direction::WAIT);

    // Loop through existing labels with the same or earlier time to determine if the new label is dominated.
    const auto next_once_off_bitset = get_once_off_bitset<feasible>(next);
    const auto next_rectangle_bitset = get_rectangle_bitset<feasible>(next);
    auto first = reinterpret_cast<Label<feasible>**>(&closed_[nt.n]);
    auto insert_prev_next = first;
    const auto current = next->parent;
    for (auto existing = *insert_prev_next; existing && existing->nt.t <= nt.t; existing = *insert_prev_next)
    {
        if (!is_wait_action || existing != current)
        {
            // Get the existing label.
            const auto existing_g = existing->g;
            const auto existing_t = existing->nt.t;

            // Calculate the potential cost of the existing label if it incurred the same penalties as the new label.
            // If the existing label still costs less than or equal to the new label, even after incurring these
            // penalties, then the new label is dominated.
            const auto existing_once_off_bitset = get_once_off_bitset<feasible>(existing);
            const auto existing_rectangle_bitset = get_rectangle_bitset<feasible>(existing);
            auto existing_potential_g = existing_g;
            for (Size index = 0; index < once_off_penalties.size(); ++index)
            {
                const auto existing_not_paid =
                    (get_bitset(next_once_off_bitset, index) > get_bitset(existing_once_off_bitset, index));
                existing_potential_g += once_off_penalties[index].cost * existing_not_paid;
            }
            for (Size index = 0; index < rectangle_penalties.size(); ++index)
            {
                const auto existing_not_paid =
                    (get_bitset(existing_rectangle_bitset, index) > get_bitset(next_rectangle_bitset, index));
                existing_potential_g += rectangle_penalties[index].cost * existing_not_paid;
            }

            // Calculate the cost of waiting until the new arrival time.
            {
                auto wait_interval = wait_intervals;
                do
                {
                    const auto wait_start = std::max(existing_t, wait_interval->start);
                    const auto wait_end = std::min(next_t, wait_interval->end);
                    const auto duration_in_wait_interval = std::max(wait_end - wait_start, 0);
                    debug_assert(wait_start >= 0);
                    debug_assert(wait_end >= 0);
                    debug_assert(duration_in_wait_interval >= 0);
                    existing_potential_g += duration_in_wait_interval == 0 ?
                                            0.0 :
                                            duration_in_wait_interval * (default_edge_cost<feasible>() + wait_interval->cost);
                }
                while (wait_interval->next && (wait_interval = wait_interval->next)->start < next_t);
            }

            // Discard the new label if dominated.
            if (is_le(existing_potential_g, next->g))
            {
                return nullptr;
            }
        }

        // Update where to insert the new label.
        insert_prev_next = &existing->next;
    }

    // Loop through existing labels with the same or later time to determine if they are dominated.
    {
        auto prev_next = first;
        auto existing = *prev_next;
        for (; existing && existing->nt.t < nt.t; existing = *prev_next)
        {
            prev_next = &existing->next;
        }
        for (; existing; existing = *prev_next)
        {
            // Get the existing label.
            const auto existing_g = existing->g;
            const auto existing_t = existing->nt.t;
            debug_assert(existing_t >= next_t);

            // Make the same calculation in the other direction. The new label could be better than the existing label if
            // the new label visited a superset of the once-off penalties visited by the existing label.
            const auto existing_once_off_bitset = get_once_off_bitset<feasible>(existing);
            const auto existing_rectangle_bitset = get_rectangle_bitset<feasible>(existing);
            auto next_potential_g = next->g;
            for (Size index = 0; index < once_off_penalties.size(); ++index)
            {
                const auto next_not_paid =
                    (get_bitset(existing_once_off_bitset, index) > get_bitset(next_once_off_bitset, index));
                next_potential_g += once_off_penalties[index].cost * next_not_paid;
            }
            for (Size index = 0; index < rectangle_penalties.size(); ++index)
            {
                const auto next_not_paid =
                    (get_bitset(next_rectangle_bitset, index) > get_bitset(existing_rectangle_bitset, index));
                next_potential_g += rectangle_penalties[index].cost * next_not_paid;
            }

            // Calculate the cost of waiting until the new arrival time.
            {
                auto wait_interval = wait_intervals;
                do
                {
                    const auto wait_start = std::max(next_t, wait_interval->start);
                    const auto wait_end = std::min(existing_t, wait_interval->end);
                    const auto duration_in_wait_interval = std::max(wait_end - wait_start, 0);
                    debug_assert(wait_start >= 0);
                    debug_assert(wait_end >= 0);
                    debug_assert(duration_in_wait_interval >= 0);
                    next_potential_g += duration_in_wait_interval == 0 ?
                                        0.0 :
                                        duration_in_wait_interval * (default_edge_cost<feasible>() + wait_interval->cost);
                }
                while (wait_interval->next && (wait_interval = wait_interval->next)->start < existing_t);
            }

            // Delete the existing label if dominated.
            if (is_le(next_potential_g, existing_g))
            {
                // Remove from the priority queue.
                if (existing->pq_index >= 0)
                {
                    open<feasible>().erase(existing->pq_index);
                    debug_assert(existing->pq_index == -1);
                }

                // Delete the existing label from future dominance checks.
                *prev_next = existing->next;
                if (insert_prev_next == &existing->next)
                {
                    insert_prev_next = prev_next;
                }
            }
            else
            {
                // Advance to the next label.
                prev_next = &existing->next;
            }
        }
    }

    // Link in the new label.
    debug_assert(insert_prev_next);
    next->next = *insert_prev_next;
    *insert_prev_next = next;

    // Check ordering.
#ifdef DEBUG
    for (auto current = static_cast<Label<feasible>*>(closed_[nt.n]); current && current->next; current = current->next)
    {
        debug_assert(current->nt.t <= current->next->nt.t);
    }
#endif

    // Store the new label.
    label_storage_.commit_buffer();
    open<feasible>().push(next);
    debug_assert(next->pq_index >= 0);

    // Stored the new label.
    return next;
}

template<Bool feasible>
void SharedTimeIntervalAStar::add_path(Label<feasible>* end)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(is_end(end->nt));
    const auto reduced_cost = end->g;
    if (is_lt(reduced_cost, 0.0))
    {
        // Retrieve the path.
        auto variable_ptr = Variable::construct(a_, end->nt.t + 1);
        auto path = variable_ptr->path();
        const Label<feasible>* last;
        {
            // ZoneScopedNC("Get path", TRACY_COLOUR);

            last = end->parent;
            NodeTime nt{last->nt.n, end->nt.t};
            debug_assert(nt.n == target_);
            path[nt.t] = Edge{nt.n, Direction::INVALID};
            for (auto label = (end->nt.t == last->nt.t ? last->parent : last); label; label = label->parent)
            {
                const auto prev_nt = label->nt;
                path[nt.t - 1] = Edge{prev_nt.n, map_.get_direction(prev_nt.n, nt.n)};
                for (Time t = nt.t - 2; t >= prev_nt.t; --t)
                {
                    path[t] = Edge{prev_nt.n, Direction::WAIT};
                }
                nt = prev_nt;
            }
        }

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("            Time-interval A* found path of length {}, reduced cost {}: {}",
                    path.size(), reduced_cost, format_path_with_time(path, map_));
        }
#endif

        // Check.
#ifdef DEBUG
        debug_assert(path.size() == end->nt.t + 1);
        debug_assert(path.front().n == start_.n);
        debug_assert(path.back().n == target_);
        debug_assert(path.back().n == last->nt.n);
        for (auto label = last; label; label = label->parent)
        {
            debug_assert(path[label->nt.t].n == label->nt.n);
            debug_assert(!label->parent || path[label->nt.t - 1].n == label->parent->nt.n);
        }
        for (Time t = 0; t < path.size() - 1; ++t)
        {
            debug_assert(map_[path[t].n]);
            debug_assert(map_.get_destination(path[t]) == path[t + 1].n);

            const auto [x1, y1] = map_.get_xy(path[t].n);
            const auto [x2, y2] = map_.get_xy(path[t + 1].n);
            debug_assert(std::abs(x2 - x1) + std::abs(y2 - y2) <= 1);
        }
#endif

        // Add the column.
#ifdef USE_SHARED_TIME_INTERVAL_ASTAR_PRICER
        problem_.master().add_column(std::move(variable_ptr), reduced_cost);
        ++num_added_;
#endif

        // Store the best found objective value.
        obj_ = std::min(obj_, reduced_cost);
    }
}

template<Bool feasible>
Cost SharedTimeIntervalAStar::solve(const Agent a,
                                    Vector<NodeTime>& waypoints,
                                    const Float constant,
                                    const SharedIntervals& intervals,
                                    OnceOffPenalties& once_off_penalties,
                                    RectanglePenalties& rectangle_penalties)
{
    ZoneScopedC(TRACY_COLOUR);

    // Store the input data.
    a_ = a;
    start_ = NodeTime{instance_.agents[a].start, 0};
    target_ = instance_.agents[a].target;
    waypoints_ = &waypoints;
    constant_ = constant;
    intervals_ = &intervals;
    once_off_penalties_ = &once_off_penalties;
    rectangle_penalties_ = &rectangle_penalties;

    // Print iteration.
    ZoneValue(iter);
#ifdef DEBUG
    if (verbose)
    {
        print_separator();
        String str;
        for (const auto nt : waypoints)
        {
            if (str.empty())
            {
                str += " via " ;
            }
            else
            {
                str += ", ";
            }
            str += format_nodetime(nt, map_);
        }
        println("Running time-interval A* for agent {} from {} to {}{} in iteration {}",
                a_, format_node(start_.n, map_), format_node(target_, map_), str, iter);
        println("");
    }
#endif

    // Clear objective value from the previous iteration.
    obj_ = 0.0;

    // Skip if the agent cannot have negative reduced cost.
    if (is_ge(constant_, 0.0))
    {
        goto FINISHED;
    }

    // Compute the earliest and latest target time.
    {
        // ZoneScopedNC("Compute the earliest and latest target time", TRACY_COLOUR);

        earliest_target_time_ = -1;
        auto interval = intervals.get_end_intervals();
        do
        {
            if (interval->cost < INF)
            {
                earliest_target_time_ = interval->start;
                break;
            }
        }
        while ((interval = interval->next));
        if (earliest_target_time_ < 0)
        {
            goto FINISHED;
        }

        latest_target_time_ = -1;
        for (; interval; interval = interval->next)
            if (interval->cost < INF)
            {
                latest_target_time_ = interval->end - 1;
            }
        if (latest_target_time_ < 0)
        {
            goto FINISHED;
        }
    }

    // Compute the minimum time between each waypoint.
    {
        // ZoneScopedNC("Compute the minimum time between each waypoint", TRACY_COLOUR);

        // Sort the waypoints by time and insert the target at the back.
        std::sort(waypoints.begin(),
                  waypoints.end(),
                  [](const NodeTime a, const NodeTime b) { return a.t < b.t; });
        waypoints.emplace_back(target_, earliest_target_time_);

        // Calculate the lower bound from each waypoint to the next waypoint.
        h_waypoint_to_target_.resize(waypoints.size());
        h_waypoint_to_target_.back() = 0;
        for (Size w = waypoints.size() - 2; w >= 0; --w)
        {
            const auto h = distance_heuristic_.get_h(waypoints[w + 1].n)[waypoints[w].n];
            const auto time_diff = waypoints[w + 1].t - waypoints[w].t;
            if (w != waypoints.size() - 2 && time_diff < h)
            {
                goto FINISHED;
            }
            h_waypoint_to_target_[w] = std::max(h, time_diff) + h_waypoint_to_target_[w + 1];
        }
    }

    // Prepare data structures.
    {
        ZoneScopedNC("Prepare data structures", TRACY_COLOUR);

        // Calculate the bytes of the bitsets. Round up to the next 8 and divide by 8.
        once_off_bitset_size_ = ((once_off_penalties.size() + 7) & (-8)) / 8;
        rectangle_bitset_size_ = ((rectangle_penalties.size() + 7) & (-8)) / 8;

        // Clear data structures from the previous iteration.
        label_storage_.reset(Label<feasible>::data_size + once_off_bitset_size_ + rectangle_bitset_size_);
        std::memset(closed_ - 1, 0, (map_.size() + 1) * sizeof(void*));
        open<feasible>().clear();
    }

    // Print penalties.
#ifdef DEBUG
    if (verbose) { println("Constant: {:.4f}\n", constant_); }
    if (verbose) { intervals.print(); }
    if (verbose) { once_off_penalties.print(map_); }
    if (verbose) { rectangle_penalties.print(map_); }
#endif

    // Solve.
    {
        ZoneScopedNC("Solve", TRACY_COLOUR);

        next_waypoint_index_ = 0;
        auto towards_end = (next_waypoint_index_ == waypoints.size() - 1);
        NodeTime next_waypoint = waypoints[next_waypoint_index_];
        waypoint_time_ = next_waypoint.t;
        h_to_waypoint_ = distance_heuristic_.get_h(next_waypoint.n);
        if (towards_end)
        {
            generate_start<feasible, true>();
        }
        else
        {
            generate_start<feasible, false>();
        }
        while (!open<feasible>().empty() && !towards_end)
        {
            ZoneScopedNC("Solving to one waypoint", TRACY_COLOUR);

#ifdef DEBUG
            if (verbose) { println("Searching to waypoint {}", format_nodetime(next_waypoint, map_)); }
#endif
            while (!open<feasible>().empty())
            {
                // Exit if terminated.
                // Problem::stop_if_terminated();

                // Pop the first item off the priority queue.
                auto current = open<feasible>().pop();
                const auto nt = current->nt;
                debug_assert(nt.t <= next_waypoint.t);

                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    if constexpr (feasible)
                    {
                        println("    Popped nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                                nt.id,
                                nt.n,
                                format_nodetime(nt, map_),
                                current->f,
                                current->g,
                                format_bitset(get_once_off_bitset<feasible>(current), once_off_penalties_->size()),
                                format_bitset(get_rectangle_bitset<feasible>(current), rectangle_penalties_->size()),
                                current->reservations);
                    }
                    else
                    {
                        println("    Popped nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                                nt.id,
                                nt.n,
                                format_nodetime(nt, map_),
                                current->g,
                                format_bitset(get_once_off_bitset<feasible>(current), once_off_penalties_->size()),
                                format_bitset(get_rectangle_bitset<feasible>(current), rectangle_penalties_->size()),
                                current->reservations,
                                current->time_f,
                                current->nt.t);
                    }
                }
#endif

                // Exit if reached the waypoint or the end.
                if (nt == next_waypoint)
                {
                    // Clear closed set and open set.
                    std::memset(closed_ - 1, 0, (map_.size() + 1) * sizeof(void*));
                    open<feasible>().clear();

                    // Insert the nodetime back into the priority queue.
                    push<feasible, false>(current);

                    // Advance to the next waypoint.
                    ++next_waypoint_index_;
                    towards_end = (next_waypoint_index_ == waypoints.size() - 1);
                    next_waypoint = waypoints[next_waypoint_index_];
                    waypoint_time_ = next_waypoint.t;
                    h_to_waypoint_ = distance_heuristic_.get_h(next_waypoint.n);
                    break;
                }

                // Expand to neighbours.
                expand<feasible, false>(current);
            }
        }
        if (!open<feasible>().empty() && towards_end)
        {
            ZoneScopedNC("Solving to one waypoint", TRACY_COLOUR);

#ifdef DEBUG
            if (verbose) { println("Searching to waypoint {}", format_nodetime(next_waypoint, map_)); }
#endif
            while (!open<feasible>().empty())
            {
                // Exit if terminated.
                // Problem::stop_if_terminated();

                // Pop the first item off the priority queue.
                auto current = open<feasible>().pop();
                const auto nt = current->nt;
                debug_assert(nt.t <= latest_target_time_);

                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    if constexpr (feasible)
                    {
                        println("    Popped nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, reservations {}",
                                nt.id,
                                nt.n,
                                format_nodetime(nt, map_),
                                current->f,
                                current->g,
                                format_bitset(get_once_off_bitset<feasible>(current), once_off_penalties_->size()),
                                format_bitset(get_rectangle_bitset<feasible>(current), rectangle_penalties_->size()),
                                current->reservations);
                    }
                    else
                    {
                        println("    Popped nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, reservations {}, time f {}, time g {}",
                                nt.id,
                                nt.n,
                                format_nodetime(nt, map_),
                                current->g,
                                format_bitset(get_once_off_bitset<feasible>(current), once_off_penalties_->size()),
                                format_bitset(get_rectangle_bitset<feasible>(current), rectangle_penalties_->size()),
                                current->reservations,
                                current->time_f,
                                current->nt.t);
                    }
                }
#endif

                // Exit if reached the waypoint or the end.
                if (is_end(nt))
                {
                    // Found an optimal path.
                    break;
                }

                // Expand to neighbours.
                expand<feasible, true>(current);
            }
        }
    }
    FINISHED:

    // Check optimal value against results from A*.
// #ifdef CHECK_USING_ASTAR
//     debug_assert(std::isnan(astar_obj) || is_eq(obj_, astar_obj));
// #endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (obj_ >= 0.0) { println("Time-interval A* failed to find a feasible path"); }
    }
#endif

    // Next iteration.
#if defined(DEBUG) || defined(TRACY_ENABLE)
    ++iter;
#endif

    // Clear data structures before setting up the next iteration.
    reset();

    // Return the optimal value if available.
    return obj_;
}
template Cost SharedTimeIntervalAStar::solve<true>(const Agent a,
                                                   Vector<NodeTime>& waypoints,
                                                   const Float constant,
                                                   const SharedIntervals& intervals,
                                                   OnceOffPenalties& once_off_penalties,
                                                   RectanglePenalties& rectangle_penalties);
template Cost SharedTimeIntervalAStar::solve<false>(const Agent a,
                                                    Vector<NodeTime>& waypoints,
                                                    const Float constant,
                                                    const SharedIntervals& intervals,
                                                    OnceOffPenalties& once_off_penalties,
                                                    RectanglePenalties& rectangle_penalties);

#ifdef DEBUG
void SharedTimeIntervalAStar::set_verbose(const Bool value)
{
    verbose = value;
}
#endif
