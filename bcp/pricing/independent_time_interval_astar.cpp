/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#ifdef USE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER

// #define PRINT_DEBUG

#include "pricing/independent_time_interval_astar.h"
#include "master/master.h"
#include "output/formatting.h"
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

template <Bool feasible>
IndependentTimeIntervalAStar<feasible>::IndependentTimeIntervalAStar(
    const Instance& instance, Problem& problem, DistanceHeuristic& distance_heuristic,
    const Agent a, Size64& num_added) :
    instance_(instance),
    map_(instance_.map),
    problem_(problem),
    distance_heuristic_(distance_heuristic),

    a_(a),
    start_({instance_.agents[a].start, 0}),
    target_(instance_.agents[a].target),

    waypoints_(),
    h_waypoint_to_target_(),
    next_waypoint_index_(0),
    waypoint_time_(),
    h_to_waypoint_(nullptr),

    constant_(),
    intervals_(map_),
    once_off_penalties_(),
    rectangle_penalties_(),
    once_off_bitset_size_(0),
    rectangle_bitset_size_(0),
    earliest_target_time_(0),
    latest_target_time_(TIME_MAX),

    label_storage_(),
    closed_(nullptr),
    open_(),
    obj_()
#ifdef USE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER
    ,
    num_added_(num_added)
#endif

#ifdef CHECK_USING_ASTAR
    ,
    astar_(instance, problem, distance_heuristic, a, num_added)
#endif
{
    ZoneScopedC(TRACY_COLOUR);

    // Allocate memory for the closed set.
    {
        ZoneScopedNC("Allocate closed set", TRACY_COLOUR);

        closed_ = static_cast<Label**>(std::malloc((map_.size() + 1) * sizeof(Label*)));
        ++closed_;
    }

    // Reset data structures.
    reset();
}

#pragma GCC diagnostic pop

template <Bool feasible>
Byte* IndependentTimeIntervalAStar<feasible>::get_once_off_bitset(Label* const label)
{
    return label->bitset;
}

template <Bool feasible>
Byte* IndependentTimeIntervalAStar<feasible>::get_rectangle_bitset(Label* const label)
{
    return label->bitset + once_off_bitset_size_;
}

template <Bool feasible>
IndependentTimeIntervalAStar<feasible>::~IndependentTimeIntervalAStar()
{
    ZoneScopedC(TRACY_COLOUR);

    --closed_;
    std::free(closed_);
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::reset()
{
    ZoneScopedC(TRACY_COLOUR);

    constant_ = 0.0;
    waypoints_.clear();
    intervals_.clear();
    once_off_penalties_.clear();
    rectangle_penalties_.clear();
    // earliest_target_time_ and latest_target_time_ are computed, whereas they are directly
    // inputted in time-expanded A*
}

template <Bool feasible>
template <Bool towards_end>
void IndependentTimeIntervalAStar<feasible>::generate_start()
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the start nodetime.
    const NodeTime next_nt = start_;
    const auto& next_n = next_nt.n;
    constexpr Time next_t = 0;
    DEBUG_ASSERT(map_[next_n]);

    // Calculate the distance to the next waypoint.
    constexpr Size64 next_waypoint_index = 0;
    const auto h_to_waypoint = h_to_waypoint_[next_n];
    DEBUG_ASSERT(0 <= h_to_waypoint && h_to_waypoint < TIME_MAX);
    const auto waypoint_t = waypoint_time_;
    const auto h_to_waypoint_time = std::max(h_to_waypoint, waypoint_t - next_t);

    // Calculate the distance to the target.
    const auto h_waypoint_to_target = towards_end ? 0 : h_waypoint_to_target_[next_waypoint_index];
    DEBUG_ASSERT(0 <= h_waypoint_to_target && h_waypoint_to_target < TIME_MAX);
    const auto h_to_target = h_to_waypoint_time + h_waypoint_to_target;
    DEBUG_ASSERT(h_to_target >= earliest_target_time_ - next_t);

    // Calculate the earliest arrival time.
    const auto next_time_f = next_t + h_to_target;

    // Exit if time infeasible.
    if ((!towards_end && next_t + h_to_waypoint > waypoint_t) || next_time_f > latest_target_time_)
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            PRINTLN("        Time infeasible at start nt {}, n {}, xyt {}",
                    next_nt.id(),
                    next_n,
                    format_nodetime(next_nt, map_));
        }
#endif

        // Exit.
        return;
    }

    // Allocate memory for the label.
    auto next = new (label_storage_.get_buffer<false, true>()) Label;

    // Calculate the g value.
    next->g = constant_;

    // Calculate the f value.
    if constexpr (feasible)
    {
        // Calculate the h value.
        const auto h_target_to_end = intervals_.get_finish_time_h(next_time_f);
        // This h for feasible master problems is stronger than in LPA* which doesn't support
        // varying h at the moment.
        const auto next_h = h_to_target + h_target_to_end;

        // Update the f value.
        next->f = next->g + next_h;
        DEBUG_ASSERT(next->f == next->g || is_ge(next->f, next->g));
    }
    else
    {
        next->time_f = next_time_f;
    }

    // Exit if cost infeasible.
    Cost cost_f;
    if constexpr (feasible)
    {
        cost_f = next->f;
    }
    else
    {
        cost_f = next->g;
    }
    if (is_ge(cost_f, 0.0))
    {
        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                PRINTLN("        Cost infeasible at start nt {}, n {}, xyt {}, f {}, g {}, "
                        "once-off {}, rectangle {}, reservations {}",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->f,
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                        next->reservations);
            }
            else
            {
                PRINTLN("        Cost infeasible at start nt {}, n {}, xyt {}, g {}, once-off {}, "
                        "rectangle {}, reservations {}, time f {}, time g {}",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
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
    next = push<false>(next);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if constexpr (feasible)
        {
            PRINTLN("        Generated start at nt {}, n {}, xyt {}, f {}, g {}, once-off {}, "
                    "rectangle {}, reservations {}",
                    next_nt.id(),
                    next_n,
                    format_nodetime(next_nt, map_),
                    next->f,
                    next->g,
                    format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                    next->reservations);
        }
        else
        {
            PRINTLN("        Generated start at nt {}, n {}, xyt {}, g {}, once-off {}, rectangle "
                    "{}, reservations {}, time f {}, time g {}",
                    next_nt.id(),
                    next_n,
                    format_nodetime(next_nt, map_),
                    next->g,
                    format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                    next->reservations,
                    next->time_f,
                    next_t);
        }
    }
#endif
}

template <Bool feasible>
template <Bool towards_end>
void IndependentTimeIntervalAStar<feasible>::generate_nodetimes(const Node next_n,
                                                                const Direction d, Label* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(map_[next_n]);

    // Get the label data.
    const auto current_nt = current->nt;

    // Calculate the distance to the next waypoint.
    const auto next_waypoint_index = towards_end ? 0 : next_waypoint_index_;
    const auto h_to_waypoint = h_to_waypoint_[next_n];
    DEBUG_ASSERT(0 <= h_to_waypoint && h_to_waypoint < TIME_MAX);
    const auto waypoint_t = waypoint_time_;

    // Calculate the distance to the target.
    const auto h_waypoint_to_target = towards_end ? 0 : h_waypoint_to_target_[next_waypoint_index];
    DEBUG_ASSERT(0 <= h_waypoint_to_target && h_waypoint_to_target < TIME_MAX);

    // Skip intervals earlier than the current time.
    const Interval* move_interval;
    {
        // ZoneScopedNC("Skip to earliest move interval", TRACY_COLOUR);

        move_interval = intervals_.get_intervals(current_nt.n, d);
        while (current_nt.t >= move_interval->end)
        {
            move_interval = move_interval->next;
        }
    }
    const Interval* wait_interval;
    {
        // ZoneScopedNC("Skip to earliest wait interval", TRACY_COLOUR);

        wait_interval = intervals_.get_intervals(current_nt.n, Direction::WAIT);
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
            auto wait_interval = intervals_.get_intervals(current_nt.n, Direction::WAIT);
            do
            {
                auto wait_duration =
                    std::min(wait_interval->end, std::max(current_nt.t, move_interval->start)) -
                    std::max(current_nt.t, wait_interval->start);
                wait_duration = std::max(wait_duration, 0);
                check_wait_cost += wait_duration == 0 ?
                                       0.0 :
                                       wait_duration * (default_edge_cost + wait_interval->cost);
            } while ((wait_interval = wait_interval->next));
        }
#endif
        {
            // ZoneScopedNC("Calculate wait cost", TRACY_COLOUR);
            do
            {
                const auto wait_start = wait_end;
                DEBUG_ASSERT(wait_interval->start <= wait_start);
                wait_end = std::min(std::max(wait_end, move_interval->start), wait_interval->end);
                const auto duration_in_wait_interval = wait_end - wait_start;
                DEBUG_ASSERT(wait_start >= 0);
                DEBUG_ASSERT(wait_end >= 0);
                DEBUG_ASSERT(duration_in_wait_interval >= 0);
                if (duration_in_wait_interval > 0 && wait_interval->cost == COST_INF)
                {
                    return;
                }
                wait_cost +=
                    duration_in_wait_interval == 0 ?
                        0.0 :
                        duration_in_wait_interval * (default_edge_cost + wait_interval->cost);
                DEBUG_ASSERT(!std::isnan(wait_cost));
            } while (wait_end >= wait_interval->end && (wait_interval = wait_interval->next));
            DEBUG_ASSERT(is_eq(wait_cost, check_wait_cost));
        }

        // Get the next nodetime.
        const auto next_t = wait_end + 1;
        const NodeTime next_nt{next_n, next_t};

        // Calculate the earliest arrival time.
        const auto h_to_waypoint_time = std::max(h_to_waypoint, waypoint_t - next_t);
        const auto h_to_target = h_to_waypoint_time + h_waypoint_to_target;
        DEBUG_ASSERT(h_to_target >= earliest_target_time_ - next_t);
        const auto next_time_f = next_t + h_to_target;

        // Exit if time infeasible.
        if ((!towards_end && next_t + h_to_waypoint > waypoint_t) ||
            next_time_f > latest_target_time_)
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                PRINTLN("        Time infeasible at nt {}, n {}, xyt {}",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_));
            }
#endif

            // Exit.
            return;
        }

        // Allocate memory for the label.
        auto next = new (label_storage_.get_buffer<false, false>()) Label;
        std::memcpy(next, current, label_storage_.object_size());

        // Calculate the g value.
        next->g += wait_cost + default_edge_cost + move_interval->cost;
        {
            auto once_off_bitset = get_once_off_bitset(next);
            for (Size64 index = 0; index < once_off_penalties_.size(); ++index)
                if (!get_bitset(once_off_bitset, index))
                {
                    // Get the penalty data.
                    const auto& [cost, nt, d] = once_off_penalties_[index];

                    // Accumulate the penalty if the nodetime is crossed.
                    const auto crossed = (d == OnceOffDirection::GEq) ?
                                             ((nt.n == current_nt.n && nt.t <= next_t - 1) ||
                                              (nt.n == next_n && nt.t <= next_t)) :
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
            auto rectangle_bitset = get_rectangle_bitset(next);
            for (Size64 index = 0; index < rectangle_penalties_.size(); ++index)
            {
                // Get the penalty data.
                const auto& [cost, first_ets, length, n_increment] = rectangle_penalties_[index];

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
        DEBUG_ASSERT(is_ge(next->g, current->g));

        // Calculate the f value.
        if constexpr (feasible)
        {
            // Calculate the h value.
            const auto h_target_to_end = intervals_.get_finish_time_h(next_time_f);
            // This h for feasible master problems is stronger than in LPA* which doesn't support
            // varying h at the moment.
            const auto next_h = h_to_target + h_target_to_end;

            // Update the f value.
            next->f = next->g + next_h;
            DEBUG_ASSERT(next->f == next->g || is_ge(next->f, next->g));
        }
        else
        {
            next->time_f = next_time_f;
        }

        // Exit if cost infeasible.
        Cost cost_f;
        if constexpr (feasible)
        {
            cost_f = next->f;
        }
        else
        {
            cost_f = next->g;
        }
        if (is_ge(cost_f, 0.0))
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                if constexpr (feasible)
                {
                    PRINTLN("        Cost infeasible nt {}, n {}, xyt {}, f {}, g {}, once-off {}, "
                            "rectangle {}, reservations {}",
                            next_nt.id(),
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->f,
                            next->g,
                            format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                            next->reservations);
                }
                else
                {
                    PRINTLN("        Cost infeasible nt {}, n {}, xyt {}, g {}, once-off {}, "
                            "rectangle {}, reservations {}, time f {}, time g {}",
                            next_nt.id(),
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
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
        next = push<false>(next);

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                PRINTLN("        {} nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, "
                        "reservations {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->f,
                        next_copy->g,
                        format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()),
                        next_copy->reservations);
            }
            else
            {
                PRINTLN("        {} nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, "
                        "reservations {}, time f {}, time g {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()),
                        next_copy->reservations,
                        next_copy->time_f,
                        next_t);
            }
        }
#endif
    } while ((move_interval = move_interval->next));
}

template <Bool feasible>
template <Bool towards_end>
void IndependentTimeIntervalAStar<feasible>::generate_waypoint(Label* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Check.
    static_assert(!towards_end);

    // Get the label data.
    const auto current_nt = current->nt;
    DEBUG_ASSERT(current_nt.n == waypoints_[next_waypoint_index_].n);
    const auto next_n = current_nt.n;

    // Calculate the distance to the next waypoint and target.
    const auto next_waypoint_index = towards_end ? 0 : next_waypoint_index_;
    DEBUG_ASSERT(h_to_waypoint_[next_n] == 0);
    const auto waypoint_t = waypoint_time_;

    // Calculate the distance to the target.
    const auto h_waypoint_to_target = towards_end ? 0 : h_waypoint_to_target_[next_waypoint_index];
    DEBUG_ASSERT(0 <= h_waypoint_to_target && h_waypoint_to_target < TIME_MAX);

    // Skip intervals earlier than the current time.
    const Interval* wait_interval;
    {
        // ZoneScopedNC("Skip to earliest wait interval", TRACY_COLOUR);

        wait_interval = intervals_.get_intervals(current_nt.n, Direction::WAIT);
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
            auto wait_interval = intervals_.get_intervals(next_n, Direction::WAIT);
            do
            {
                auto wait_duration =
                    std::min(wait_interval->end, std::max(current_nt.t, waypoint_t)) -
                    std::max(current_nt.t, wait_interval->start);
                wait_duration = std::max(wait_duration, 0);
                check_wait_cost += wait_duration == 0 ?
                                       0.0 :
                                       wait_duration * (default_edge_cost + wait_interval->cost);
            } while ((wait_interval = wait_interval->next));
        }
#endif
        {
            // ZoneScopedNC("Calculate wait cost", TRACY_COLOUR);
            do
            {
                const auto wait_start = wait_end;
                DEBUG_ASSERT(wait_interval->start <= wait_start);
                wait_end = std::min(std::max(wait_end, waypoint_t), wait_interval->end);
                const auto duration_in_wait_interval = wait_end - wait_start;
                DEBUG_ASSERT(wait_start >= 0);
                DEBUG_ASSERT(wait_end >= 0);
                DEBUG_ASSERT(duration_in_wait_interval >= 0);
                if (duration_in_wait_interval > 0 && wait_interval->cost == COST_INF)
                {
                    return;
                }
                wait_cost +=
                    duration_in_wait_interval == 0 ?
                        0.0 :
                        duration_in_wait_interval * (default_edge_cost + wait_interval->cost);
                DEBUG_ASSERT(!std::isnan(wait_cost));
            } while (wait_end >= wait_interval->end && (wait_interval = wait_interval->next));
            DEBUG_ASSERT(is_eq(wait_cost, check_wait_cost));
        }

        // Get the next nodetime.
        const auto next_t = wait_end;
        DEBUG_ASSERT(next_t == waypoint_t);
        const NodeTime next_nt{next_n, next_t};

        // Calculate the earliest arrival time.
        constexpr auto h_to_waypoint_time = 0;
        const auto h_to_target = h_to_waypoint_time + h_waypoint_to_target;
        DEBUG_ASSERT(h_to_target >= earliest_target_time_ - next_t);
        const auto next_time_f = next_t + h_to_target;

        // Exit if time infeasible.
        if (next_time_f > latest_target_time_)
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                PRINTLN("        Time infeasible at nt {}, n {}, xyt {}",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_));
            }
#endif

            // Exit.
            return;
        }

        // Allocate memory for the label.
        auto next = new (label_storage_.get_buffer<false, false>()) Label;
        std::memcpy(next, current, label_storage_.object_size());

        // Calculate the g value.
        next->g += wait_cost;
        {
            auto once_off_bitset = get_once_off_bitset(next);
            for (Size64 index = 0; index < once_off_penalties_.size(); ++index)
                if (!get_bitset(once_off_bitset, index))
                {
                    // Get the penalty data.
                    const auto& [cost, nt, d] = once_off_penalties_[index];

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
        DEBUG_ASSERT(is_ge(next->g, current->g));

        // Calculate the f value.
        if constexpr (feasible)
        {
            // Calculate the h value.
            const auto h_target_to_end = intervals_.get_finish_time_h(next_time_f);
            // This h for feasible master problems is stronger than in LPA* which doesn't support
            // varying h at the moment.
            const auto next_h = h_to_target + h_target_to_end;

            // Update the f value.
            next->f = next->g + next_h;
            DEBUG_ASSERT(next->f == next->g || is_ge(next->f, next->g));
        }
        else
        {
            next->time_f = next_time_f;
        }

        // Exit if cost infeasible.
        Cost cost_f;
        if constexpr (feasible)
        {
            cost_f = next->f;
        }
        else
        {
            cost_f = next->g;
        }
        if (is_ge(cost_f, 0.0))
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                if constexpr (feasible)
                {
                    PRINTLN("        Cost infeasible nt {}, n {}, xyt {}, f {}, g {}, once-off {}, "
                            "rectangle {}, reservations {}",
                            next_nt.id(),
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->f,
                            next->g,
                            format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                            next->reservations);
                }
                else
                {
                    PRINTLN("        Cost infeasible nt {}, n {}, xyt {}, g {}, once-off {}, "
                            "rectangle {}, reservations {}, time f {}, time g {}",
                            next_nt.id(),
                            next_n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
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
        next = push<true>(next);

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                PRINTLN("        {} nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}, "
                        "reservations {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->f,
                        next_copy->g,
                        format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()),
                        next_copy->reservations);
            }
            else
            {
                PRINTLN("        {} nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, "
                        "reservations {}, time f {}, time g {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id(),
                        next_n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()),
                        next_copy->reservations,
                        next_copy->time_f,
                        next_t);
            }
        }
#endif
    }
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::generate_end(Label* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the label data.
    const auto current_nt = current->nt;

    // Skip intervals earlier than the current time.
    const Interval* move_interval;
    {
        // ZoneScopedNC("Skip to earliest move interval", TRACY_COLOUR);

        move_interval = intervals_.get_end_intervals();
        while (current_nt.t >= move_interval->end)
        {
            move_interval = move_interval->next;
        }
    }
    const Interval* wait_interval;
    {
        // ZoneScopedNC("Skip to earliest wait interval", TRACY_COLOUR);

        wait_interval = intervals_.get_intervals(target_, Direction::WAIT);
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
            auto wait_interval = intervals_.get_intervals(target_, Direction::WAIT);
            do
            {
                auto wait_duration =
                    std::min(wait_interval->end, std::max(current_nt.t, move_interval->start)) -
                    std::max(current_nt.t, wait_interval->start);
                wait_duration = std::max(wait_duration, 0);
                check_wait_cost += wait_duration == 0 ?
                                       0.0 :
                                       wait_duration * (default_edge_cost + wait_interval->cost);
            } while ((wait_interval = wait_interval->next));
        }
#endif
        {
            // ZoneScopedNC("Calculate wait cost", TRACY_COLOUR);
            do
            {
                const auto wait_start = wait_end;
                DEBUG_ASSERT(wait_interval->start <= wait_start);
                wait_end = std::min(std::max(wait_end, move_interval->start), wait_interval->end);
                const auto duration_in_wait_interval = wait_end - wait_start;
                DEBUG_ASSERT(wait_start >= 0);
                DEBUG_ASSERT(wait_end >= 0);
                DEBUG_ASSERT(duration_in_wait_interval >= 0);
                if (duration_in_wait_interval > 0 && wait_interval->cost == COST_INF)
                {
                    return;
                }
                wait_cost +=
                    duration_in_wait_interval == 0 ?
                        0.0 :
                        duration_in_wait_interval * (default_edge_cost + wait_interval->cost);
                DEBUG_ASSERT(!std::isnan(wait_cost));
            } while (wait_end >= wait_interval->end && (wait_interval = wait_interval->next));
            DEBUG_ASSERT(is_eq(wait_cost, check_wait_cost));
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
                PRINTLN("        Time infeasible at nt {}, n {}, xyt {}",
                        next_nt.id(),
                        next_nt.n,
                        format_nodetime(next_nt, map_));
            }
#endif

            // Exit.
            return;
        }

        // Allocate memory for the label.
        auto next = new (label_storage_.get_buffer<false, false>()) Label;
        std::memcpy(next, current, label_storage_.object_size());

        // Calculate the g value.
        next->g += wait_cost + move_interval->cost;
        DEBUG_ASSERT(is_ge(next->g, current->g));

        // Exit if cost infeasible.
        if (is_ge(next->g, 0.0))
        {
            // Print.
#ifdef DEBUG
            if (verbose)
            {
                if constexpr (feasible)
                {
                    PRINTLN("        Cost infeasible at end nt {}, n {}, xyt {}, g {}, once-off "
                            "{}, rectangle {}",
                            next_nt.id(),
                            next_nt.n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()));
                }
                else
                {
                    PRINTLN("        Cost infeasible at end nt {}, n {}, xyt {}, g {}, once-off "
                            "{}, rectangle {}, time g {}",
                            next_nt.id(),
                            next_nt.n,
                            format_nodetime(next_nt, map_),
                            next->g,
                            format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                            next_t);
                }
            }
#endif

            // Proceed to the next interval.
            continue;
        }
        DEBUG_ASSERT(next_t >= earliest_target_time_);

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
        next = push<false>(next);

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            if constexpr (feasible)
            {
                PRINTLN(
                    "        {} at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}",
                    next ? "Generated" : "Dominated",
                    next_nt.id(),
                    next_nt.n,
                    format_nodetime(next_nt, map_),
                    next_copy->g,
                    format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()));
            }
            else
            {
                PRINTLN("        {} at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, "
                        "time g {}",
                        next ? "Generated" : "Dominated",
                        next_nt.id(),
                        next_nt.n,
                        format_nodetime(next_nt, map_),
                        next_copy->g,
                        format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()),
                        next_t);
            }
        }
#endif

        // Add the new path to the master problem.
        if (next)
        {
            add_path(next);
        }
    } while ((move_interval = move_interval->next));
}

template <Bool feasible>
template <Bool towards_end>
void IndependentTimeIntervalAStar<feasible>::expand(Label* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Push to four directions.
    const auto current_nt = current->nt;
    const auto current_n = current_nt.n;
    if (const auto next_n = map_.get_north(current_n); map_[next_n])
    {
        generate_nodetimes<towards_end>(next_n, Direction::NORTH, current);
    }
    if (const auto next_n = map_.get_south(current_n); map_[next_n])
    {
        generate_nodetimes<towards_end>(next_n, Direction::SOUTH, current);
    }
    if (const auto next_n = map_.get_west(current_n); map_[next_n])
    {
        generate_nodetimes<towards_end>(next_n, Direction::WEST, current);
    }
    if (const auto next_n = map_.get_east(current_n); map_[next_n])
    {
        generate_nodetimes<towards_end>(next_n, Direction::EAST, current);
    }

    // Push to the waypoint if at the correct location.
    if constexpr (!towards_end)
        if (current_n == waypoints_[next_waypoint_index_].n)
        {
            generate_waypoint<towards_end>(current);
        }

    // Push to the end if at the correct location.
    if constexpr (towards_end)
        if (current_n == target_)
        {
            generate_end(current);
        }
}

#ifdef USE_RESERVATION_LOOKUP
template <Bool feasible>
Bool IndependentTimeIntervalAStar<feasible>::calculate_reservation(const NodeTime nt)
{
    const auto& projection = problem_.projection();
    const auto total_use = projection.find_summed_nodetime(nt);
    const auto individual_use = projection.find_agent_nodetime(a_, nt);
    const auto other_use = total_use - individual_use;
    const auto reservation = is_gt(other_use, 0.0);
    return reservation;
}
#endif

// static inline Bool approx_dominates(const Cost lhs_potential_g, const Cost
// rhs_potential_g,
//                                     const UInt32 lhs_reservations, const
//                                     UInt32 rhs_reservations)
// {
//     return (is_lt(lhs_potential_g, rhs_potential_g)) ||
//            (is_eq(lhs_potential_g, rhs_potential_g) &&
//            is_lt(lhs_reservations, rhs_reservations));
// }
// static inline Bool dominates(const Cost lhs_potential_g, const Cost
// rhs_potential_g,
//                              const UInt32 lhs_reservations, const UInt32
//                              rhs_reservations)
// {
//     return (lhs_potential_g <  rhs_potential_g) ||
//            (lhs_potential_g == rhs_potential_g && lhs_reservations <
//            rhs_reservations);
// }

template <Bool feasible>
template <Bool is_wait_action>
typename IndependentTimeIntervalAStar<feasible>::Label* IndependentTimeIntervalAStar<
    feasible>::push(Label* next)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the new label.
    const auto nt = next->nt;
    const auto next_t = nt.t;

    // Check ordering.
#ifdef DEBUG
    for (auto current = closed_[nt.n]; current && current->next; current = current->next)
    {
        DEBUG_ASSERT(current->nt.t <= current->next->nt.t);
    }
#endif

    // Get the intervals.
    const auto wait_intervals = intervals_.get_intervals(nt.n, Direction::WAIT);

    // Loop through existing labels with the same or earlier time to determine if the new label is
    // dominated.
    const auto next_once_off_bitset = get_once_off_bitset(next);
    const auto next_rectangle_bitset = get_rectangle_bitset(next);
    auto& first = closed_[nt.n];
    Label** insert_prev_next = &first;
    const auto current = next->parent;
    for (auto existing = *insert_prev_next; existing && existing->nt.t <= nt.t;
         existing = *insert_prev_next)
    {
        if (!is_wait_action || existing != current)
        {
            // Get the existing label.
            const auto existing_g = existing->g;
            const auto existing_t = existing->nt.t;

            // Calculate the potential cost of the existing label if it incurred the same penalties
            // as the new label. If the existing label still costs less than or equal to the new
            // label, even after incurring these penalties, then the new label is dominated.
            const auto existing_once_off_bitset = get_once_off_bitset(existing);
            const auto existing_rectangle_bitset = get_rectangle_bitset(existing);
            auto existing_potential_g = existing_g;
            for (Size64 index = 0; index < once_off_penalties_.size(); ++index)
            {
                const auto existing_not_paid = (get_bitset(next_once_off_bitset, index) >
                                                get_bitset(existing_once_off_bitset, index));
                existing_potential_g += once_off_penalties_[index].cost * existing_not_paid;
            }
            for (Size64 index = 0; index < rectangle_penalties_.size(); ++index)
            {
                const auto existing_not_paid = (get_bitset(existing_rectangle_bitset, index) >
                                                get_bitset(next_rectangle_bitset, index));
                existing_potential_g += rectangle_penalties_[index].cost * existing_not_paid;
            }

            // Calculate the cost of waiting until the new arrival time.
            {
                auto wait_interval = wait_intervals;
                do
                {
                    const auto wait_start = std::max(existing_t, wait_interval->start);
                    const auto wait_end = std::min(next_t, wait_interval->end);
                    const auto duration_in_wait_interval = std::max(wait_end - wait_start, 0);
                    DEBUG_ASSERT(wait_start >= 0);
                    DEBUG_ASSERT(wait_end >= 0);
                    DEBUG_ASSERT(duration_in_wait_interval >= 0);
                    existing_potential_g +=
                        duration_in_wait_interval == 0 ?
                            0.0 :
                            duration_in_wait_interval * (default_edge_cost + wait_interval->cost);
                } while (wait_interval->next &&
                         (wait_interval = wait_interval->next)->start < next_t);
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
        Label** prev_next = &first;
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
            DEBUG_ASSERT(existing_t >= next_t);

            // Make the same calculation in the other direction. The new label could be better than
            // the existing label if the new label visited a superset of the once-off penalties
            // visited by the existing label.
            const auto existing_once_off_bitset = get_once_off_bitset(existing);
            const auto existing_rectangle_bitset = get_rectangle_bitset(existing);
            auto next_potential_g = next->g;
            for (Size64 index = 0; index < once_off_penalties_.size(); ++index)
            {
                const auto next_not_paid = (get_bitset(existing_once_off_bitset, index) >
                                            get_bitset(next_once_off_bitset, index));
                next_potential_g += once_off_penalties_[index].cost * next_not_paid;
            }
            for (Size64 index = 0; index < rectangle_penalties_.size(); ++index)
            {
                const auto next_not_paid = (get_bitset(next_rectangle_bitset, index) >
                                            get_bitset(existing_rectangle_bitset, index));
                next_potential_g += rectangle_penalties_[index].cost * next_not_paid;
            }

            // Calculate the cost of waiting until the new arrival time.
            {
                auto wait_interval = wait_intervals;
                do
                {
                    const auto wait_start = std::max(next_t, wait_interval->start);
                    const auto wait_end = std::min(existing_t, wait_interval->end);
                    const auto duration_in_wait_interval = std::max(wait_end - wait_start, 0);
                    DEBUG_ASSERT(wait_start >= 0);
                    DEBUG_ASSERT(wait_end >= 0);
                    DEBUG_ASSERT(duration_in_wait_interval >= 0);
                    next_potential_g +=
                        duration_in_wait_interval == 0 ?
                            0.0 :
                            duration_in_wait_interval * (default_edge_cost + wait_interval->cost);
                } while (wait_interval->next &&
                         (wait_interval = wait_interval->next)->start < existing_t);
            }

            // Delete the existing label if dominated.
            if (is_le(next_potential_g, existing_g))
            {
                // Remove from the priority queue.
                if (existing->pq_index >= 0)
                {
                    open_.erase(existing->pq_index);
                    DEBUG_ASSERT(existing->pq_index == -1);
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
    DEBUG_ASSERT(insert_prev_next);
    next->next = *insert_prev_next;
    *insert_prev_next = next;

    // Check ordering.
#ifdef DEBUG
    for (auto current = closed_[nt.n]; current && current->next; current = current->next)
    {
        DEBUG_ASSERT(current->nt.t <= current->next->nt.t);
    }
#endif

    // Store the new label.
    label_storage_.commit_buffer();
    open_.push(next);
    DEBUG_ASSERT(next->pq_index >= 0);

    // Stored the new label.
    return next;
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::add_path(Label* end)
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(is_end(end->nt));
    const auto reduced_cost = end->g;
    if (is_lt(reduced_cost, 0.0))
    {
        // Retrieve the path.
        auto variable_ptr = Variable::construct(a_, end->nt.t + 1);
        auto path = variable_ptr->path();
        const Label* last;
        {
            // ZoneScopedNC("Get path", TRACY_COLOUR);

            last = end->parent;
            NodeTime nt{last->nt.n, end->nt.t};
            DEBUG_ASSERT(nt.n == target_);
            path[nt.t] = Edge{nt.n, Direction::INVALID};
            for (auto label = (end->nt.t == last->nt.t ? last->parent : last); label;
                 label = label->parent)
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
            PRINTLN("            Time-interval A* found path of length {}, reduced cost {}: {}",
                    path.size(),
                    reduced_cost,
                    format_path_with_time(path, map_));
        }
#endif

        // Check.
#ifdef DEBUG
        DEBUG_ASSERT(path.size() == end->nt.t + 1);
        DEBUG_ASSERT(path.front().n == start_.n);
        DEBUG_ASSERT(path.back().n == target_);
        DEBUG_ASSERT(path.back().n == last->nt.n);
        for (auto label = last; label; label = label->parent)
        {
            DEBUG_ASSERT(path[label->nt.t].n == label->nt.n);
            DEBUG_ASSERT(!label->parent || path[label->nt.t - 1].n == label->parent->nt.n);
        }
        for (Time t = 0; t < path.size() - 1; ++t)
        {
            DEBUG_ASSERT(map_[path[t].n]);
            DEBUG_ASSERT(map_.get_destination(path[t]) == path[t + 1].n);

            const auto [x1, y1] = map_.get_xy(path[t].n);
            const auto [x2, y2] = map_.get_xy(path[t + 1].n);
            DEBUG_ASSERT(std::abs(x2 - x1) + std::abs(y2 - y2) <= 1);
        }
#endif

        // Add the column.
#ifdef USE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER
        problem_.master().add_column(std::move(variable_ptr), reduced_cost);
        ++num_added_;
#endif

        // Store the best found objective value.
        obj_ = std::min(obj_, reduced_cost);
    }
}

template <Bool feasible>
Cost IndependentTimeIntervalAStar<feasible>::solve()
{
    ZoneScopedC(TRACY_COLOUR);

    // Run time-expanded A* to validate results.
#ifdef CHECK_USING_ASTAR
    const auto astar_obj = astar_.solve();
    // const auto astar_obj = COST_NAN;
#endif

    // Print iteration.
    ZoneValue(iter);
#ifdef DEBUG
    if (verbose)
    {
        PRINT_SEP();
        String str;
        for (const auto nt : waypoints_)
        {
            if (str.empty())
            {
                str += " via ";
            }
            else
            {
                str += ", ";
            }
            str += format_nodetime(nt, map_);
        }
        PRINTLN("Running time-interval A* for agent {} from {} to {}{} in iteration {}",
                a_,
                format_node(start_.n, map_),
                format_node(target_, map_),
                str,
                iter);
        PRINTLN("");
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
        // ZoneScopedNC("Compute the earliest and latest target time",
        // TRACY_COLOUR);

        earliest_target_time_ = -1;
        auto interval = intervals_.get_end_intervals();
        do
        {
            if (interval->cost < COST_INF)
            {
                earliest_target_time_ = interval->start;
                break;
            }
        } while ((interval = interval->next));
        if (earliest_target_time_ < 0)
        {
            goto FINISHED;
        }

        latest_target_time_ = -1;
        for (; interval; interval = interval->next)
            if (interval->cost < COST_INF)
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
        // ZoneScopedNC("Compute the minimum time between each waypoint",
        // TRACY_COLOUR);

        // Sort the waypoints by time and insert the target at the back.
        std::sort(waypoints_.begin(),
                  waypoints_.end(),
                  [](const NodeTime a, const NodeTime b) { return a.t < b.t; });
        waypoints_.emplace_back(target_, earliest_target_time_);

        // Calculate the lower bound from each waypoint to the next waypoint.
        h_waypoint_to_target_.resize(waypoints_.size());
        h_waypoint_to_target_.back() = 0;
        for (Size64 w = waypoints_.size() - 2; w >= 0; --w)
        {
            const auto h = distance_heuristic_.get_h(waypoints_[w + 1].n)[waypoints_[w].n];
            const auto time_diff = waypoints_[w + 1].t - waypoints_[w].t;
            if (w != waypoints_.size() - 2 && time_diff < h)
            {
                goto FINISHED;
            }
            h_waypoint_to_target_[w] = std::max(h, time_diff) + h_waypoint_to_target_[w + 1];
        }
    }

    // Prepare data structures.
    {
        ZoneScopedNC("Prepare data structures", TRACY_COLOUR);

        // Calculate lower bounds on finish time penalties.
        intervals_.finalise();

        // Calculate the bytes of the bitsets. Round up to the next 8 and divide by 8.
        once_off_bitset_size_ = ((once_off_penalties_.size() + 7) & (-8)) / 8;
        rectangle_bitset_size_ = ((rectangle_penalties_.size() + 7) & (-8)) / 8;

        // Clear data structures from the previous iteration.
        label_storage_.reset(Label::data_size + once_off_bitset_size_ + rectangle_bitset_size_);
        std::memset(closed_ - 1, 0, (map_.size() + 1) * sizeof(Label*));
        open_.clear();
    }

    // Print penalties.
#ifdef DEBUG
    if (verbose)
    {
        PRINTLN("Constant: {:.4f}\n", constant_);
    }
    if (verbose)
    {
        intervals_.print();
    }
    if (verbose)
    {
        once_off_penalties_.print(map_);
    }
    if (verbose)
    {
        rectangle_penalties_.print(map_);
    }
#endif

    // Solve.
    {
        ZoneScopedNC("Solve", TRACY_COLOUR);

        next_waypoint_index_ = 0;
        auto towards_end = (next_waypoint_index_ == waypoints_.size() - 1);
        NodeTime next_waypoint = waypoints_[next_waypoint_index_];
        waypoint_time_ = next_waypoint.t;
        h_to_waypoint_ = distance_heuristic_.get_h(next_waypoint.n);
        if (towards_end)
        {
            generate_start<true>();
        }
        else
        {
            generate_start<false>();
        }
        while (!open_.empty() && !towards_end)
        {
            ZoneScopedNC("Solving to one waypoint", TRACY_COLOUR);

#ifdef DEBUG
            if (verbose)
            {
                PRINTLN("Searching to waypoint {}", format_nodetime(next_waypoint, map_));
            }
#endif
            while (!open_.empty())
            {
                // Exit if terminated.
                // Problem::stop_if_terminated();

                // Pop the first item off the priority queue.
                auto current = open_.pop();
                const auto nt = current->nt;
                DEBUG_ASSERT(nt.t <= next_waypoint.t);

                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    if constexpr (feasible)
                    {
                        PRINTLN(
                            "    Popped nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle "
                            "{}, reservations {}",
                            nt.id(),
                            nt.n,
                            format_nodetime(nt, map_),
                            current->f,
                            current->g,
                            format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(current),
                                          rectangle_penalties_.size()),
                            current->reservations);
                    }
                    else
                    {
                        PRINTLN(
                            "    Popped nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, "
                            "reservations {}, time f {}, time g {}",
                            nt.id(),
                            nt.n,
                            format_nodetime(nt, map_),
                            current->g,
                            format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(current),
                                          rectangle_penalties_.size()),
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
                    std::memset(closed_ - 1, 0, (map_.size() + 1) * sizeof(Label*));
                    open_.clear();

                    // Insert the nodetime back into the priority queue.
                    push<false>(current);

                    // Advance to the next waypoint.
                    ++next_waypoint_index_;
                    towards_end = (next_waypoint_index_ == waypoints_.size() - 1);
                    next_waypoint = waypoints_[next_waypoint_index_];
                    waypoint_time_ = next_waypoint.t;
                    h_to_waypoint_ = distance_heuristic_.get_h(next_waypoint.n);
                    break;
                }

                // Expand to neighbours.
                expand<false>(current);
            }
        }
        if (!open_.empty() && towards_end)
        {
            ZoneScopedNC("Solving to one waypoint", TRACY_COLOUR);

#ifdef DEBUG
            if (verbose)
            {
                PRINTLN("Searching to waypoint {}", format_nodetime(next_waypoint, map_));
            }
#endif
            while (!open_.empty())
            {
                // Exit if terminated.
                // Problem::stop_if_terminated();

                // Pop the first item off the priority queue.
                auto current = open_.pop();
                const auto nt = current->nt;
                DEBUG_ASSERT(nt.t <= latest_target_time_);

                // Print.
#ifdef DEBUG
                if (verbose)
                {
                    if constexpr (feasible)
                    {
                        PRINTLN(
                            "    Popped nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle "
                            "{}, reservations {}",
                            nt.id(),
                            nt.n,
                            format_nodetime(nt, map_),
                            current->f,
                            current->g,
                            format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(current),
                                          rectangle_penalties_.size()),
                            current->reservations);
                    }
                    else
                    {
                        PRINTLN(
                            "    Popped nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, "
                            "reservations {}, time f {}, time g {}",
                            nt.id(),
                            nt.n,
                            format_nodetime(nt, map_),
                            current->g,
                            format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                            format_bitset(get_rectangle_bitset(current),
                                          rectangle_penalties_.size()),
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
                expand<true>(current);
            }
        }
    }
FINISHED:

    // Check optimal value against results from time-expanded A*.
#ifdef CHECK_USING_ASTAR
    DEBUG_ASSERT(std::isnan(astar_obj) || is_eq(obj_, astar_obj));
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (obj_ >= 0.0)
        {
            PRINTLN("Time-interval A* failed to find a feasible path");
        }
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

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::add_waypoint(const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    // Include the waypoint in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.add_waypoint(nt);
#endif

    // Add a waypoint.
    waypoints_.push_back(nt);
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::set_constant(const Cost constant)
{
    ZoneScopedC(TRACY_COLOUR);

    // Include the constant in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.set_constant(constant);
#endif

    // Overwrite the constant.
    constant_ = constant;
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::add_nodetime_penalty(const NodeTime nt,
                                                                  const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(map_[nt.n]);
    DEBUG_ASSERT(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_waypoint_[nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.add_nodetime_penalty(nt, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        PRINTLN("    Adding nodetime penalty at {} of cost {} to agent {}",
                format_nodetime(nt, map_),
                cost,
                a_);
    }
#endif

    // Add the penalty to the five outgoing directions.
    if (const auto to_n = map_.get_north(nt.n); map_[to_n])
    {
        intervals_.add_penalty(nt.n, Direction::NORTH, nt.t, nt.t + 1, cost);
    }
    if (const auto to_n = map_.get_south(nt.n); map_[to_n])
    {
        intervals_.add_penalty(nt.n, Direction::SOUTH, nt.t, nt.t + 1, cost);
    }
    if (const auto to_n = map_.get_west(nt.n); map_[to_n])
    {
        intervals_.add_penalty(nt.n, Direction::WEST, nt.t, nt.t + 1, cost);
    }
    if (const auto to_n = map_.get_east(nt.n); map_[to_n])
    {
        intervals_.add_penalty(nt.n, Direction::EAST, nt.t, nt.t + 1, cost);
    }
    {
        intervals_.add_penalty(nt.n, Direction::WAIT, nt.t, nt.t + 1, cost);
    }

    // Add the penalty when going to the end if applicable.
    if (nt.n == target_)
    {
        intervals_.add_end_penalty(0, nt.t + 1, cost);
    }
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::add_edgetime_penalty(const EdgeTime et,
                                                                  const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(map_[et.n]);
    DEBUG_ASSERT(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_waypoint_[et.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.add_edgetime_penalty(et, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        PRINTLN("    Adding edgetime penalty at {} of cost {} to agent {}",
                format_edgetime(et, map_),
                cost,
                a_);
    }
#endif

    // Add the penalty to the edgetime.
    intervals_.add_penalty(et.n, et.d, et.t, et.t + 1, cost);

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT && et.n == target_)
    {
        intervals_.add_end_penalty(0, et.t + 1, cost);
    }
}

template <Bool feasible>
template <OnceOffDirection d>
void IndependentTimeIntervalAStar<feasible>::add_once_off_penalty(const NodeTime nt,
                                                                  const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(map_[nt.n]);
    DEBUG_ASSERT(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_waypoint_[nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.template add_once_off_penalty<d>(nt, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        PRINTLN("    Adding once-off penalty at {} of cost {} to agent {}",
                format_nodetime(nt, map_),
                cost,
                a_);
    }
#endif

    // If the node is blocked after a certain time, add it as a penalty. Otherwise handle it in the
    // search because the penalty is only incurred at most once.
    if (cost == COST_INF)
    {
        // TODO
        ASSERT(d == OnceOffDirection::GEq,
               "once-off penalty with <= time direction is not yet "
               "supported for infinite cost");

        // Add the penalty to the five outgoing directions.
        if (const auto to_n = map_.get_north(nt.n); map_[to_n])
        {
            intervals_.add_penalty(nt.n, Direction::NORTH, nt.t, TIME_MAX, cost);
        }
        if (const auto to_n = map_.get_south(nt.n); map_[to_n])
        {
            intervals_.add_penalty(nt.n, Direction::SOUTH, nt.t, TIME_MAX, cost);
        }
        if (const auto to_n = map_.get_west(nt.n); map_[to_n])
        {
            intervals_.add_penalty(nt.n, Direction::WEST, nt.t, TIME_MAX, cost);
        }
        if (const auto to_n = map_.get_east(nt.n); map_[to_n])
        {
            intervals_.add_penalty(nt.n, Direction::EAST, nt.t, TIME_MAX, cost);
        }
        {
            intervals_.add_penalty(nt.n, Direction::WAIT, nt.t, TIME_MAX, cost);
        }

        // Add the penalty when going to the end if applicable.
        if (nt.n == target_)
        {
            intervals_.add_end_penalty(0, nt.t + 1, cost);
        }
    }
    else
    {
        // Add the once-off penalty.
        once_off_penalties_.add(cost, nt, d);

        // If the once-off penalty can be bypassed by waiting at a neighbour node, add intervals to
        // facilitate this bypass.
        if constexpr (d == OnceOffDirection::LEq)
        {
            if (const auto from_n = map_.get_north(nt.n); map_[from_n])
            {
                intervals_.add_penalty(from_n, Direction::SOUTH, nt.t, TIME_MAX, 0.0);
            }
            if (const auto from_n = map_.get_south(nt.n); map_[from_n])
            {
                intervals_.add_penalty(from_n, Direction::NORTH, nt.t, TIME_MAX, 0.0);
            }
            if (const auto from_n = map_.get_west(nt.n); map_[from_n])
            {
                intervals_.add_penalty(from_n, Direction::EAST, nt.t, TIME_MAX, 0.0);
            }
            if (const auto from_n = map_.get_east(nt.n); map_[from_n])
            {
                intervals_.add_penalty(from_n, Direction::WEST, nt.t, TIME_MAX, 0.0);
            }
        }
    }
}

template void IndependentTimeIntervalAStar<true>::add_once_off_penalty<OnceOffDirection::LEq>(
    const NodeTime nt, const Cost cost);
template void IndependentTimeIntervalAStar<true>::add_once_off_penalty<OnceOffDirection::GEq>(
    const NodeTime nt, const Cost cost);
template void IndependentTimeIntervalAStar<false>::add_once_off_penalty<OnceOffDirection::LEq>(
    const NodeTime nt, const Cost cost);
template void IndependentTimeIntervalAStar<false>::add_once_off_penalty<OnceOffDirection::GEq>(
    const NodeTime nt, const Cost cost);

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::add_rectangle_penalty(const EdgeTime first_entry,
                                                                   const EdgeTime first_exit,
                                                                   const Time length,
                                                                   const Node n_increment,
                                                                   const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    DEBUG_ASSERT(cost >= 0.0);

    // Ignore if the the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.add_rectangle_penalty(first_entry, first_exit, length, n_increment, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        PRINTLN("    Adding rectangle penalty from {} and {} of cost {} to agent {}",
                format_edgetime(first_entry, map_),
                format_edgetime(first_exit, map_),
                cost,
                a_);
    }
#endif

    // Add the penality.
    rectangle_penalties_.add(cost, first_entry, first_exit, length, n_increment);

    // Add intervals to bypass crossing the two boundaries.
    const auto d = first_entry.d;
    for (Time i = 0; i <= length; ++i)
    {
        const auto n = first_entry.n + i * n_increment;
        const auto next_n = map_.get_destination(n, d);
        if (map_[n] && map_[next_n])
        {
            const auto t = first_entry.t + i;
            intervals_.add_penalty(n, d, t + 1, TIME_MAX, 0.0);
        }
    }
    for (Time i = 0; i <= length; ++i)
    {
        const auto n = first_exit.n + i * n_increment;
        const auto next_n = map_.get_destination(n, d);
        if (map_[n] && map_[next_n])
        {
            const auto t = first_exit.t + i;
            intervals_.add_penalty(n, d, t + 1, TIME_MAX, 0.0);
        }
    }
}

template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::add_end_penalty(const Time earliest, const Time latest,
                                                             const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Ignore if the penalty is 0.
    DEBUG_ASSERT(cost >= 0.0);
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-expanded A* solver for debugging.
#ifdef CHECK_USING_ASTAR
    astar_.add_end_penalty(earliest, latest, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        PRINTLN("    Adding end penalty on interval [{},{}) of cost {} to agent {}",
                earliest,
                latest,
                cost,
                a_);
    }
#endif

    // Add the penalty.
    DEBUG_ASSERT((earliest == 0) ^ (latest == TIME_MAX));
    intervals_.add_end_penalty(earliest, latest, cost);
}

// template<Bool feasible>
// String IndependentTimeIntervalAStar<feasible>::format_label(const Label*
// const label) const
// {
//     const auto nt = label->nt;
//     if constexpr (feasible)
//     {
//         return fmt::format("nt {}, n {}, xyt {}, f {}, g {}, reservations {},
//         parent {}, ptr {}",
//                            nt.id(), nt.n, format_nodetime(nt, map_), label->f,
//                            label->g, label->reservations, label->parent ?
//                            format_nodetime(label->parent->nt, map_) : "-",
//                            fmt::ptr(label));
//     }
//     else
//     {
//         return fmt::format("nt {}, n {}, xyt {}, g {}, reservations {}, time
//         f {}, time g {}, parent {}, ptr {}",
//                            nt.id(), nt.n, format_nodetime(nt, map_), label->g,
//                            label->reservations, label->time_f, label->nt.t,
//                            label->parent ? format_nodetime(label->parent->nt,
//                            map_) : "-", fmt::ptr(label));
//     }
// }

#ifdef DEBUG
template <Bool feasible>
void IndependentTimeIntervalAStar<feasible>::set_verbose(const Bool value)
{
    verbose = value;
}
#endif

template class IndependentTimeIntervalAStar<true>;
template class IndependentTimeIntervalAStar<false>;

#endif
