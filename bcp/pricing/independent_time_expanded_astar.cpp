/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "master/master.h"
#include "output/formatting.h"
#include "pricing/independent_time_expanded_astar.h"
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

template<Bool feasible>
IndependentTimeExpandedAStar<feasible>::IndependentTimeExpandedAStar(const Instance& instance,
                                                                     Problem& problem,
                                                                     DistanceHeuristic& distance_heuristic,
                                                                     const Agent a,
                                                                     Size& num_added) :
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
    edgetime_penalties_(),
    finish_time_penalties_(),
    once_off_penalties_(),
    rectangle_penalties_(),
    once_off_bitset_size_(0),
    rectangle_bitset_size_(0),
    latest_visit_time_(map_.size()),
    earliest_target_time_(0),
    latest_target_time_(TIME_MAX),

    label_storage_(),
    closed_(),
    open_(),
    obj_()
#ifdef USE_INDEPENDENT_TIME_EXPANDED_ASTAR_PRICER
  , num_added_(num_added)
#endif

#ifdef CHECK_USING_TIASTAR
  , tiastar_(instance, problem, distance_heuristic, a, num_added)
#endif
{
    ZoneScopedC(TRACY_COLOUR);

    // Reset data structures.
    reset();
}

#pragma GCC diagnostic pop

template<Bool feasible>
Byte* IndependentTimeExpandedAStar<feasible>::get_once_off_bitset(Label* const label)
{
    return label->bitset;
}

template<Bool feasible>
Byte* IndependentTimeExpandedAStar<feasible>::get_rectangle_bitset(Label* const label)
{
    return label->bitset + once_off_bitset_size_;
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::reset()
{
    ZoneScopedC(TRACY_COLOUR);

    constant_ = 0.0;
    waypoints_.clear();
    edgetime_penalties_.clear();
    finish_time_penalties_.clear();
    once_off_penalties_.clear();
    rectangle_penalties_.clear();
    for (Node n = 0; n < map_.size(); ++n)
    {
        latest_visit_time_[n] = map_[n] * TIME_MAX;
    }
    earliest_target_time_ = 0;
    latest_target_time_ = TIME_MAX;
}

template<Bool feasible>
template<Bool towards_end>
void IndependentTimeExpandedAStar<feasible>::generate_start()
{
    // ZoneScopedC(TRACY_COLOUR);

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

    // Update the earliest arrival time.
    debug_assert(next_time_f >= earliest_target_time_);
    earliest_target_time_ = next_time_f;

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
    auto next = static_cast<Label*>(label_storage_.get_buffer<false, true>());

    // Calculate the g value.
    next->g = constant_;

    // Calculate the f value.
    if constexpr (feasible)
    {
        // Calculate the h value.
        const auto h_target_to_end = finish_time_penalties_.get_h(next_time_f);
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
                println("        Cost infeasible at start nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->f,
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()));
            }
            else
            {
                println("        Cost infeasible at start nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time f {}, time g {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
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
    // next->parent = nullptr; // Set to 0 when getting buffer
    // next->next = nullptr;
    next = push(next);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if constexpr (feasible)
        {
            println("        Generated start at nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_),
                    next->f,
                    next->g,
                    format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()));
        }
        else
        {
            println("        Generated start at nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time f {}, time g {}",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_),
                    next->g,
                    format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                    next->time_f,
                    next_t);
        }
    }
#endif
}
template<Bool feasible>
template<Bool towards_end>
void IndependentTimeExpandedAStar<feasible>::generate_nodetime(const NodeTime next_nt,
                                                               Label* current,
                                                               const Direction d,
                                                               const Cost edgetime_penalty)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the next nodetime.
    const auto& next_n = next_nt.n;
    const auto& next_t = next_nt.t;
    debug_assert(map_[next_n]);

    // Calculate the distance to the next waypoint.
    const auto next_waypoint_index = towards_end ? 0 : next_waypoint_index_;
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
    auto next = static_cast<Label*>(label_storage_.get_buffer<false, false>());
    std::memcpy(next, current, label_storage_.object_size());

    // Calculate the g value.
    next->g += default_edge_cost + edgetime_penalty;
    {
        auto once_off_bitset = get_once_off_bitset(next);
        for (Size index = 0; index < once_off_penalties_.size(); ++index)
            if (!get_bitset(once_off_bitset, index))
            {
                // Get the penalty data.
                const auto& [cost, nt, d] = once_off_penalties_[index];

                // Accumulate the penalty if the nodetime is crossed.
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
    if (d != Direction::WAIT)
    {
        auto rectangle_bitset = get_rectangle_bitset(next);
        for (Size index = 0; index < rectangle_penalties_.size(); ++index)
        {
            // Get the penalty data.
            const auto& [cost, first_ets, length, n_increment] = rectangle_penalties_[index];

            // Accumulate the penalty if the boundary is crossed.
            if (d == first_ets[0].d)
            {
                const auto num_boundaries_crossed = get_bitset(rectangle_bitset, index);
                const auto et = first_ets[num_boundaries_crossed];
                const auto i = current->nt.t - et.t;
                const auto crossed = (0 <= i && i <= length) &&
                                     (current->nt.n == et.n + i * n_increment) &&
                                     (current->nt.t == et.t + i);
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
        const auto h_target_to_end = finish_time_penalties_.get_h(next_time_f);
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
                println("        Cost infeasible nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->f,
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()));
            }
            else
            {
                println("        Cost infeasible nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time f {}, time g {}",
                        next_nt.id,
                        next_n,
                        format_nodetime(next_nt, map_),
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
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
        next->reservations += calculate_reservation(next_nt);
    }
#endif

    // Push into the priority queue.
    next->nt = next_nt;
    next->parent = current;
    next->next = nullptr;
#ifdef DEBUG
    auto next_copy = next;
#endif
    next = push(next);

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if constexpr (feasible)
        {
            println("        {} nt {}, n {}, xyt {}, f {}, g {}, once-off {}, rectangle {}",
                    next ? "Generated" : "Dominated",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_),
                    next_copy->f,
                    next_copy->g,
                    format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()));
        }
        else
        {
            println("        {} nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time f {}, time g {}",
                    next ? "Generated" : "Dominated",
                    next_nt.id,
                    next_n,
                    format_nodetime(next_nt, map_),
                    next_copy->g,
                    format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()),
                    next_copy->time_f,
                    next_t);
        }
    }
#endif
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::generate_end(Label* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Get the curent nodetime.
    const auto current_nt = current->nt;
    debug_assert(earliest_target_time_ <= current_nt.t && current_nt.t <= latest_target_time_);

    // Get the end nodetime.
    const NodeTime next_nt{end_n(), current->nt.t};
    const auto& next_t = next_nt.t;

    // Allocate memory for the label.
    auto next = static_cast<Label*>(label_storage_.get_buffer<false, false>());
    std::memcpy(next, current, label_storage_.object_size());

    // Calculate the g value.
    next->g += finish_time_penalties_.get_penalty(current_nt.t);
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
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()));
            }
            else
            {
                println("        Cost infeasible at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time g {}",
                        next_nt.id,
                        next_nt.n,
                        format_nodetime(next_nt, map_),
                        next->g,
                        format_bitset(get_once_off_bitset(next), once_off_penalties_.size()),
                        format_bitset(get_rectangle_bitset(next), rectangle_penalties_.size()),
                        next_t);
            }
        }
#endif

        // Exit.
        return;
    }

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
    next->next = nullptr;
#ifdef DEBUG
    auto next_copy = next;
#endif
    next = push(next);

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
                    format_bitset(get_once_off_bitset(next_copy), once_off_penalties_.size()),
                    format_bitset(get_rectangle_bitset(next_copy), rectangle_penalties_.size()));
        }
        else
        {
            println("        {} at end nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time g {}",
                    next ? "Generated" : "Dominated",
                    next_nt.id,
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
}

template<Bool feasible>
template<Bool towards_end>
void IndependentTimeExpandedAStar<feasible>::expand(Label* current)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Push to five directions.
    const auto current_nt = current->nt;
    const auto current_n = current_nt.n;
    const auto next_t = current_nt.t + 1;
    const auto edgetime_penalties = edgetime_penalties_[current_nt];
    if (const auto next_n = map_.get_north(current_n);
        next_t <= latest_visit_time_[next_n] && edgetime_penalties.north < INF)
    {
        const NodeTime next_nt{next_n, next_t};
        generate_nodetime<towards_end>(next_nt, current, Direction::NORTH, edgetime_penalties.north);
    }
    if (const auto next_n = map_.get_south(current_n);
        next_t <= latest_visit_time_[next_n] && edgetime_penalties.south < INF)
    {
        const NodeTime next_nt{next_n, next_t};
        generate_nodetime<towards_end>(next_nt, current, Direction::SOUTH, edgetime_penalties.south);
    }
    if (const auto next_n = map_.get_west(current_n);
        next_t <= latest_visit_time_[next_n] && edgetime_penalties.west < INF)
    {
        const NodeTime next_nt{next_n, next_t};
        generate_nodetime<towards_end>(next_nt, current, Direction::WEST, edgetime_penalties.west);
    }
    if (const auto next_n = map_.get_east(current_n);
        next_t <= latest_visit_time_[next_n] && edgetime_penalties.east < INF)
    {
        const NodeTime next_nt{next_n, next_t};
        generate_nodetime<towards_end>(next_nt, current, Direction::EAST, edgetime_penalties.east);
    }
    if (const auto next_n = map_.get_wait(current_n);
        next_t <= latest_visit_time_[next_n] && edgetime_penalties.wait < INF)
    {
        const NodeTime next_nt{next_n, next_t};
        generate_nodetime<towards_end>(next_nt, current, Direction::WAIT, edgetime_penalties.wait);
    }

    // Push to the end if at the correct location.
    if (towards_end && current_n == target_ && current_nt.t >= earliest_target_time_)
    {
        generate_end(current);
    }
}

#ifdef USE_RESERVATION_LOOKUP
template<Bool feasible>
Bool IndependentTimeExpandedAStar<feasible>::calculate_reservation(const NodeTime nt)
{
    const auto& projection = problem_.projection();
    const auto total_use = projection.find_summed_nodetime(nt);
    const auto individual_use = projection.find_agent_nodetime(a_, nt);
    const auto other_use = total_use - individual_use;
    const auto reservation = is_gt(other_use, 0.0);
    return reservation;
}
#endif

template<Bool feasible>
typename IndependentTimeExpandedAStar<feasible>::Label* IndependentTimeExpandedAStar<feasible>::push(Label* next)
{
    // ZoneScopedC(TRACY_COLOUR);

    // Determine if the new path is dominated by an existing path or vice versa.
    const auto nt = next->nt;
    const auto next_once_off_bitset = get_once_off_bitset(next);
    const auto next_rectangle_bitset = get_rectangle_bitset(next);
    auto insert_prev_next = &closed_[nt];
    for (auto existing = *insert_prev_next; existing; existing = *insert_prev_next)
    {
        // Get the existing label.
        const auto existing_g = existing->g;
        debug_assert(existing->nt == next->nt);

        // Calculate the potential cost of the existing label if it incurred the same penalties as the new label.
        // If the existing label still costs less than or equal to the new label, even after incurring these
        // penalties, then the new label is dominated.
        const auto existing_once_off_bitset = get_once_off_bitset(existing);
        const auto existing_rectangle_bitset = get_rectangle_bitset(existing);
        auto existing_potential_g = existing_g;
        for (Size index = 0; index < once_off_penalties_.size(); ++index)
        {
            const auto existing_not_paid =
                (get_bitset(next_once_off_bitset, index) > get_bitset(existing_once_off_bitset, index));
            existing_potential_g += once_off_penalties_[index].cost * existing_not_paid;
        }
        for (Size index = 0; index < rectangle_penalties_.size(); ++index)
        {
            const auto existing_not_paid =
                (get_bitset(existing_rectangle_bitset, index) > get_bitset(next_rectangle_bitset, index));
            existing_potential_g += rectangle_penalties_[index].cost * existing_not_paid;
        }
        if (is_le(existing_potential_g, next->g))
        {
            return nullptr;
        }

        // Make the same calculation in the other direction. The new label could be better than the existing label if
        // the new label visited a superset of the once-off penalties visited by the existing label.
        auto next_potential_g = next->g;
        for (Size index = 0; index < once_off_penalties_.size(); ++index)
        {
            const auto next_not_paid =
                (get_bitset(existing_once_off_bitset, index) > get_bitset(next_once_off_bitset, index));
            next_potential_g += once_off_penalties_[index].cost * next_not_paid;
        }
        for (Size index = 0; index < rectangle_penalties_.size(); ++index)
        {
            const auto next_not_paid =
                (get_bitset(next_rectangle_bitset, index) > get_bitset(existing_rectangle_bitset, index));
            next_potential_g += rectangle_penalties_[index].cost * next_not_paid;
        }
        if (is_le(next_potential_g, existing_g))
        {
            // If the existing label is not yet expanded, use its memory to store the new label. This may not always
            // happen due to floating point inaccuracies.
            // if (existing->pq_index >= 0)
            // {
            //     if (store_in_existing_label)
            //     {
            //         debug_assert(store_in_existing_label->pq_index >= 0);
            //         open_.erase(store_in_existing_label->pq_index);
            //         debug_assert(store_in_existing_label->pq_index == -1);
            //     }
            //     store_in_existing_label = existing;
            // }

            // Remove from the priority queue.
            if (existing->pq_index >= 0)
            {
                open_.erase(existing->pq_index);
                debug_assert(existing->pq_index == -1);
            }

            // Delete the existing label from future dominance checks.
            *insert_prev_next = existing->next;
        }
        else
        {
            insert_prev_next = &(*insert_prev_next)->next;
        }
    }

    // Link in the new label.
    debug_assert(insert_prev_next);
    debug_assert(!*insert_prev_next);
    *insert_prev_next = next;

    // Store the new label.
    label_storage_.commit_buffer();
    open_.push(next);
    debug_assert(next->pq_index >= 0);

    // Stored the new label.
    return next;
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::add_path(Label* end)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(is_end(end->nt));
    const auto reduced_cost = end->g;
    if (is_lt(reduced_cost, obj_))
    {
        // Retrieve the path.
        auto variable_ptr = Variable::construct(a_, end->nt.t + 1);
        auto path = variable_ptr->path();
        const Label* last;
        {
            ZoneScopedNC("Get path", TRACY_COLOUR);

            last = end->parent;
            NodeTime nt{last->nt.n, end->nt.t};
            debug_assert(nt.n == target_);
            path[nt.t] = Edge{nt.n, Direction::INVALID};
            for (auto label = last->parent; label; label = label->parent)
            {
                const auto prev_nt = label->nt;
                path[prev_nt.t] = Edge{prev_nt.n, map_.get_direction(prev_nt.n, nt.n)};
                nt = prev_nt;
            }
        }

        // Print.
#ifdef DEBUG
        if (verbose)
        {
            println("            Time-expanded A* found path of length {}, reduced cost {}: {}",
                    path.size(), reduced_cost, format_path_with_time(path, map_));
        }
#endif

        // Check.
#ifdef DEBUG
        debug_assert(path.size() == last->nt.t + 1);
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

            const auto [x1, y1] = map_.get_xy(path[t].n);
            const auto [x2, y2] = map_.get_xy(path[t + 1].n);
            debug_assert(std::abs(x2 - x1) + std::abs(y2 - y2) <= 1);
        }
#endif

        // Add the column.
#ifdef USE_INDEPENDENT_TIME_EXPANDED_ASTAR_PRICER
        problem_.master().add_column(std::move(variable_ptr), reduced_cost);
        ++num_added_;
#endif

        // Store the best found objective value.
        obj_ = std::min(obj_, reduced_cost);
    }
}

template<Bool feasible>
Cost IndependentTimeExpandedAStar<feasible>::solve()
{
    ZoneScopedC(TRACY_COLOUR);

    // Run time-interval A* to validate results.
#ifdef CHECK_USING_TIASTAR
    const auto tiastar_obj = tiastar_.solve();
    // const auto tiastar_obj = std::numeric_limits<Cost>::quiet_NaN();
#endif

    // Print iteration.
    ZoneValue(iter);
#ifdef DEBUG
    if (verbose)
    {
        print_separator();
        String str;
        for (const auto nt : waypoints_)
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
        println("Running time-expanded A* for agent {} from {} to {}{} in iteration {}",
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

    // Compute the minimum time between each waypoint.
    {
        // ZoneScopedNC("Compute the minimum time between each waypoint", TRACY_COLOUR);

        // Sort the waypoints by time and insert the target at the back.
        std::sort(waypoints_.begin(),
                  waypoints_.end(),
                  [](const NodeTime a, const NodeTime b) { return a.t < b.t; });
        waypoints_.emplace_back(target_, earliest_target_time_);

        // Calculate the lower bound from each waypoint to the next waypoint.
        h_waypoint_to_target_.resize(waypoints_.size());
        h_waypoint_to_target_.back() = 0;
        for (Size w = waypoints_.size() - 2; w >= 0; --w)
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

        // Finalise costs and penalties.
        finish_time_penalties_.finalise();

        // Calculate the bytes of the bitsets. Round up to the next 8 and divide by 8.
        once_off_bitset_size_ = ((once_off_penalties_.size() + 7) & (-8)) / 8;
        rectangle_bitset_size_ = ((rectangle_penalties_.size() + 7) & (-8)) / 8;

        // Clear data structures from the previous iteration.
        label_storage_.reset(Label::data_size + once_off_bitset_size_ + rectangle_bitset_size_);
        closed_.clear();
        open_.clear();
    }

    // Print penalties.
#ifdef DEBUG
    if (verbose) { println("Constant: {:.4f}\n", constant_); }
    if (verbose) { edgetime_penalties_.print_outgoing(map_); }
    if (verbose) { finish_time_penalties_.print(); }
    if (verbose) { once_off_penalties_.print(map_); }
    if (verbose) { rectangle_penalties_.print(map_); }
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
            if (verbose) { println("Searching to waypoint {}", format_nodetime(next_waypoint, map_)); }
#endif
            while (!open_.empty())
            {
                // Exit if terminated.
                // Problem::stop_if_terminated();

                // Pop the first item off the priority queue.
                auto current = open_.pop();
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
                                format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                                format_bitset(get_rectangle_bitset(current), rectangle_penalties_.size()),
                                current->reservations);
                    }
                    else
                    {
                        println("    Popped nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time f {}, time g {}, reservations {}",
                                nt.id,
                                nt.n,
                                format_nodetime(nt, map_),
                                current->g,
                                format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                                format_bitset(get_rectangle_bitset(current), rectangle_penalties_.size()),
                                current->time_f,
                                current->nt.t,
                                current->reservations);
                    }
                }
#endif

                // Exit if reached the waypoint or the end.
                if (nt == next_waypoint)
                {
                    // Clear closed set and open set.
                    closed_.clear();
                    open_.clear();

                    // Insert the nodetime back into the priority queue.
                    push(current);

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
            if (verbose) { println("Searching to waypoint {}", format_nodetime(next_waypoint, map_)); }
#endif
            while (!open_.empty())
            {
                // Exit if terminated.
                // Problem::stop_if_terminated();

                // Pop the first item off the priority queue.
                auto current = open_.pop();
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
                                format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                                format_bitset(get_rectangle_bitset(current), rectangle_penalties_.size()),
                                current->reservations);
                    }
                    else
                    {
                        println("    Popped nt {}, n {}, xyt {}, g {}, once-off {}, rectangle {}, time f {}, time g {}, reservations {}",
                                nt.id,
                                nt.n,
                                format_nodetime(nt, map_),
                                current->g,
                                format_bitset(get_once_off_bitset(current), once_off_penalties_.size()),
                                format_bitset(get_rectangle_bitset(current), rectangle_penalties_.size()),
                                current->time_f,
                                current->nt.t,
                                current->reservations);
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

    // Check optimal value against results from time-interval A*.
#ifdef CHECK_USING_TIASTAR
    debug_assert(std::isnan(tiastar_obj) || is_eq(obj_, tiastar_obj));
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        if (obj_ >= 0.0) { println("Time-expanded A* failed to find a feasible path"); }
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

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::add_waypoint(const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    // Include the waypoint in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.add_waypoint(nt);
#endif

    // Add a waypoint.
    waypoints_.push_back(nt);
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::set_constant(const Cost constant)
{
    ZoneScopedC(TRACY_COLOUR);

    // Include the constant in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.set_constant(constant);
#endif

    // Overwrite the constasnt.
    constant_ = constant;
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::add_nodetime_penalty(const NodeTime nt, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_waypoint_[nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.add_nodetime_penalty(nt, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        println("    Adding nodetime penalty at {} of cost {} to agent {}", format_nodetime(nt, map_), cost, a_);
    }
#endif

    // Add the penalty to the five outgoing directions.
    if (const auto to_n = map_.get_north(nt.n); map_[to_n])
    {
        edgetime_penalties_.add_edgetime_penalty(nt, Direction::NORTH, cost);
    }
    if (const auto to_n = map_.get_south(nt.n); map_[to_n])
    {
        edgetime_penalties_.add_edgetime_penalty(nt, Direction::SOUTH, cost);
    }
    if (const auto to_n = map_.get_west(nt.n); map_[to_n])
    {
        edgetime_penalties_.add_edgetime_penalty(nt, Direction::WEST, cost);
    }
    if (const auto to_n = map_.get_east(nt.n); map_[to_n])
    {
        edgetime_penalties_.add_edgetime_penalty(nt, Direction::EAST, cost);
    }
    {
        edgetime_penalties_.add_edgetime_penalty(nt, Direction::WAIT, cost);
    }

    // Add the penalty when going to the end if applicable.
    if (nt.n == target_)
    {
        finish_time_penalties_.add(nt.t, cost);
    }
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::add_edgetime_penalty(const EdgeTime et, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(map_[et.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_waypoint_[et.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.add_edgetime_penalty(et, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        println("    Adding edgetime penalty at {} of cost {} to agent {}", format_edgetime(et, map_), cost, a_);
    }
#endif

    // Add the penalty to the edgetime.
    edgetime_penalties_.add_edgetime_penalty(et.nt(), et.d, cost);

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT && et.n == target_)
    {
        finish_time_penalties_.add(et.t, cost);
    }
}

template<Bool feasible>
template<OnceOffDirection d>
void IndependentTimeExpandedAStar<feasible>::add_once_off_penalty(const NodeTime nt, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_waypoint_[nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.template add_once_off_penalty<d>(nt, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        println("    Adding once-off penalty at {} of cost {} to agent {}",
                format_nodetime(nt, map_), cost, a_);
    }
#endif

    // If the node is blocked after a certain time, tighten the latest visit time. Otherwise handle it in the search
    // because the penalty is only incurred at most once.
    if (cost == INF)
    {
        // TODO
        release_assert(d == OnceOffDirection::GEq,
                       "Once-off penalty with <= time direction is not yet supported for infinite cost");

        // Tighten the node visit time bounds.
        latest_visit_time_[nt.n] = std::min(latest_visit_time_[nt.n], nt.t - 1);

        // Tighten the finish time bounds if applicable.
        if (nt.n == target_)
        {
            latest_target_time_ = std::min(latest_target_time_, nt.t - 1);
        }
    }
    else
    {
        once_off_penalties_.add(cost, nt, d);
    }
}
template void IndependentTimeExpandedAStar<true>::add_once_off_penalty<OnceOffDirection::LEq>(
    const NodeTime nt, const Cost cost);
template void IndependentTimeExpandedAStar<true>::add_once_off_penalty<OnceOffDirection::GEq>(
    const NodeTime nt, const Cost cost);
template void IndependentTimeExpandedAStar<false>::add_once_off_penalty<OnceOffDirection::LEq>(
    const NodeTime nt, const Cost cost);
template void IndependentTimeExpandedAStar<false>::add_once_off_penalty<OnceOffDirection::GEq>(
    const NodeTime nt, const Cost cost);

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::add_rectangle_penalty(const EdgeTime first_entry,
                                                                   const EdgeTime first_exit,
                                                                   const Time length,
                                                                   const Node n_increment,
                                                                   const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(cost >= 0.0);

    // Ignore if the the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.add_rectangle_penalty(first_entry, first_exit, length, n_increment, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        println("    Adding rectangle penalty from {} and {} of cost {} to agent {}",
                format_edgetime(first_entry, map_), format_edgetime(first_exit, map_), cost, a_);
    }
#endif

    // Add the penality.
    rectangle_penalties_.add(cost, first_entry, first_exit, length, n_increment);
}

template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::add_end_penalty(const Time earliest, const Time latest, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Ignore if the penalty is 0.
    debug_assert(cost >= 0.0);
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Include the penalty in the time-interval A* solver for debugging.
#ifdef CHECK_USING_TIASTAR
    tiastar_.add_end_penalty(earliest, latest, cost);
#endif

    // Print.
#ifdef DEBUG
    if (verbose)
    {
        println("    Adding end penalty on interval [{},{}) of cost {} to agent {}", earliest, latest, cost, a_);
    }
#endif

    // Tighten the finish time bounds.
    debug_assert((earliest == 0) ^ (latest == TIME_MAX));
    if (cost == INF)
    {
        if (earliest == 0)
        {
            earliest_target_time_ = std::max(earliest_target_time_, latest);
        }
        else
        {
            debug_assert(latest == TIME_MAX);
            latest_target_time_ = std::min(latest_target_time_, earliest - 1);
        }
    }
    else if (earliest == 0)
    {
        debug_assert(latest >= 1);
        finish_time_penalties_.add(latest - 1, cost);
    }
    else
    {
        err("Adding end penalty unhandled");
    }
}

#ifdef DEBUG
template<Bool feasible>
void IndependentTimeExpandedAStar<feasible>::set_verbose(const Bool value)
{
    verbose = value;
}
#endif

template class IndependentTimeExpandedAStar<true>;
template class IndependentTimeExpandedAStar<false>;
