/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/corridor_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "pricing/distance_heuristic.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/map_types.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::Firebrick

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

CorridorConflictSeparator::CorridorConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    distance_heuristic_(problem.pricer().distance_heuristic()),
    candidates_(),
    num_separated_(instance.num_agents(), instance.num_agents())
{
}

void CorridorConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN("Starting separator for corridor conflicts");

    // Get the problem data.
    const auto A = instance_.num_agents();
    const auto& map = instance_.map;
    const auto& master = problem_.master();

    // Get the solution.
    const auto& projection = problem_.projection();

    // Print paths.
#ifdef PRINT_DEBUG
    problem_.master().print_paths();
    DEBUGLN("");
#endif

    // Find cuts.
    candidates_.clear();
    for (Agent a1 = 0; a1 < A; ++a1)
    {
        const auto a1_target = instance_.agents[a1].target;
        for (const auto& [a1_et, a1_et_val] : projection.agent_move_edgetimes(a1))
            if ((a1_et.d == Direction::NORTH || a1_et.d == Direction::WEST) &&
                !is_integral(a1_et_val))
            {
                const auto nt = a1_et.nt();
                const auto degree = map.get_degree(nt.n);
                if (degree == 2)
                {
                    // Get the opposite edgetime.
                    const EdgeTime a2_et{map.get_opposite_edge(a1_et.e()), a1_et.t};
                    const auto& a2_et_vals = projection.find_list_fractional_move_edgetime(a2_et);

                    // Loop through the second agent.
                    for (Agent a2 = 0; a2 < A; ++a2)
                    {
                        const auto a2_target = instance_.agents[a2].target;
                        const auto a2_et_val = a2_et_vals[a2];
                        if (is_gt(a2_et_val, 0.0) && a2 != a1 && a1_et.n != a2_target &&
                            a2_et.n != a1_target)
                        {
                            // Determine the endpoints of the corridor.
                            const auto [a1_begin, a2_begin, a1_end, a2_end] =
                                find_endpoint(a1_et, a2_et, a1_target, a2_target, map);
                            DEBUGLN("    Agent {} endpoints {} and {} (start {}, target {})",
                                    a1,
                                    format_nodetime(a1_begin, map),
                                    format_nodetime(a1_end, map),
                                    format_node(instance_.agents[a1].start, map),
                                    format_node(instance_.agents[a1].target, map));
                            DEBUGLN("    Agent {} endpoints {} and {} (start {}, target {})",
                                    a2,
                                    format_nodetime(a2_begin, map),
                                    format_nodetime(a2_end, map),
                                    format_node(instance_.agents[a2].start, map),
                                    format_node(instance_.agents[a2].target, map));
                            DEBUG_ASSERT(a1_begin.t == a2_begin.t);
                            DEBUG_ASSERT(a1_end.t == a2_end.t);
                            DEBUG_ASSERT(a1_begin.n == a2_end.n);
                            DEBUG_ASSERT(a1_end.n == a2_begin.n);

                            // Ignore pure edge conflicts.
                            const auto corridor_length = a1_end.t - a1_begin.t + 1;
                            DEBUG_ASSERT(corridor_length % 2 == 0);
                            if (corridor_length >= 4)
                            {
                                // Check.
                                DEBUG_ASSERT(a1_end.n != a1_target);
                                DEBUG_ASSERT(a2_end.n != a2_target);

                                // Get the earliest arrival time at the end of the corridor.
                                const auto a1_start = instance_.agents[a1].start;
                                const auto a2_start = instance_.agents[a2].start;
                                const auto a1_time_to_corridor_begin =
                                    distance_heuristic_.get_h(a1_start)[a1_begin.n];
                                const auto a2_time_to_corridor_begin =
                                    distance_heuristic_.get_h(a2_start)[a2_begin.n];
                                const auto a1_time_to_corridor_end =
                                    distance_heuristic_.get_h(a1_start)[a1_end.n];
                                const auto a2_time_to_corridor_end =
                                    distance_heuristic_.get_h(a2_start)[a2_end.n];

                                // If the agents can reach the end of the corridor before the start
                                // of the corridor, then the agent turned around and there is no
                                // valid corridor conflict.
                                if (a1_time_to_corridor_end < a1_time_to_corridor_begin ||
                                    a2_time_to_corridor_end < a2_time_to_corridor_begin)
                                {
                                    continue;
                                }

                                // Get the earliest arrival time if taking a detour. This requires a
                                // corridor of length at least 3 so that the blocked node is not
                                // within the corridor.
                                auto blocked_map = map;
                                blocked_map.set_obstacle(a1_et.n);
                                blocked_map.set_obstacle(a2_et.n);
                                const auto a1_detour_time_to_corridor_end =
                                    distance_heuristic_.get_h_using_map(a1_start,
                                                                        blocked_map)[a1_end.n];
                                const auto a2_detour_time_to_corridor_end =
                                    distance_heuristic_.get_h_using_map(a2_start,
                                                                        blocked_map)[a2_end.n];
                                DEBUG_ASSERT(a1_et.n != a1_begin.n);
                                DEBUG_ASSERT(a1_et.n != a1_end.n);
                                DEBUG_ASSERT(a1_et.n != a2_begin.n);
                                DEBUG_ASSERT(a1_et.n != a2_end.n);
                                DEBUG_ASSERT(a2_et.n != a1_begin.n);
                                DEBUG_ASSERT(a2_et.n != a1_end.n);
                                DEBUG_ASSERT(a2_et.n != a2_begin.n);
                                DEBUG_ASSERT(a2_et.n != a2_end.n);

                                // Get the overall earliest arrival time.
                                const auto a1_earliest_arrival_time =
                                    std::min(a1_detour_time_to_corridor_end,
                                             a2_time_to_corridor_end + corridor_length + 2);
                                const auto a2_earliest_arrival_time =
                                    std::min(a2_detour_time_to_corridor_end,
                                             a1_time_to_corridor_end + corridor_length + 2);
                                const NodeTime a1_earliest_arrival{a1_end.n,
                                                                   a1_earliest_arrival_time};
                                const NodeTime a2_earliest_arrival{a2_end.n,
                                                                   a2_earliest_arrival_time};

                                // Compute the LHS.
                                Real64 lhs = 0.0;
                                for (const auto& variable : master.agent_variables(a1))
                                {
                                    const auto val = master.variable_primal_sol(variable);
                                    if (is_gt(val, 0.0))
                                    {
                                        const auto& path = variable.path();
                                        for (Time t = 0;
                                             t < std::min<Time>(path.size(), a1_earliest_arrival.t);
                                             ++t)
                                            if (path[t].n == a1_earliest_arrival.n)
                                            {
                                                lhs += val;
                                                break;
                                            }
                                    }
                                }
                                for (const auto& variable : master.agent_variables(a2))
                                {
                                    const auto val = master.variable_primal_sol(variable);
                                    if (is_gt(val, 0.0))
                                    {
                                        const auto& path = variable.path();
                                        for (Time t = 0;
                                             t < std::min<Time>(path.size(), a2_earliest_arrival.t);
                                             ++t)
                                            if (path[t].n == a2_earliest_arrival.n)
                                            {
                                                lhs += val;
                                                break;
                                            }
                                    }
                                }

                                // Store a cut candidate if violated.
                                if (is_gt(lhs, 1.0 + CUT_VIOLATION))
                                {
                                    candidates_.push_back(
                                        {lhs, a1, a2, a1_earliest_arrival, a2_earliest_arrival});
                                }
                            }
                        }
                    }
                }
            }
    }

    // Create the most violated cuts.
    Size64 num_separated_this_run = 0;
    num_separated_.set(0);
    std::sort(candidates_.begin(),
              candidates_.end(),
              [](const auto& a, const auto& b) { return a.lhs > b.lhs; });
    for (const auto& candidate : candidates_)
    {
        const auto& [lhs, a1, a2, a1_earliest_arrival, a2_earliest_arrival] = candidate;
        auto& num_separated = num_separated_(std::min(a1, a2), std::max(a1, a2));
        if (num_separated == 0)
        {
            // Create the row.
            create_row(candidate);
            ++num_separated;

            // Exit if enough cuts are found.
            ++num_separated_this_run;
            if (num_separated_this_run >= MAX_CUTS_PER_RUN)
            {
                break;
            }
        }
    }
}

Tuple<NodeTime, NodeTime, NodeTime, NodeTime> CorridorConflictSeparator::find_endpoint(
    const EdgeTime a1_et, const EdgeTime a2_et, const Node a1_target, const Node a2_target,
    const Map& map)
{
    // Get the starting location for searching outward.
    Time time_diff = 0;
    Node a1_current = a1_et.n;
    Node a1_next = a2_et.n;
    Node a2_current = a2_et.n;
    Node a2_next = a1_et.n;
    DEBUGLN("Starting search for corridor from {} {} at time {}:",
            format_node(a1_current, map),
            format_node(a2_current, map),
            a1_et.t);

    // Search outward.
    while (map.get_degree(a1_next) == 2 && map.get_degree(a2_next) == 2)
    {
        // Extend the corridor.
        Node a1_previous = a1_current;
        Node a2_previous = a2_current;
        a1_current = a1_next;
        a2_current = a2_next;
        ++time_diff;
        DEBUGLN("    {} {}", format_node(a1_current, map), format_node(a2_current, map));

        // Find a neighbour that has degree 2 and is not the start or target of either agent.
        for (Size64 a1_d = 0; a1_d < 4; ++a1_d)
        {
            a1_next = map.get_destination(a1_current, static_cast<Direction>(a1_d));
            if (map[a1_next] && map.get_degree(a1_next) <= 2 && a1_next != a1_previous &&
                a1_next != a1_target)
            {
                for (Size64 a2_d = 0; a2_d < 4; ++a2_d)
                {
                    a2_next = map.get_destination(a2_current, static_cast<Direction>(a2_d));
                    if (map[a2_next] && map.get_degree(a2_next) <= 2 && a2_next != a2_previous &&
                        a2_next != a2_target)
                    {
                        goto NEXT_LOCATION;
                    }
                }
            }
        }
        break;
    NEXT_LOCATION:;
    }
    return {NodeTime{a2_current, a1_et.t - time_diff + 1},
            NodeTime{a1_current, a2_et.t - time_diff + 1},
            NodeTime{a1_current, a1_et.t + time_diff},
            NodeTime{a2_current, a2_et.t + time_diff}};
}

void CorridorConflictSeparator::create_row(const CorridorConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, a1, a2, a1_earliest_arrival, a2_earliest_arrival] = candidate;

    // Print.
    DEBUGLN("    Creating corridor conflict cut on {} up to time {} for agent {} and {} up to time "
            "{} for agent {} with LHS {}",
            format_node(a1_earliest_arrival.n, map),
            a1_earliest_arrival.t,
            a1,
            format_node(a2_earliest_arrival.n, map),
            a2_earliest_arrival.t,
            a2,
            lhs);

    // Create the row.
    auto name = fmt::format("corridor({},{},{},{},{},{})",
                            a1,
                            format_node(a1_earliest_arrival.n, map),
                            a1_earliest_arrival.t,
                            a2,
                            format_node(a2_earliest_arrival.n, map),
                            a2_earliest_arrival.t);
    constexpr auto data_size = sizeof(ConstraintData);
    constexpr auto hash_size = data_size;
    auto constraint = Constraint::construct(
        '<', 1.0, 2, data_size, hash_size, &apply_in_pricer, &get_coeff, name);
    auto data = new (constraint->data()) ConstraintData;
    data->a1 = a1;
    data->a2 = a2;
    data->a1_earliest_arrival = a1_earliest_arrival;
    data->a2_earliest_arrival = a2_earliest_arrival;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void CorridorConflictSeparator::apply_in_pricer(const Constraint& constraint, const Real64 dual,
                                                Pricer& pricer)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [a1, a2, a1_earliest_arrival, a2_earliest_arrival] =
        *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Add the dual solution to the reduced cost function.
    const NodeTime a1_conflict_arrival{a1_earliest_arrival.n, a1_earliest_arrival.t - 1};
    const NodeTime a2_conflict_arrival{a2_earliest_arrival.n, a2_earliest_arrival.t - 1};
    pricer.add_once_off_penalty_one_agent<OnceOffDirection::LEq>(a1, a1_conflict_arrival, -dual);
    pricer.add_once_off_penalty_one_agent<OnceOffDirection::LEq>(a2, a2_conflict_arrival, -dual);
}

Real64 CorridorConflictSeparator::get_coeff(const Constraint& constraint, const Agent a,
                                            const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [a1, a2, a1_earliest_arrival, a2_earliest_arrival] =
        *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Calculate coefficient.
    if (a == a1)
    {
        Real64 coeff = 0.0;
        for (Time t = 0; t < std::min<Time>(path.size(), a1_earliest_arrival.t); ++t)
            if (path[t].n == a1_earliest_arrival.n)
            {
                coeff = 1.0;
                break;
            }
        return coeff;
    }
    else if (a == a2)
    {
        Real64 coeff = 0.0;
        for (Time t = 0; t < std::min<Time>(path.size(), a2_earliest_arrival.t); ++t)
            if (path[t].n == a2_earliest_arrival.n)
            {
                coeff = 1.0;
                break;
            }
        return coeff;
    }
    else
    {
        return 0.0;
    }
}
