/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/multi_agent_exit_entry_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/pointers.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::Salmon

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

MultiAgentExitEntryConflictSeparator::MultiAgentExitEntryConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    candidates_(),
    num_separated_(instance.agents.size())
{
}

void MultiAgentExitEntryConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for multi-agent exit-entry conflicts");

    // Get the problem data.
    const Agent A = instance_.agents.size();
    const auto& map = instance_.map;

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    for (Agent canonical = 0; canonical < A; ++canonical)
        for (const auto& [canonical_et, et_val_canonical] : projection.agent_move_edgetimes(canonical))
            if (!is_integral(et_val_canonical))
            {
                debug_assert(canonical_et.d != Direction::WAIT);

                // Get the usage of the edgetime.
                const auto& [et_total, et_vals] = projection.find_list_fractional_move_edgetime(canonical_et);
                debug_assert(is_eq(et_val_canonical, et_vals[canonical]));
                const auto et_val_other = et_total - et_val_canonical;

                // Get the origin and destination of the edge.
                const auto t = canonical_et.t;
                const auto canonical_e = canonical_et.et.e;
                const auto origin = canonical_e.n;
                const auto destination = map.get_destination(canonical_e);

                // Get the usage at the origin and destination.
                const auto& [origin_total, origin_vals] = projection.find_list_fractional_nodetime(NodeTime{origin, t});
                const auto& [destination_total, destination_vals] =
                    projection.find_list_fractional_nodetime(NodeTime{destination, t + 1});
                const auto origin_val_other = origin_total - origin_vals[canonical];
                const auto destination_val_other = destination_total - destination_vals[canonical];

                // Get the opposite edge.
                const auto opposite = EdgeTime{map.get_opposite_edge(canonical_e), t};
                const auto& [opposite_total, opposite_vals] = projection.find_list_fractional_move_edgetime(opposite);
                const auto opposite_other = opposite_total - opposite_vals[canonical];

                // Compute part of the LHS, comprising the canonical agent's edge, all edges outgoing from the origin,
                // all edges incoming to the destination and subtracting the duplicated edge.
                const auto lhs = 3.0 * et_val_canonical + origin_val_other + destination_val_other - et_val_other +
                                 opposite_other;

                // Store a cut candidate if violated and more than two other agents are involved.
                // println("Multi-agent exit-entry cut LHS: {}", lhs);
                if (is_gt(lhs, 3.0 + CUT_VIOLATION))
                {
                    // Count the number of other agents involved in the cut.
                    Size num_other = 0;
                    for (Agent a = 0; a < A; ++a)
                        if (a != canonical)
                        {
                            num_other += is_gt(origin_vals[a], 0.0) +
                                         is_gt(destination_vals[a], 0.0) +
                                         is_gt(opposite_vals[a], 0.0);
                        }

                    // Store the candidate if involving three or more agents.
                    if (num_other >= 2)
                    {
                        candidates_.push_back({lhs,
                                               num_other,
                                               canonical,
                                               t,
                                               canonical_e,
                                               opposite.et.e,
                                               origin,
                                               destination});
                    }
                }
            }

    // Create the most violated cuts.
    Size num_separated_this_run = 0;
    num_separated_.clear();
    std::sort(candidates_.begin(),
              candidates_.end(),
              [](const auto& a, const auto& b) { return std::tie(a.lhs, a.num_other) > std::tie(b.lhs, b.num_other); });
    for (const auto& candidate : candidates_)
    {
        const auto& [lhs, num_other, canonical, t, canonical_e, opposite_e, origin, destination] = candidate;
        auto& num_separated = num_separated_[{canonical, t}];
        if (num_separated == 0)
        {
            // Create the row.
            create_row(candidate);
            ++num_added_;
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

void MultiAgentExitEntryConflictSeparator::create_row(const MultiAgentExitEntryConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, num_other, canonical, t, canonical_e, opposite_e, origin, destination] = candidate;

    // Include the opposite edge.
    Size32 num_other_es = 1;
    Edge other_es[10]{opposite_e};

    // Include the five outgoing edges from the origin.
    if (const auto to_n = map.get_north(origin); map[to_n])
    {
        other_es[num_other_es++] = Edge{origin, Direction::NORTH};
    }
    if (const auto to_n = map.get_south(origin); map[to_n])
    {
        other_es[num_other_es++] = Edge{origin, Direction::SOUTH};
    }
    if (const auto to_n = map.get_west(origin); map[to_n])
    {
        other_es[num_other_es++] = Edge{origin, Direction::WEST};
    }
    if (const auto to_n = map.get_east(origin); map[to_n])
    {
        other_es[num_other_es++] = Edge{origin, Direction::EAST};
    }
    {
        other_es[num_other_es++] = Edge{origin, Direction::WAIT};
    }

    // Include the five incoming edges to the destination.
    if (const auto from_n = map.get_north(destination); from_n != origin && map[from_n])
    {
        debug_assert(num_other_es < 10);
        other_es[num_other_es++] = Edge{from_n, Direction::SOUTH};
    }
    if (const auto from_n = map.get_south(destination); from_n != origin && map[from_n])
    {
        debug_assert(num_other_es < 10);
        other_es[num_other_es++] = Edge{from_n, Direction::NORTH};
    }
    if (const auto from_n = map.get_west(destination);  from_n != origin && map[from_n])
    {
        debug_assert(num_other_es < 10);
        other_es[num_other_es++] = Edge{from_n, Direction::EAST};
    }
    if (const auto from_n = map.get_east(destination);  from_n != origin && map[from_n])
    {
        debug_assert(num_other_es < 10);
        other_es[num_other_es++] = Edge{from_n, Direction::WEST};
    }
    {
        debug_assert(num_other_es < 10);
        other_es[num_other_es++] = Edge{destination, Direction::WAIT};
    }

    // Check.
// #ifdef DEBUG
//     {
//         const auto& projection = problem_.projection();
//         Float check_lhs = 3 * projection.find_agent_move_edgetime(canonical, EdgeTime{canonical_e, t});
//         for (Size idx = 0; idx < num_other_es; ++idx)
//         {
//             const auto& [total, agent] = other_es[idx].d != Direction::WAIT ?
//                                          projection.find_list_fractional_move_edgetime(EdgeTime{other_es[idx], t}) :
//                                          projection.find_list_fractional_wait_edgetime(EdgeTime{other_es[idx], t});
//             check_lhs += total - agent[canonical];
//
//             for (Size idx2 = idx + 1; idx2 < num_other_es; ++idx2)
//                 debug_assert(other_es[idx] != other_es[idx2]);
//         }
//         debug_assert(is_eq(lhs, check_lhs));
//     }
// #endif

    // Print.
#ifdef PRINT_DEBUG
    {
        String other_str;
        for (Size idx = 0; idx < num_other_es; ++idx)
        {
            if (!other_str.empty())
            {
                other_str += ", ";
            }
            const auto e = other_es[idx];
            other_str += format_edgetime(EdgeTime{e, t}, map);
        }
        debugln("    Creating multi-agent exit-entry conflict cut on {} for agent {} and {} for other agents "
                "with LHS {}",
                format_edgetime(EdgeTime{canonical_e, t}, map),
                canonical,
                other_str,
                lhs);
    }
#endif

    // Create the row.
    auto name = fmt::format("multi_agent_exit_entry({},{},{},{})",
                            canonical, format_edgetime(EdgeTime{canonical_e, t}, map),
                            format_node(origin, map), format_node(destination, map));
    const auto array_size = sizeof(Edge) * num_other_es;
    const auto object_size = sizeof(MultiAgentExitEntryConstraint) + array_size;
    const auto hash_size = sizeof(Agent) * 2 + sizeof(Time) + sizeof(Edge) + sizeof(Size32) + array_size;
    auto constraint = Constraint::construct<MultiAgentExitEntryConstraint>(object_size,
                                                                           hash_size,
                                                                           ConstraintFamily::MultiAgentExitEntry,
                                                                           this,
                                                                           std::move(name),
                                                                           1,
                                                                           '<',
                                                                           3.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->every_agent) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    // WARNING: Cannot add as a universal cut because each agent has costs on different edgetimes.
    constraint->every_agent = -1; // The master problem uses this -1 together with the 1 in construct() to determine the
                                  // agents to call this separator.
    constraint->canonical = canonical;
    constraint->t = t;
    constraint->canonical_e = canonical_e;
    constraint->num_other_es = num_other_es;
    std::copy(other_es, other_es + num_other_es, constraint->other_es);
    master.add_row(std::move(constraint));
}

void MultiAgentExitEntryConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& multi_agent_exit_entry_constraint = *static_cast<const MultiAgentExitEntryConstraint*>(&constraint);
    const auto canonical = multi_agent_exit_entry_constraint.canonical;
    const auto t = multi_agent_exit_entry_constraint.t;
    const auto canonical_e = multi_agent_exit_entry_constraint.canonical_e;
    const auto num_other_es = multi_agent_exit_entry_constraint.num_other_es;
    const auto other_es = multi_agent_exit_entry_constraint.other_es;

    // Add the penalty on the edgetimes.
    auto& pricer = problem_.pricer();
    pricer.add_edgetime_penalty_one_agent(canonical, EdgeTime{canonical_e, t}, -3.0 * dual);
    for (Size index = 0; index < num_other_es; ++index)
    {
        const EdgeTime et{other_es[index], t};
        pricer.add_edgetime_penalty_all_except_one_agent(canonical, et, -dual);
    }
}

Float MultiAgentExitEntryConflictSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& multi_agent_exit_entry_constraint = *static_cast<const MultiAgentExitEntryConstraint*>(&constraint);
    const auto canonical = multi_agent_exit_entry_constraint.canonical;
    const auto t = multi_agent_exit_entry_constraint.t;
    const auto canonical_e = multi_agent_exit_entry_constraint.canonical_e;
    const auto num_other_es = multi_agent_exit_entry_constraint.num_other_es;
    const auto other_es = multi_agent_exit_entry_constraint.other_es;

    // Calculate coefficient.
    if (a == canonical)
    {
        return calculate_canonical_agent_coeff(canonical_e, t, path);
    }
    else
    {
        return calculate_other_agent_coeff(num_other_es, other_es, t, path);
    }
}

Float MultiAgentExitEntryConflictSeparator::calculate_canonical_agent_coeff(const Edge e,
                                                                            const Time t,
                                                                            const Path& path)
{
    return 3.0 * calculate_move_edgetime_coeff(EdgeTime{e, t}, path);
}

Float MultiAgentExitEntryConflictSeparator::calculate_other_agent_coeff(const Size32 num_es,
                                                                        const Edge* es,
                                                                        const Time t,
                                                                        const Path& path)
{
    for (Size idx = 0; idx < num_es; ++idx)
        if (calculate_edgetime_coeff(EdgeTime{es[idx], t}, path))
        {
            return true;
        }
    return false;
}
