/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/exit_entry_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/pointers.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::LightCoral

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

ExitEntryConflictSeparator::ExitEntryConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    candidates_(),
    num_separated_(instance.num_agents(), instance.num_agents())
{
}

void ExitEntryConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for exit-entry conflicts");

    // Get the problem data.
    const auto A = instance_.num_agents();
    const auto& map = instance_.map;

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    for (Agent a1 = 0; a1 < A; ++a1)
        for (const auto& [a1_et, a1_et_val] : projection.agent_move_edgetimes(a1))
            if (!is_integral(a1_et_val))
            {
                debug_assert(a1_et.d != Direction::WAIT);

                // Get the origin and destination of the edge.
                const auto t = a1_et.t;
                const auto a1_e = a1_et.et.e;
                const auto origin = a1_e.n;
                const auto destination = map.get_destination(a1_e);

                // Get the usage at the origin and destination.
                const auto& origin_vals = projection.find_list_fractional_nodetime(NodeTime{origin, t});
                const auto& destination_vals = projection.find_list_fractional_nodetime(NodeTime{destination, t + 1});

                // Get the usage of the opposite edge.
                const auto a2_opposite_et = EdgeTime{map.get_opposite_edge(a1_e), t};
                const auto& a2_et_vals = projection.find_list_fractional_move_edgetime(a1_et);
                const auto& a2_opposite_et_vals = projection.find_list_fractional_move_edgetime(a2_opposite_et);

                // Stop processing if a cut cannot possibly be violated.
                const auto potential_lhs = a1_et_val +
                                           (origin_vals.total - origin_vals[a1]) +
                                           (destination_vals.total - destination_vals[a1]) +
                                           (a2_opposite_et_vals.total - a2_opposite_et_vals[a1]);
                if (is_le(potential_lhs, 1.0 + CUT_VIOLATION))
                {
                    continue;
                }

                // Loop through the second agent.
                for (Agent a2 = 0; a2 < A; ++a2)
                    if (a2 != a1)
                    {
                        // Compute the LHS.
                        const auto lhs = a1_et_val + origin_vals[a2] + destination_vals[a2] - a2_et_vals[a2] +
                                         a2_opposite_et_vals[a2];

                        // Store a cut candidate if violated.
                        if (is_gt(lhs, 1.0 + CUT_VIOLATION))
                        {
                            candidates_.push_back({lhs, a1, a2, t, a1_e, a2_opposite_et.et.e, origin, destination});
                        }
                    }
            }

    // Create the most violated cuts.
    Size num_separated_this_run = 0;
    num_separated_.set(0);
    std::sort(candidates_.begin(), candidates_.end(), [](const auto& a, const auto& b) { return a.lhs > b.lhs; });
    for (const auto& candidate : candidates_)
    {
        const auto& [lhs, a1, a2, t, a1_e, a2_opposite_e, origin, destination] = candidate;
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

void ExitEntryConflictSeparator::create_row(const ExitEntryConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, a1, a2, t, a1_e, a2_opposite_e, origin, destination] = candidate;

    // Include the opposite edge.
    Size32 num_a2_es = 1;
    Edge a2_es[10]{a2_opposite_e};

    // Include the five outgoing edges from the origin.
    if (const auto to_n = map.get_north(origin); map[to_n])
    {
        a2_es[num_a2_es++] = Edge{origin, Direction::NORTH};
    }
    if (const auto to_n = map.get_south(origin); map[to_n])
    {
        a2_es[num_a2_es++] = Edge{origin, Direction::SOUTH};
    }
    if (const auto to_n = map.get_west(origin); map[to_n])
    {
        a2_es[num_a2_es++] = Edge{origin, Direction::WEST};
    }
    if (const auto to_n = map.get_east(origin); map[to_n])
    {
        a2_es[num_a2_es++] = Edge{origin, Direction::EAST};
    }
    {
        a2_es[num_a2_es++] = Edge{origin, Direction::WAIT};
    }

    // Include the five incoming edges to the destination.
    if (const auto from_n = map.get_north(destination); from_n != origin && map[from_n])
    {
        debug_assert(num_a2_es < 10);
        a2_es[num_a2_es++] = Edge{from_n, Direction::SOUTH};
    }
    if (const auto from_n = map.get_south(destination); from_n != origin && map[from_n])
    {
        debug_assert(num_a2_es < 10);
        a2_es[num_a2_es++] = Edge{from_n, Direction::NORTH};
    }
    if (const auto from_n = map.get_west(destination); from_n != origin && map[from_n])
    {
        debug_assert(num_a2_es < 10);
        a2_es[num_a2_es++] = Edge{from_n, Direction::EAST};
    }
    if (const auto from_n = map.get_east(destination); from_n != origin && map[from_n])
    {
        debug_assert(num_a2_es < 10);
        a2_es[num_a2_es++] = Edge{from_n, Direction::WEST};
    }
    {
        debug_assert(num_a2_es < 10);
        a2_es[num_a2_es++] = Edge{destination, Direction::WAIT};
    }

    // Check.
#ifdef DEBUG
    {
        const auto& projection = problem_.projection();
        Float check_lhs = projection.find_agent_move_edgetime(a1, EdgeTime{a1_e, t});
        for (Size idx = 0; idx < num_a2_es; ++idx)
        {
            const EdgeTime et{a2_es[idx], t};
            if (et.d != Direction::WAIT)
            {
                check_lhs += projection.find_agent_move_edgetime(a2, et);
            }
            else
            {
                check_lhs += projection.find_agent_wait_edgetime(a2, et);
            }

            for (Size idx2 = idx + 1; idx2 < num_a2_es; ++idx2)
                debug_assert(a2_es[idx] != a2_es[idx2]);
        }
        debug_assert(is_eq(lhs, check_lhs));
    }
#endif

    // Print.
#ifdef PRINT_DEBUG
    {
        String a2_str;
        for (Size idx = 0; idx < num_a2_es; ++idx)
        {
            if (!a2_str.empty())
            {
                a2_str += ", ";
            }
            const auto e = a2_es[idx];
            a2_str += format_edgetime(EdgeTime{e, t}, map);
        }
        debugln("    Creating exit-entry conflict cut on {} for agent {} and {} for agent {} with LHS {}",
                format_edgetime(EdgeTime{a1_e, t}, map),
                a1,
                a2_str,
                a2,
                lhs);
    }
#endif

    // Create the row.
    auto name = fmt::format("exit_entry({},{},{},{},{})",
                            a1,
                            format_edgetime(EdgeTime{a1_e, t}, map),
                            a2,
                            format_node(origin, map),
                            format_node(destination, map));
    const auto array_size = sizeof(Edge) * num_a2_es;
    const auto object_size = sizeof(ExitEntryConstraint) + array_size;
    const auto hash_size = sizeof(Agent) * 2 + sizeof(Time) + sizeof(Edge) + sizeof(Size32) + array_size;
    auto constraint = Constraint::construct<ExitEntryConstraint>(object_size,
                                                                 hash_size,
                                                                 ConstraintFamily::ExitEntry,
                                                                 this,
                                                                 std::move(name),
                                                                 2,
                                                                 '<',
                                                                 1.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->a1) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    constraint->a1 = a1;
    constraint->a2 = a2;
    constraint->t = t;
    constraint->a1_e = a1_e;
    constraint->num_a2_es = num_a2_es;
    std::copy(a2_es, a2_es + num_a2_es, constraint->a2_es);
    master.add_row(std::move(constraint));
    ++num_added_;
}

void ExitEntryConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& exit_entry_constraint = *static_cast<const ExitEntryConstraint*>(&constraint);
    const auto a1 = exit_entry_constraint.a1;
    const auto a2 = exit_entry_constraint.a2;
    const auto t = exit_entry_constraint.t;
    const auto a1_e = exit_entry_constraint.a1_e;
    const auto num_a2_es = exit_entry_constraint.num_a2_es;
    const auto a2_es = exit_entry_constraint.a2_es;

    // Add the penalty on the edgetimes.
    auto& pricer = problem_.pricer();
    pricer.add_edgetime_penalty_one_agent(a1, EdgeTime{a1_e, t}, -dual);
    for (Size index = 0; index < num_a2_es; ++index)
    {
        const EdgeTime et{a2_es[index], t};
        pricer.add_edgetime_penalty_one_agent(a2, et, -dual);
    }
}

Float ExitEntryConflictSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& exit_entry_constraint = *static_cast<const ExitEntryConstraint*>(&constraint);
    const auto a1 = exit_entry_constraint.a1;
    const auto t = exit_entry_constraint.t;
    const auto a1_e = exit_entry_constraint.a1_e;
    const auto num_a2_es = exit_entry_constraint.num_a2_es;
    const auto a2_es = exit_entry_constraint.a2_es;

    // Calculate coefficient.
    if (a == a1)
    {
        return calculate_a1_coeff(a1_e, t, path);
    }
    else
    {
        debug_assert(a == exit_entry_constraint.a2);
        return calculate_a2_coeff(num_a2_es, a2_es, t, path);
    }
}

Bool ExitEntryConflictSeparator::calculate_a1_coeff(const Edge e, const Time t, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    return calculate_move_edgetime_coeff(EdgeTime{e, t}, path);
}

Bool ExitEntryConflictSeparator::calculate_a2_coeff(const Size32 num_es, const Edge* es, const Time t, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    for (Size idx = 0; idx < num_es; ++idx)
        if (calculate_edgetime_coeff(EdgeTime{es[idx], t}, path))
        {
            return true;
        }
    return false;
}
