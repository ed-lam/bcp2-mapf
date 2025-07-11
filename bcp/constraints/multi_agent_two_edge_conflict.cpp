/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/multi_agent_two_edge_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::WebMaroon

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

MultiAgentTwoEdgeConflictSeparator::MultiAgentTwoEdgeConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    candidates_(),
    num_separated_(instance.agents.size())
{
}

void MultiAgentTwoEdgeConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for multi-agent two-edge conflicts");

    // Get the problem data.
    const Agent A = instance_.agents.size();
    const auto& map = instance_.map;

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    Array<Pair<EdgeTime, Float>, 4> canonical_et2s;
    Array<Pair<EdgeTime, const ProjectionValues*>, 4> other_et2s;
    Size num_et2s;
    for (Agent canonical = 0; canonical < A; ++canonical)
        for (const auto& [canonical_et1, canonical_et1_val] : projection.agent_move_edgetimes(canonical))
            if (!is_integral(canonical_et1_val))
            {
                debug_assert(canonical_et1.d != Direction::WAIT);

                // Get the time.
                const auto t = canonical_et1.t;

                // Get the first edge of the other agents.
                const EdgeTime other_et1{map.get_opposite_edge(canonical_et1.et.e), t};
                const auto& other_et1_vals = projection.find_list_fractional_move_edgetime(other_et1);
                const auto other_et1_val = other_et1_vals.total - other_et1_vals[canonical];

                // Get the second edge of the canonical agent.
                const auto canonical_et2_orig = map.get_destination(canonical_et1);
                num_et2s = 0;
                {
                    const EdgeTime canonical_et2{canonical_et2_orig, Direction::NORTH, t};
                    const auto dest = map.get_destination(canonical_et2);
                    if (map[dest] && dest != canonical_et1.n)
                    {
                        canonical_et2s[num_et2s].first = canonical_et2;
                        canonical_et2s[num_et2s].second =
                            projection.find_agent_move_edgetime(canonical, canonical_et2);

                        const auto other_et2 = EdgeTime{dest, Direction::SOUTH, t};
                        other_et2s[num_et2s].first = other_et2;
                        other_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(other_et2);

                        ++num_et2s;
                    }
                }
                {
                    const EdgeTime canonical_et2{canonical_et2_orig, Direction::SOUTH, t};
                    const auto dest = map.get_destination(canonical_et2);
                    if (map[dest] && dest != canonical_et1.n)
                    {
                        canonical_et2s[num_et2s].first = canonical_et2;
                        canonical_et2s[num_et2s].second =
                            projection.find_agent_move_edgetime(canonical, canonical_et2);

                        const auto other_et2 = EdgeTime{dest, Direction::NORTH, t};
                        other_et2s[num_et2s].first = other_et2;
                        other_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(other_et2);

                        ++num_et2s;
                    }
                }
                {
                    const EdgeTime canonical_et2{canonical_et2_orig, Direction::WEST, t};
                    const auto dest = map.get_destination(canonical_et2);
                    if (map[dest] && dest != canonical_et1.n)
                    {
                        canonical_et2s[num_et2s].first = canonical_et2;
                        canonical_et2s[num_et2s].second =
                            projection.find_agent_move_edgetime(canonical, canonical_et2);

                        const auto other_et2 = EdgeTime{dest, Direction::EAST, t};
                        other_et2s[num_et2s].first = other_et2;
                        other_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(other_et2);

                        ++num_et2s;
                    }
                }
                {
                    const EdgeTime canonical_et2{canonical_et2_orig, Direction::EAST, t};
                    const auto dest = map.get_destination(canonical_et2);
                    if (map[dest] && dest != canonical_et1.n)
                    {
                        canonical_et2s[num_et2s].first = canonical_et2;
                        canonical_et2s[num_et2s].second =
                            projection.find_agent_move_edgetime(canonical, canonical_et2);

                        const auto other_et2 = EdgeTime{dest, Direction::WEST, t};
                        other_et2s[num_et2s].first = other_et2;
                        other_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(other_et2);

                        ++num_et2s;
                    }
                }

                // Get the wait edge of both agents.
                const EdgeTime et3{canonical_et2_orig, Direction::WAIT, t};
                const auto& et3_vals = projection.find_list_fractional_wait_edgetime(et3);

                // Loop through the second edge of the canonical agent.
                for (Size index = 0; index < num_et2s; ++index)
                {
                    // Get the values of the remaining edges.
                    const auto& [canonical_et2, canonical_et2_val] = canonical_et2s[index];
                    const auto& [other_et2, other_et2_vals_ptr] = other_et2s[index];
                    const auto& other_et2_vals = *other_et2_vals_ptr;
                    const auto other_et2_val = other_et2_vals.total - other_et2_vals[canonical];
                    const auto canonical_et3_val = et3_vals[canonical];
                    const auto other_et3_val = et3_vals.total - canonical_et3_val;

                    // Store a cut candidate if violated.
                    const auto lhs = 2.0 * (canonical_et1_val + canonical_et2_val + canonical_et3_val) +
                                     other_et1_val + other_et2_val + other_et3_val;
                    if (is_gt(lhs, 2.0 + CUT_VIOLATION))
                    {
                        // Count the number of other agents involved in the cut.
                        Size num_other = 0;
                        for (Agent a = 0; a < A; ++a)
                            if (a != canonical)
                            {
                                num_other += is_gt(other_et1_vals[a], 0.0) +
                                             is_gt(other_et2_vals[a], 0.0) +
                                             is_gt(et3_vals[a], 0.0);
                            }

                        // Store the candidate if involving three or more agents.
                        if (num_other >= 2)
                        {
                            candidates_.push_back({lhs, num_other,
                                                   canonical, t,
                                                   {canonical_et1.et.e, canonical_et2.et.e, et3.et.e},
                                                   {other_et1.et.e, other_et2.et.e, et3.et.e}});
                        }
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
        const auto& [lhs, num_other, canonical, t, canonical_ets, other_ets] = candidate;
        auto& num_separated = num_separated_[{canonical, t}];
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

void MultiAgentTwoEdgeConflictSeparator::create_row(const MultiAgentTwoEdgeConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, num_other, canonical, t, canonical_es, other_es] = candidate;

    // Print.
    debugln("    Creating multi-agent two-edge conflict cut on {}, {} and {} for agent {} and {}, {} and {} for other "
            "agents with LHS {}",
            format_edgetime(EdgeTime{canonical_es[0], t}, instance_.map),
            format_edgetime(EdgeTime{canonical_es[1], t}, instance_.map),
            format_edgetime(EdgeTime{canonical_es[2], t}, instance_.map),
            canonical,
            format_edgetime(EdgeTime{other_es[0], t}, instance_.map),
            format_edgetime(EdgeTime{other_es[1], t}, instance_.map),
            format_edgetime(EdgeTime{other_es[2], t}, instance_.map),
            lhs);

    // Create the row.
    auto name = fmt::format("multi_agent_two_edge({},{},{},{},{},{},{},{})",
                            t,
                            canonical,
                            format_edge(canonical_es[0], instance_.map),
                            format_edge(canonical_es[1], instance_.map),
                            format_edge(canonical_es[2], instance_.map),
                            format_edge(other_es[0], instance_.map),
                            format_edge(other_es[1], instance_.map),
                            format_edge(other_es[2], instance_.map));
    constexpr auto object_size = sizeof(MultiAgentTwoEdgeConstraint);
    constexpr auto hash_size = sizeof(Agent) * 2 + sizeof(Time) + sizeof(Edge) * 6;
    auto constraint = Constraint::construct<MultiAgentTwoEdgeConstraint>(object_size,
                                                                         hash_size,
                                                                         ConstraintFamily::MultiAgentTwoEdge,
                                                                         this,
                                                                         std::move(name),
                                                                         1,
                                                                         '<',
                                                                         2.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->every_agent) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    // WARNING: Cannot add as a universal cut because each agent has costs on different edgetimes.
    constraint->every_agent = -1; // The master problem uses this -1 together with the 1 in construct() to determine the
                                  // agents to call this separator.
    constraint->canonical = canonical;
    constraint->t = t;
    constraint->canonical_es = canonical_es;
    constraint->other_es = other_es;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void MultiAgentTwoEdgeConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& multi_agent_two_edge_constraint = *static_cast<const MultiAgentTwoEdgeConstraint*>(&constraint);
    const auto canonical = multi_agent_two_edge_constraint.canonical;
    const auto t = multi_agent_two_edge_constraint.t;
    const auto& canonical_es = multi_agent_two_edge_constraint.canonical_es;
    const auto& other_es = multi_agent_two_edge_constraint.other_es;

    // Add the penalty on the edgetimes.
    auto& pricer = problem_.pricer();
    for (const auto e : canonical_es)
    {
        pricer.add_edgetime_penalty_one_agent(canonical, EdgeTime{e, t}, -2.0 * dual);
    }
    for (const auto e : other_es)
    {
        pricer.add_edgetime_penalty_all_except_one_agent(canonical, EdgeTime{e, t}, -dual);
    }
}

Float MultiAgentTwoEdgeConflictSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& multi_agent_two_edge_constraint = *static_cast<const MultiAgentTwoEdgeConstraint*>(&constraint);
    const auto canonical = multi_agent_two_edge_constraint.canonical;
    const auto t = multi_agent_two_edge_constraint.t;
    const auto& canonical_es = multi_agent_two_edge_constraint.canonical_es;
    const auto& other_es = multi_agent_two_edge_constraint.other_es;

    // Calculate coefficient.
    if (a == canonical)
    {
        return 2.0 * calculate_coeff(canonical_es, t, path);
    }
    else
    {
        return calculate_coeff(other_es, t, path);
    }
}

Float MultiAgentTwoEdgeConflictSeparator::calculate_coeff(const Array<Edge, 3>& es, const Time t, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    return calculate_move_edgetime_coeff(EdgeTime{es[0], t}, path) +
           calculate_move_edgetime_coeff(EdgeTime{es[1], t}, path) +
           calculate_wait_edgetime_coeff(EdgeTime{es[2], t}, path);
}
