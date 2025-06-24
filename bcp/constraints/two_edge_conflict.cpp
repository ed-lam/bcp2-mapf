/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/two_edge_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::Tomato

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

TwoEdgeConflictSeparator::TwoEdgeConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    candidates_(),
    num_separated_(instance.agents.size(), instance.agents.size())
{
}

void TwoEdgeConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for two-edge conflicts");

    // Get the problem data.
    const Agent A = instance_.agents.size();
    const auto& map = instance_.map;

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    Array<Pair<EdgeTime, Float>, 4> a1_et2s;
    Array<Pair<EdgeTime, const ProjectionValues*>, 4> a2_et2s;
    Size num_et2s;
    for (Agent a1 = 0; a1 < A - 1; ++a1)
        for (const auto& [a1_et1, a1_et1_val] : projection.agent_move_edgetimes(a1))
            if (!is_integral(a1_et1_val))
            {
                debug_assert(a1_et1.d != Direction::WAIT);

                // Get the time.
                const auto t = a1_et1.t;

                // Get the first edge of agent 2.
                const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.et.e), t};
                const auto& a2_et1_vals = projection.find_list_fractional_move_edgetime(a2_et1);

                // Get the second edge of agent 1.
                const auto a1_e2_orig = map.get_destination(a1_et1);
                num_et2s = 0;
                {
                    const EdgeTime et{a1_e2_orig, Direction::NORTH, t};
                    const auto dest = map.get_destination(et);
                    if (map[dest] && dest != a1_et1.n)
                    {
                        a1_et2s[num_et2s].first = et;
                        a1_et2s[num_et2s].second = projection.find_agent_move_edgetime(a1, et);

                        a2_et2s[num_et2s].first = EdgeTime{dest, Direction::SOUTH, t};
                        a2_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(a2_et2s[num_et2s].first);

                        ++num_et2s;
                    }
                }
                {
                    const EdgeTime et{a1_e2_orig, Direction::SOUTH, t};
                    const auto dest = map.get_destination(et);
                    if (map[dest] && dest != a1_et1.n)
                    {
                        a1_et2s[num_et2s].first = et;
                        a1_et2s[num_et2s].second = projection.find_agent_move_edgetime(a1, et);

                        a2_et2s[num_et2s].first = EdgeTime{dest, Direction::NORTH, t};
                        a2_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(a2_et2s[num_et2s].first);

                        ++num_et2s;
                    }
                }
                {
                    const EdgeTime et{a1_e2_orig, Direction::WEST, t};
                    const auto dest = map.get_destination(et);
                    if (map[dest] && dest != a1_et1.n)
                    {
                        a1_et2s[num_et2s].first = et;
                        a1_et2s[num_et2s].second = projection.find_agent_move_edgetime(a1, et);

                        a2_et2s[num_et2s].first = EdgeTime{dest, Direction::EAST, t};
                        a2_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(a2_et2s[num_et2s].first);

                        ++num_et2s;
                    }
                }
                {
                    const EdgeTime et{a1_e2_orig, Direction::EAST, t};
                    const auto dest = map.get_destination(et);
                    if (map[dest] && dest != a1_et1.n)
                    {
                        a1_et2s[num_et2s].first = et;
                        a1_et2s[num_et2s].second = projection.find_agent_move_edgetime(a1, et);

                        a2_et2s[num_et2s].first = EdgeTime{dest, Direction::WEST, t};
                        a2_et2s[num_et2s].second = &projection.find_list_fractional_move_edgetime(a2_et2s[num_et2s].first);

                        ++num_et2s;
                    }
                }

                // Get the wait edge of both agents.
                const EdgeTime a12_et3{a1_e2_orig, Direction::WAIT, t};
                const auto& a12_et3_vals = projection.find_list_fractional_wait_edgetime(a12_et3);

                // Loop through the second edge of agent 1.
                for (Size index = 0; index < num_et2s; ++index)
                {
                    // Get the second edge of both agents.
                    const auto& [a1_et2, a1_et2_val] = a1_et2s[index];
                    const auto& [a2_et2, a2_et2_vals_ptr] = a2_et2s[index];
                    const auto& a2_et2_vals = *a2_et2_vals_ptr;

                    // Stop processing if a cut cannot possibly be violated.
                    const auto potential_lhs = a1_et1_val + a1_et2_val +
                                               (a2_et1_vals.total - a2_et1_vals[a1]) +
                                               (a2_et2_vals.total - a2_et2_vals[a1]) +
                                               a12_et3_vals.total;
                    if (is_le(potential_lhs, 1.0 + CUT_VIOLATION))
                    {
                        continue;
                    }

                    // Loop through the second agent.
                    for (Agent a2 = a1 + 1; a2 < A; ++a2)
                    {
                        // Store a cut if violated.
                        const auto lhs = a1_et1_val + a1_et2_val + a2_et1_vals[a2] + a2_et2_vals[a2] +
                                         a12_et3_vals[a1] + a12_et3_vals[a2];
                        if (is_gt(lhs, 1.0 + CUT_VIOLATION))
                        {
                            candidates_.push_back({lhs,
                                                   a1, a2, t,
                                                   {a1_et1.et.e, a1_et2.et.e, a12_et3.et.e},
                                                   {a2_et1.et.e, a2_et2.et.e, a12_et3.et.e}});
                        }
                    }
                }
            }

    // Create the most violated cuts.
    Size num_separated_this_run = 0;
    num_separated_.set(0);
    std::sort(candidates_.begin(), candidates_.end(), [](const auto& a, const auto& b) { return a.lhs > b.lhs; });
    for (const auto& candidate : candidates_)
    {
        const auto& [lhs, a1, a2, t, a1_es, a2_es] = candidate;
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

void TwoEdgeConflictSeparator::create_row(const TwoEdgeConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, a1, a2, t, a1_es, a2_es] = candidate;

    // Print.
    debugln("    Creating two-edge conflict cut on {}, {} and {} for agent {} and {}, {} and {} for agent {} "
            "with LHS {}",
            format_edgetime(EdgeTime{a1_es[0], t}, instance_.map),
            format_edgetime(EdgeTime{a1_es[1], t}, instance_.map),
            format_edgetime(EdgeTime{a1_es[2], t}, instance_.map),
            a1,
            format_edgetime(EdgeTime{a2_es[0], t}, instance_.map),
            format_edgetime(EdgeTime{a2_es[1], t}, instance_.map),
            format_edgetime(EdgeTime{a2_es[2], t}, instance_.map),
            a2,
            lhs);

    // Create the row.
    auto name = fmt::format("two_edge({},{},{},{},{},{},{},{},{})",
                            t,
                            a1,
                            format_edge(a1_es[0], instance_.map),
                            format_edge(a1_es[1], instance_.map),
                            format_edge(a1_es[2], instance_.map),
                            a2,
                            format_edge(a2_es[0], instance_.map),
                            format_edge(a2_es[1], instance_.map),
                            format_edge(a2_es[2], instance_.map));
    constexpr auto object_size = sizeof(TwoEdgeConstraint);
    constexpr auto hash_size = sizeof(Agent) * 2 + sizeof(Time) + sizeof(Edge) * 6;
    auto constraint = Constraint::construct<TwoEdgeConstraint>(object_size,
                                                               hash_size,
                                                               ConstraintFamily::TwoEdge,
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
    constraint->a1_es = a1_es;
    constraint->a2_es = a2_es;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void TwoEdgeConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& two_edge_constraint = *static_cast<const TwoEdgeConstraint*>(&constraint);
    const auto a1 = two_edge_constraint.a1;
    const auto a2 = two_edge_constraint.a2;
    const auto t = two_edge_constraint.t;
    const auto& a1_es = two_edge_constraint.a1_es;
    const auto& a2_es = two_edge_constraint.a2_es;

    // Add the penalty on the edgetimes.
    auto& pricer = problem_.pricer();
    for (const auto e : a1_es)
    {
        pricer.add_edgetime_penalty_one_agent(a1, EdgeTime{e, t}, -dual);
    }
    for (const auto e : a2_es)
    {
        pricer.add_edgetime_penalty_one_agent(a2, EdgeTime{e, t}, -dual);
    }
}

Float TwoEdgeConflictSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& two_edge_constraint = *static_cast<const TwoEdgeConstraint*>(&constraint);
    const auto a1 = two_edge_constraint.a1;
    const auto a2 = two_edge_constraint.a2;
    const auto t = two_edge_constraint.t;
    const auto& a1_es = two_edge_constraint.a1_es;
    const auto& a2_es = two_edge_constraint.a2_es;

    // Calculate coefficient.
    return (a == a1 && calculate_coeff(a1_es, t, path)) || (a == a2 && calculate_coeff(a2_es, t, path));
}

Float TwoEdgeConflictSeparator::calculate_coeff(const Array<Edge, 3>& es, const Time t, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    return calculate_move_edgetime_coeff(EdgeTime{es[0], t}, path) +
           calculate_move_edgetime_coeff(EdgeTime{es[1], t}, path) +
           calculate_wait_edgetime_coeff(EdgeTime{es[2], t}, path);
}
