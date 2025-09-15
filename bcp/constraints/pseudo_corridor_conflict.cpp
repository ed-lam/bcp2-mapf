/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/pseudo_corridor_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::DeepPink

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

PseudoCorridorConflictSeparator::PseudoCorridorConflictSeparator(const Instance& instance,
                                                                 Problem& problem) :
    Separator(instance, problem),
    candidates_(),
    num_separated_(instance.num_agents(), instance.num_agents())
{
}

void PseudoCorridorConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN("Starting separator for pseudo-corridor conflicts");

    // Get the problem data.
    const auto A = instance_.num_agents();
    const auto& map = instance_.map;

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    for (Agent a1 = 0; a1 < A; ++a1)
        for (const auto& [a1_et1, a1_et1_val] : projection.agent_move_edgetimes(a1))
            if (!is_integral(a1_et1_val))
            {
                ZoneScopedNC("Get values", TRACY_COLOUR);

                DEBUG_ASSERT(a1_et1.d != Direction::WAIT);

                // Get the time.
                const auto t = a1_et1.t;

                // Get the second edge of agent 1.
                const EdgeTime a1_et2{a1_et1.e(), t + 1};
                const auto a1_et2_val = projection.find_agent_move_edgetime(a1, a1_et2);

                // Get the first edge of agent 2.
                const EdgeTime a2_et1{map.get_opposite_edge(a1_et1.e()), t};
                const auto& a2_et1_vals = projection.find_list_fractional_move_edgetime(a2_et1);

                // Get the second edge of agent 2.
                const EdgeTime a2_et2{a2_et1.e(), t + 1};
                const auto& a2_et2_vals = projection.find_list_fractional_move_edgetime(a2_et2);

                // Get the candidate third edge of agent 1.
                const EdgeTime a1_et3{a2_et1.n, Direction::WAIT, t};
                const auto a1_et3_val = projection.find_agent_wait_edgetime(a1, a1_et3);

                // Get the candidate fourth edge of agent 1.
                const EdgeTime a1_et4{a1_et2.n, Direction::WAIT, t + 1};
                const auto a1_et4_val = projection.find_agent_wait_edgetime(a1, a1_et4);

                // Stop processing if a cut cannot possibly be violated.
                const auto potential_lhs = a1_et1_val + a1_et2_val + a1_et3_val + a1_et4_val +
                                           (a2_et1_vals.total - a2_et1_vals[a1]) +
                                           (a2_et2_vals.total - a2_et2_vals[a1]);
                if (is_le(potential_lhs, 1.0 + CUT_VIOLATION))
                {
                    continue;
                }

                // Loop through the second agent.
                for (Agent a2 = 0; a2 < A; ++a2)
                    if (a2 != a1)
                    {
                        ZoneScopedNC("Check LHS", TRACY_COLOUR);

                        // Get values of the edges of agent 2.
                        const auto a2_et1_val = a2_et1_vals[a2];
                        const auto a2_et2_val = a2_et2_vals[a2];

                        // Compute the LHS.
                        const auto lhs = a1_et1_val + a1_et2_val + a1_et3_val + a1_et4_val +
                                         a2_et1_val + a2_et2_val;

                        // Store a cut if violated.
                        if (is_gt(lhs, 1.0 + CUT_VIOLATION))
                        {
                            ZoneScopedNC("Create candidate", TRACY_COLOUR);

                            candidates_.push_back(
                                {lhs, a1, a2, {a1_et1, a1_et2, a1_et3, a1_et4}, {a2_et1, a2_et2}});
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
        const auto& [lhs, a1, a2, a1_ets, a2_ets] = candidate;
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

void PseudoCorridorConflictSeparator::create_row(const PseudoCorridorConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, a1, a2, a1_ets, a2_ets] = candidate;

    // Print.
    DEBUGLN("    Creating pseudo-corridor conflict cut on {}, {}, {} and {} for agent {} and {} "
            "and {} for agent {} with LHS {}",
            format_edgetime(a1_ets[0], map),
            format_edgetime(a1_ets[1], map),
            format_edgetime(a1_ets[2], map),
            format_edgetime(a1_ets[3], map),
            a1,
            format_edgetime(a2_ets[0], map),
            format_edgetime(a2_ets[1], map),
            a2,
            lhs);

    // Create the row.
    auto name = fmt::format("pseudo_corridor({},{},{},{},{},{},{},{})",
                            a1,
                            format_edgetime(a1_ets[0], map),
                            format_edgetime(a1_ets[1], map),
                            format_edgetime(a1_ets[2], map),
                            format_edgetime(a1_ets[3], map),
                            a2,
                            format_edgetime(a2_ets[0], map),
                            format_edgetime(a2_ets[1], map));
    constexpr auto data_size = sizeof(ConstraintData);
    constexpr auto hash_size = data_size;
    auto constraint = Constraint::construct(
        '<', 1.0, 2, data_size, hash_size, &apply_in_pricer, &get_coeff, name);
    auto data = new (constraint->data()) ConstraintData;
    data->a1 = a1;
    data->a2 = a2;
    data->a1_ets = a1_ets;
    data->a2_ets = a2_ets;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void PseudoCorridorConflictSeparator::apply_in_pricer(const Constraint& constraint,
                                                      const Real64 dual, Pricer& pricer)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [a1, a2, a1_ets, a2_ets] =
        *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Add the dual solution to the reduced cost function.
    for (const auto et : a1_ets)
    {
        pricer.add_edgetime_penalty_one_agent(a1, et, -dual);
    }
    for (const auto et : a2_ets)
    {
        pricer.add_edgetime_penalty_one_agent(a2, et, -dual);
    }
}

Real64 PseudoCorridorConflictSeparator::get_coeff(const Constraint& constraint, const Agent a,
                                                  const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [a1, a2, a1_ets, a2_ets] =
        *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Calculate coefficient.
    return (a == a1 && calculate_a1_coeff(a1_ets, path)) ||
           (a == a2 && calculate_a2_coeff(a2_ets, path));
}

Bool PseudoCorridorConflictSeparator::calculate_a1_coeff(const Array<EdgeTime, 4>& ets,
                                                         const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    return calculate_move_edgetime_coeff(ets[0], path) +
           calculate_move_edgetime_coeff(ets[1], path) +
           calculate_wait_edgetime_coeff(ets[2], path) +
           calculate_wait_edgetime_coeff(ets[3], path);
}

Bool PseudoCorridorConflictSeparator::calculate_a2_coeff(const Array<EdgeTime, 2>& ets,
                                                         const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    return calculate_move_edgetime_coeff(ets[0], path) +
           calculate_move_edgetime_coeff(ets[1], path);
}
