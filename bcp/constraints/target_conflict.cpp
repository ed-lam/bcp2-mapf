/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/target_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::Red

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

TargetConflictSeparator::TargetConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    candidates_(),
    num_separated_(instance.num_agents(), instance.num_agents())
{
}

void TargetConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN("Starting separator for target conflicts");

    // Get the solution.
    const auto& projection = problem_.projection();
    const auto& target_finishing = projection.target_finishing();

    // Find candidates.
    candidates_.clear();
    for (const auto& [ant, crossing_val] : projection.agent_target_crossing())
    {
        // Get the agents and values.
        const auto crossing_agent = ant.a;
        const auto target = ant.n;
        const auto t = ant.t;
        const auto& [finishing_agent, finishing_vals_ptr] = target_finishing.at(target);
        const auto& finishing_vals = *finishing_vals_ptr;
        // WARNING: indices in finishing_vals are offset 1 later

        // Check for violation.
        DEBUG_ASSERT(t >= 1);
        DEBUG_ASSERT(finishing_vals.size() >= 1);
        const auto finishing_val =
            finishing_vals[std::min<Size64>(finishing_vals.size() - 1, t - 1)];
        const auto lhs = finishing_val + crossing_val;
        if (is_gt(lhs, 1.0 + CUT_VIOLATION))
        {
            candidates_.push_back({lhs, finishing_agent, crossing_agent, NodeTime{target, t}});
        }
    }

    // Create the most violated cuts.
    Size64 num_separated_this_run = 0;
    num_separated_.set(0);
    std::sort(candidates_.begin(),
              candidates_.end(),
              [](const auto& a, const auto& b)
              { return std::tie(a.lhs, b.nt.t) > std::tie(b.lhs, a.nt.t); });
    for (const auto& candidate : candidates_)
    {
        const auto& [lhs, finishing_agent, crossing_agent, nt] = candidate;
        auto& num_separated = num_separated_(finishing_agent, crossing_agent);
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

void TargetConflictSeparator::create_row(const TargetConstraintCandidate& candidate)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [lhs, finishing_agent, crossing_agent, nt] = candidate;

    // Print.
    DEBUGLN("    Creating target constraint for finishing agent {} at {} and crossing agent {} "
            "with LHS {}",
            finishing_agent,
            format_nodetime(nt, instance_.map),
            crossing_agent,
            lhs);

    // Create the row.
    auto name = fmt::format(
        "target({},{},{})", finishing_agent, format_nodetime(nt, instance_.map), crossing_agent);
    constexpr auto data_size = sizeof(ConstraintData);
    constexpr auto hash_size = data_size;
    auto constraint = Constraint::construct(
        '<', 1.0, 2, data_size, hash_size, &apply_in_pricer, &get_coeff, name);
    auto data = new (constraint->data()) ConstraintData;
    data->finishing_agent = finishing_agent;
    data->crossing_agent = crossing_agent;
    data->nt = nt;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void TargetConflictSeparator::apply_in_pricer(const Constraint& constraint, const Real64 dual,
                                              Pricer& pricer)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [finishing_agent, crossing_agent, nt] =
        *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Add the penalty if the finishing agent finishes at or before time t.
    pricer.add_end_penalty_one_agent(finishing_agent, 0, nt.t + 1, -dual);

    // Add the penalty if the crossing agent visits the finshing agent's target at or after time t.
    pricer.add_once_off_penalty_one_agent(crossing_agent, nt, -dual);
}

Real64 TargetConflictSeparator::get_coeff(const Constraint& constraint, const Agent a,
                                          const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [finishing_agent, crossing_agent, nt] =
        *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Calculate coefficient.
    if (a == finishing_agent)
    {
        return calculate_finishing_agent_coeff(nt, path);
    }
    else if (a == crossing_agent)
    {
        return calculate_crossing_agent_coeff(nt, path);
    }
    else
    {
        return 0.0;
    }
}

Real64 TargetConflictSeparator::calculate_finishing_agent_coeff(const NodeTime nt, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    const Time finishing_time = path.size() - 1;
    return (finishing_time <= nt.t);
}

Real64 TargetConflictSeparator::calculate_crossing_agent_coeff(const NodeTime nt, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    const Time finishing_time = path.size() - 1;

    // Targets must be unique, so the crossing agent cannot finish at n, so no need to test <=
    // finishing_time.
    Real64 coeff = 0.0;
    for (Time t = nt.t; t < finishing_time; ++t)
        if (path[t].n == nt.n)
        {
            coeff = 1.0;
            break;
        }
    DEBUG_ASSERT(path[finishing_time].n != nt.n);
    return coeff;
}
