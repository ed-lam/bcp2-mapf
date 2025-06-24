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
    num_separated_(instance.agents.size(), instance.agents.size())
{
}

void TargetConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for target conflicts");

    // Get the problem data.
    const Agent A = instance_.agents.size();
    auto& master = problem_.master();

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find candidates.
    candidates_.clear();
    for (const auto& [an, finish_vals] : projection.agent_node_finish())
    {
        // Get the agent and target.
        const auto finishing_agent = an.a;
        const auto target = an.n;

        // Check all finishing times.
        for (Time finish_time = 0; finish_time < finish_vals.size() - 1; ++finish_time)
        {
            // Get the contribution by the finishing agent.
            const auto finish_val = finish_vals[finish_time + 1];
#ifdef DEBUG
            {
                Float check_finishing_val = 0.0;
                for (const auto& variable : master.agent_variables(finishing_agent))
                {
                    const auto& path = variable.path();
                    const auto path_finish_time = path.size() - 1;
                    const auto val = master.variable_primal_sol(variable);
                    check_finishing_val += (path_finish_time <= finish_time) * val;
                }
                debug_assert(is_eq(check_finishing_val, finish_val));
            }
#endif

            // Proceed for times when the agent finishes with a different finishing value.
            if (is_gt(finish_val, 0.0) && is_ne(finish_val, finish_vals[finish_time]))
            {
                // Calculate the contribution by the crossing agent.
                for (Agent crossing_agent = 0; crossing_agent < A; ++crossing_agent)
                    if (crossing_agent != finishing_agent)
                    {
                        // Calculate the contribution.
                        Float crossing_val = 0.0;
                        for (const auto& variable : master.agent_variables(crossing_agent))
                        {
                            const auto val = master.variable_primal_sol(variable);
                            if (is_gt(val, 0.0))
                            {
                                const auto& path = variable.path();
                                for (Time t = finish_time; t < path.size() - 1; ++t)
                                    if (path[t].n == target)
                                    {
                                        crossing_val += val;
                                        break;
                                    }
                            }
                        }

                        // Store a cut candidate if violated.
                        const auto lhs = finish_val + crossing_val;
                        if (is_gt(lhs, 1.0 + CUT_VIOLATION))
                        {
                            debug_assert(is_gt(finish_val, 0.0));
                            debug_assert(is_gt(crossing_val, 0.0));
                            candidates_.push_back({lhs,
                                                   finishing_agent,
                                                   crossing_agent,
                                                   NodeTime{target, finish_time}});
                        }
                    }
            }
        }
    }

    // Create the most violated cuts.
    Size num_separated_this_run = 0;
    num_separated_.set(0);
    std::sort(candidates_.begin(),
              candidates_.end(),
              [](const auto& a, const auto& b) { return std::tie(a.lhs, b.nt.t) > std::tie(b.lhs, a.nt.t); });
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
    debugln("    Creating target constraint for finishing agent {} at {} and crossing agent {} with LHS {}",
            finishing_agent,
            format_nodetime(nt, instance_.map),
            crossing_agent,
            lhs);

    // Create the row.
    auto name = fmt::format("target({},{},{})", finishing_agent, format_nodetime(nt, instance_.map), crossing_agent);
    constexpr auto object_size = sizeof(TargetConstraint);
    constexpr auto hash_size = sizeof(Agent) * 2 + sizeof(NodeTime);
    auto constraint = Constraint::construct<TargetConstraint>(object_size,
                                                              hash_size,
                                                              ConstraintFamily::Target,
                                                              this,
                                                              std::move(name),
                                                              2,
                                                              '<',
                                                              1.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->finishing_agent) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    constraint->finishing_agent = finishing_agent;
    constraint->crossing_agent = crossing_agent;
    constraint->nt = nt;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void TargetConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& target_constraint = *static_cast<const TargetConstraint*>(&constraint);
    const auto finishing_agent = target_constraint.finishing_agent;
    const auto crossing_agent = target_constraint.crossing_agent;
    const auto nt = target_constraint.nt;

    // Add the penalty if the finishing agent finishes at or before time t.
    auto& pricer = problem_.pricer();
    pricer.add_end_penalty_one_agent(finishing_agent, 0, nt.t + 1, -dual);

    // Add the penalty if the crossing agent visits the finshing agent's target at or after time t.
    pricer.add_once_off_penalty_one_agent(crossing_agent, nt, -dual);
}

Float TargetConflictSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& target_constraint = *static_cast<const TargetConstraint*>(&constraint);
    const auto finishing_agent = target_constraint.finishing_agent;
    const auto crossing_agent = target_constraint.crossing_agent;
    const auto nt = target_constraint.nt;

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

Float TargetConflictSeparator::calculate_finishing_agent_coeff(const NodeTime nt, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    const Time finish_time = path.size() - 1;
    return (finish_time <= nt.t);
}

Float TargetConflictSeparator::calculate_crossing_agent_coeff(const NodeTime nt, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    const Time finish_time = path.size() - 1;

    // Targets must be unique, so the crossing agent cannot finish at n, so no need to test <= finish_time.
    Float coeff = 0.0;
    for (Time t = nt.t; t < finish_time; ++t)
        if (path[t].n == nt.n)
        {
            coeff = 1.0;
            break;
        }
    debug_assert(path[finish_time].n != nt.n);
    return coeff;
}
