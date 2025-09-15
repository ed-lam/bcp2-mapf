/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "bbtree/length_brancher.h"
#include "constraints/rectangle_clique_conflict.h"
#include "constraints/rectangle_knapsack_conflict.h"
#include "master/master.h"
#include "problem/problem.h"
#include "types/debug.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::SeaGreen

struct FinishTimeBound
{
    Time first;
    Time last;
};

Decisions LengthBrancher::branch()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();
    auto& master = problem_.master();

    // Find the earliest and latest time that an agent reaches its target.
    Vector<FinishTimeBound> finish_time_bounds(A, {TIME_MAX, 0});
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : master.agent_variables(a))
        {
            const auto val = master.variable_primal_sol(variable);
            if (is_gt(val, 0.0))
            {
                const auto& path = variable.path();
                const Time finish_time = path.size() - 1;
                finish_time_bounds[a].first = std::min(finish_time_bounds[a].first, finish_time);
                finish_time_bounds[a].last = std::max(finish_time_bounds[a].last, finish_time);
            }
        }

    // Prepare decision.
    Decisions output;
    Time best_t = TIME_MAX;
    Time best_diff = 0;
    Agent best_a = -1;

    // Choose an agent in a rectangle conflict.
    for (const auto& constraint : master.subset_constraints())
    {
        const auto name = constraint.name();
        const auto slack = master.constraint_slack(constraint);
        if (is_eq(slack, 0.0) && std::string_view(name).starts_with("rectangle"))
        {
            // Get the constraint data.
#ifdef USE_RECTANGLE_CLIQUE_CUTS
            const auto& data =
                *reinterpret_cast<const RectangleCliqueConflictSeparator::ConstraintData*>(
                    constraint.data());
#else
            const auto& data =
                *reinterpret_cast<const RectangleKnapsackConflictSeparator::ConstraintData*>(
                    constraint.data());
#endif
            const auto a1 = data.a1;
            const auto a2 = data.a2;

            // Check the first agent.
            {
                const auto earliest_finish = finish_time_bounds[a1].first;
                const auto diff = finish_time_bounds[a1].last - finish_time_bounds[a1].first;
                if (diff >= 1 &&
                    ((earliest_finish < best_t) || (earliest_finish == best_t && diff > best_diff)))
                {
                    best_t = earliest_finish;
                    best_diff = diff;
                    best_a = a1;
                }
            }

            // Check the second agent.
            {
                const auto earliest_finish = finish_time_bounds[a2].first;
                const auto diff = finish_time_bounds[a2].last - finish_time_bounds[a2].first;
                if (diff >= 1 &&
                    ((earliest_finish < best_t) || (earliest_finish == best_t && diff > best_diff)))
                {
                    best_t = earliest_finish;
                    best_diff = diff;
                    best_a = a2;
                }
            }
        }
    }
    if (best_a >= 0)
    {
        DEBUGLN("Length brancher branching on agent {} in rectangle conflict that is finishing "
                "between {} and {} to finish before {} or after {} in B&B node {}",
                best_a,
                finish_time_bounds[best_a].first,
                finish_time_bounds[best_a].last,
                best_t,
                best_t + 1,
                problem_.bbtree().current_id());
        goto BRANCH;
    }

    // Choose from remaining choices.
    for (Agent a = 0; a < A; ++a)
    {
        const auto earliest_finish = finish_time_bounds[a].first;
        const auto diff = finish_time_bounds[a].last - finish_time_bounds[a].first;
        if (diff >= 1 &&
            ((earliest_finish < best_t) || (earliest_finish == best_t && diff > best_diff)))
        {
            best_t = earliest_finish;
            best_diff = diff;
            best_a = a;
        }
    }
    if (best_a >= 0)
    {
        DEBUGLN("Length brancher branching on agent {} that is finishing between {} and {} to "
                "finish before {} or after {} in B&B node {}",
                best_a,
                finish_time_bounds[best_a].first,
                finish_time_bounds[best_a].last,
                best_t,
                best_t + 1,
                problem_.bbtree().current_id());
        goto BRANCH;
    }

    // Failed to find a decision.
    return output;

// Store the decisions.
BRANCH:
    output = {std::make_unique<LengthBrancherData>(), std::make_unique<LengthBrancherData>()};
    auto left_decision = static_cast<LengthBrancherData*>(output.first.get());
    auto right_decision = static_cast<LengthBrancherData*>(output.second.get());
    left_decision->a = best_a;
    left_decision->nt = NodeTime{instance_.agents[best_a].target, best_t};
    left_decision->dir = BranchDirection::Down;
    right_decision->a = best_a;
    right_decision->nt = NodeTime{instance_.agents[best_a].target, best_t + 1};
    right_decision->dir = BranchDirection::Up;
    num_added_ += 2;
    return output;
}

void LengthBrancher::add_pricing_costs(const BrancherData* const data)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [finish_a, nt, dir] = *static_cast<const LengthBrancherData*>(data);
    DEBUG_ASSERT(nt.n == instance_.agents[finish_a].target);

    // Add the penalty.
    auto& pricer = problem_.pricer();
    if (dir == BranchDirection::Down)
    {
        pricer.add_end_penalty_one_agent(finish_a, nt.t + 1, TIME_MAX, COST_INF);
        pricer.add_once_off_penalty_all_except_one_agent(finish_a, nt, COST_INF);
    }
    else
    {
        pricer.add_end_penalty_one_agent(finish_a, 0, nt.t, COST_INF);
    }
}

void LengthBrancher::disable_vars(const BrancherData* const data)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [finish_a, nt, dir] = *static_cast<const LengthBrancherData*>(data);
    DEBUG_ASSERT(nt.n == instance_.agents[finish_a].target);

    // Disable paths of the same agent if it doesn't reach the target at the required times.
    DEBUGLN("Length brancher disabling paths incompatible with decision on agent "
            "{} finishing at or {} time {} in B&B "
            "node {}:",
            finish_a,
            dir == BranchDirection::Down ? "before" : "after",
            nt.t,
            problem_.bbtree().current_id());
    if (dir == BranchDirection::Down)
    {
        // Disable paths of the same agent that finishes too late. Also disable paths of other
        // agents that visits the target node of the first agent.
        for (Agent a = 0; a < A; ++a)
            for (const auto& variable : master.agent_variables(a))
            {
                const auto& path = variable.path();
                for (Time t = path.size() - 1; t >= nt.t + (a == finish_a); --t)
                    if (path[t].n == nt.n)
                    {
                        DEBUGLN("    Agent {:2d}, path {}",
                                a,
                                format_path_with_time_spaced(path, instance_.map));
                        master.disable_variable(variable);
                        break;
                    }
            }
    }
    else
    {
        for (const auto& variable : master.agent_variables(finish_a))
        {
            const auto& path = variable.path();
            if (const auto finish_time = path.size() - 1; finish_time < nt.t)
            {
                DEBUGLN("    Agent {:2d}, path {}",
                        finish_a,
                        format_path_with_time_spaced(path, instance_.map));
                master.disable_variable(variable);
            }
        }
    }
    DEBUGLN("");
}

void LengthBrancher::print(const BrancherData* const data) const
{
    // Get the constraint data.
    const auto& [finish_a, nt, dir] = *static_cast<const LengthBrancherData*>(data);

    // Print.
    PRINTLN("    Agent {} finishing at or {} time {}",
            finish_a,
            dir == BranchDirection::Down ? "before" : "after",
            nt.t);
}
