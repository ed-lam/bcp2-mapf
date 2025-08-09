/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "heuristics/simple_rounding.h"
#include "master/master.h"
#include "problem/debug.h"
#include "problem/map.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"
#include <limits>

#define TRACY_COLOUR tracy::Color::ColorType::Tan

SimpleRounding::SimpleRounding(const Instance& instance, Problem& problem) :
    PrimalHeuristic(instance, problem),
    rng_(RANDOM_SEED),
    uniform_(0.0, 1.0),
    candidates_(),
    nodetime_in_use_(),
    edgetime_in_use_(),
    target_crossed_(),
    target_blocked_()
{
}

Pair<Cost, Vector<Variable*>> SimpleRounding::execute()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    const auto A = instance_.num_agents();

    // Get the master problem.
    const auto& master = problem_.master();

    // Get all candidates and sort them according to their LP solution value and tie-break randomly.
    candidates_.clear();
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable_ptr : master.agent_variable_ptrs(a))
        {
            const auto val = master.variable_primal_sol(*variable_ptr);
            if (is_gt(val, 0.0))
            {
                const UInt32 rounded_val = val * 10;
                candidates_.push_back(CandidatePath{uniform_(rng_), rounded_val, a, variable_ptr});
            }
        }
    std::sort(candidates_.begin(), candidates_.end(), [](const CandidatePath& lhs, const CandidatePath& rhs)
    {
        return std::tie(lhs.rounded_val, lhs.random) > std::tie(rhs.rounded_val, rhs.random);
    });

    // Try to insert the paths in the order above into a feasible solution.
    Pair<Cost, Vector<Variable*>> output;
    auto& [sol_cost, sol] = output;
    sol.resize(A);
    nodetime_in_use_.clear();
    edgetime_in_use_.clear();
    for (Agent a = 0; a < A; ++a)
    {
        const auto target = instance_.agents[a].target;
        target_blocked_[target] = TIME_MAX;
        target_crossed_[target] = -1;
    }
    debug_assert(target_blocked_.size() == A);
    debug_assert(target_crossed_.size() == A);
    for (const auto& [_, __, a, variable_ptr] : candidates_)
    {
        if (!sol[a])
        {
            // Get the path.
            const auto& path = variable_ptr->path();

            // Check if the path is in conflict with paths already selected.
            {
                NodeTime nt{0, 0};
                for (; nt.t < path.size() - 1; ++nt.t)
                {
                    nt.n = path[nt.t].n;
                    if (nodetime_in_use_.find(nt) != nodetime_in_use_.end())
                    {
                        goto NEXT_CANDIDATE_PATH;
                    }

                    if (auto it = target_blocked_.find(nt.n); it != target_blocked_.end() && it->second <= nt.t)
                    {
                        goto NEXT_CANDIDATE_PATH;
                    }

                    EdgeTime et{map.get_undirected_edge(path[nt.t]), nt.t};
                    if (edgetime_in_use_.find(et) != edgetime_in_use_.end())
                    {
                        goto NEXT_CANDIDATE_PATH;
                    }
                }
                {
                    nt.n = path[nt.t].n;
                    if (target_crossed_.at(nt.n) >= nt.t)
                    {
                        goto NEXT_CANDIDATE_PATH;
                    }
                }
            }

            // Store nodetimes and edgetimes of the selected path.
            {
                NodeTime nt{0, 0};
                for (; nt.t < path.size() - 1; ++nt.t)
                {
                    nt.n = path[nt.t].n;
                    nodetime_in_use_.insert(nt);

                    if (auto it = target_crossed_.find(nt.n); it != target_crossed_.end())
                    {
                        auto& target_crossed_time = it->second;
                        target_crossed_time = std::max(target_crossed_time, nt.t);
                    }

                    EdgeTime et{map.get_undirected_edge(path[nt.t]), nt.t};
                    edgetime_in_use_.insert(et);
                }
                {
                    nt.n = path[nt.t].n;
                    auto& target_blocked_time = target_blocked_.at(nt.n);
                    debug_assert(target_blocked_time == TIME_MAX);
                    target_blocked_time = nt.t;
                }
            }

            // Store the path.
            debug_assert(!sol[a]);
            sol[a] = variable_ptr;
            sol_cost += path.size() - 1;
        }
        NEXT_CANDIDATE_PATH:;
    }

    // Check if a path is found for all agents.
    for (const auto& path_ptr : sol)
        if (!path_ptr)
        {
            sol_cost = INF;
            break;
        }

    // Return.
    num_feasible_ += (sol_cost != INF);
    return output;
}
