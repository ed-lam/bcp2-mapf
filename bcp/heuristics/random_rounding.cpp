/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "heuristics/random_rounding.h"
#include "master/master.h"
#include "problem/map.h"
#include "problem/problem.h"
#include "types/debug.h"
#include "types/float_compare.h"
#include "types/tracy.h"
#include <limits>

#define TRACY_COLOUR tracy::Color::ColorType::Wheat

#define NUM_ITERATIONS (1)

RandomRounding::RandomRounding(const Instance& instance, Problem& problem) :
    PrimalHeuristic(instance, problem),
    rng_(RANDOM_SEED),
    agents_(instance_.num_agents()),
    nodetime_in_use_(),
    edgetime_in_use_(),
    target_crossed_(),
    target_blocked_()
{
    std::iota(agents_.begin(), agents_.end(), 0);
}

Pair<Cost, Vector<Variable*>> RandomRounding::execute()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    const auto A = instance_.num_agents();

    // Get the master problem.
    const auto& master = problem_.master();

    // Get all candidates.
    Vector<Vector<Variable*>> candidate_paths(A);
    Vector<Vector<Real64>> candidate_weights(A);
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable_ptr : master.agent_variable_ptrs(a))
        {
            const auto val = master.variable_primal_sol(*variable_ptr);
            if (is_gt(val, 0.0))
            {
                candidate_paths[a].push_back(variable_ptr);
                candidate_weights[a].push_back(val);
            }
        }

    // Randomly choose a path for every agent.
    Pair<Cost, Vector<Variable*>> output;
    auto& [sol_cost, sol] = output;
    sol.resize(A);
    for (Size64 iter = 0; iter < NUM_ITERATIONS; ++iter)
    {
        // Reset data structures.
        sol_cost = 0.0;
        nodetime_in_use_.clear();
        edgetime_in_use_.clear();
        for (Agent a = 0; a < A; ++a)
        {
            sol[a] = nullptr;

            const auto target = instance_.agents[a].target;
            target_blocked_[target] = TIME_MAX;
            target_crossed_[target] = -1;
        }
        DEBUG_ASSERT(target_blocked_.size() == A);
        DEBUG_ASSERT(target_crossed_.size() == A);

        // Randomly order the agents.
        std::shuffle(agents_.begin(), agents_.end(), rng_);

        // Choose a path for every agent.
        for (const auto a : agents_)
        {
            // Choose a path.
            DiscreteDistribution dist(candidate_weights[a].begin(), candidate_weights[a].end());
            const auto index = dist(rng_);

            // Get the path.
            DEBUG_ASSERT(index < candidate_paths[a].size());
            const auto& variable_ptr = candidate_paths[a][index];
            const auto& path = variable_ptr->path();

            // Check if the path is in conflict with paths already selected.
            {
                NodeTime nt{0, 0};
                for (; nt.t < path.size() - 1; ++nt.t)
                {
                    nt.n = path[nt.t].n;
                    if (nodetime_in_use_.find(nt) != nodetime_in_use_.end())
                    {
                        goto NEXT_CANDIDATE_PATH_SET;
                    }

                    if (auto it = target_blocked_.find(nt.n);
                        it != target_blocked_.end() && it->second <= nt.t)
                    {
                        goto NEXT_CANDIDATE_PATH_SET;
                    }

                    EdgeTime et{map.get_undirected_edge(path[nt.t]), nt.t};
                    if (edgetime_in_use_.find(et) != edgetime_in_use_.end())
                    {
                        goto NEXT_CANDIDATE_PATH_SET;
                    }
                }
                {
                    nt.n = path[nt.t].n;
                    if (target_crossed_.at(nt.n) >= nt.t)
                    {
                        goto NEXT_CANDIDATE_PATH_SET;
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
                    DEBUG_ASSERT(target_blocked_time == TIME_MAX);
                    target_blocked_time = nt.t;
                }
            }

            // Store the path.
            DEBUG_ASSERT(!sol[a]);
            sol[a] = variable_ptr;
            sol_cost += path.size() - 1;
        }
        ++num_feasible_;
        return output;
    NEXT_CANDIDATE_PATH_SET:;
    }

    // No feasible solution was found.
    sol_cost = COST_INF;
    return output;
}
