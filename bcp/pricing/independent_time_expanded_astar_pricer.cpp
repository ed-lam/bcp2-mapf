/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "pricing/independent_time_expanded_astar_pricer.h"
#include "problem/problem.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::DodgerBlue

IndependentTimeExpandedAStarPricer::IndependentTimeExpandedAStarPricer(const Instance& instance, Problem& problem) :
    problem_(problem),
    distance_heuristic_(instance.map),

    status_(),
    partial_pricing_(instance, problem),
    infeasible_solvers_(),
    feasible_solvers_()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& agents = instance.agents;
    const Agent A = agents.size();

    // Create pricers.
    infeasible_solvers_.reserve(A);
    feasible_solvers_.reserve(A);
    for (Agent a = 0; a < A; ++a)
    {
        infeasible_solvers_.emplace_back(instance, problem, distance_heuristic_, a, num_added_);
        feasible_solvers_.emplace_back(instance, problem, distance_heuristic_, a, num_added_);
    }
}

void IndependentTimeExpandedAStarPricer::add_waypoint(const Agent a, const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].add_waypoint(nt);
    }
    else
    {
        feasible_solvers_[a].add_waypoint(nt);
    }
}

void IndependentTimeExpandedAStarPricer::set_constant(const Agent a, const Cost constant)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].set_constant(constant);
    }
    else
    {
        feasible_solvers_[a].set_constant(constant);
    }
}

void IndependentTimeExpandedAStarPricer::add_nodetime_penalty_all_agents(const NodeTime nt, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
    {
        add_nodetime_penalty_one_agent(agent, nt, cost);
    }
}

void IndependentTimeExpandedAStarPricer::add_nodetime_penalty_all_except_one_agent(const Agent a,
                                                                                   const NodeTime nt,
                                                                                   const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
        if (agent != a)
        {
            add_nodetime_penalty_one_agent(agent, nt, cost);
        }
}

void IndependentTimeExpandedAStarPricer::add_nodetime_penalty_one_agent(const Agent a,
                                                                        const NodeTime nt,
                                                                        const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].add_nodetime_penalty(nt, cost);
    }
    else
    {
        feasible_solvers_[a].add_nodetime_penalty(nt, cost);
    }
}

void IndependentTimeExpandedAStarPricer::add_edgetime_penalty_all_agents(const EdgeTime et, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
    {
        add_edgetime_penalty_one_agent(agent, et, cost);
    }
}

void IndependentTimeExpandedAStarPricer::add_edgetime_penalty_all_except_one_agent(const Agent a,
                                                                                   const EdgeTime et,
                                                                                   const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
        if (agent != a)
        {
            add_edgetime_penalty_one_agent(agent, et, cost);
        }
}

void IndependentTimeExpandedAStarPricer::add_edgetime_penalty_one_agent(const Agent a,
                                                                        const EdgeTime et,
                                                                        const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].add_edgetime_penalty(et, cost);
    }
    else
    {
        feasible_solvers_[a].add_edgetime_penalty(et, cost);
    }
}

void IndependentTimeExpandedAStarPricer::add_end_penalty_all_agents(const Time earliest,
                                                                    const Time latest,
                                                                    const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
    {
        add_end_penalty_one_agent(agent, earliest, latest, cost);
    }
}

void IndependentTimeExpandedAStarPricer::add_end_penalty_all_except_one_agent(const Agent a,
                                                                              const Time earliest,
                                                                              const Time latest,
                                                                              const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
        if (agent != a)
        {
            add_end_penalty_one_agent(agent, earliest, latest, cost);
        }
}

void IndependentTimeExpandedAStarPricer::add_end_penalty_one_agent(const Agent a,
                                                                   const Time earliest,
                                                                   const Time latest,
                                                                   const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].add_end_penalty(earliest, latest, cost);
    }
    else
    {
        feasible_solvers_[a].add_end_penalty(earliest, latest, cost);
    }
}

// void IndependentTimeExpandedAStarPricer::add_once_off_penalty_all_agents(const NodeTime nt, const Cost cost)
// {
//     ZoneScopedC(TRACY_COLOUR);
//
//     const Agent A = infeasible_solvers_.size();
//     for (Agent agent = 0; agent < A; ++agent)
//     {
//         add_once_off_penalty_one_agent(agent, nt, cost);
//     }
// }

template<OnceOffDirection d>
void IndependentTimeExpandedAStarPricer::add_once_off_penalty_all_except_one_agent(const Agent a,
                                                                                   const NodeTime nt,
                                                                                   const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = infeasible_solvers_.size();
    for (Agent agent = 0; agent < A; ++agent)
        if (agent != a)
        {
            add_once_off_penalty_one_agent<d>(agent, nt, cost);
        }
}
template void IndependentTimeExpandedAStarPricer::add_once_off_penalty_all_except_one_agent<OnceOffDirection::LEq>(
    const Agent a, const NodeTime nt, const Cost cost);
template void IndependentTimeExpandedAStarPricer::add_once_off_penalty_all_except_one_agent<OnceOffDirection::GEq>(
    const Agent a, const NodeTime nt, const Cost cost);

template<OnceOffDirection d>
void IndependentTimeExpandedAStarPricer::add_once_off_penalty_one_agent(const Agent a,
                                                                        const NodeTime nt,
                                                                        const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].add_once_off_penalty<d>(nt, cost);
    }
    else
    {
        feasible_solvers_[a].add_once_off_penalty<d>(nt, cost);
    }
}
template void IndependentTimeExpandedAStarPricer::add_once_off_penalty_one_agent<OnceOffDirection::LEq>(
    const Agent a, const NodeTime nt, const Cost cost);
template void IndependentTimeExpandedAStarPricer::add_once_off_penalty_one_agent<OnceOffDirection::GEq>(
    const Agent a, const NodeTime nt, const Cost cost);

void IndependentTimeExpandedAStarPricer::add_rectangle_penalty_one_agent(const Agent a,
                                                                         const EdgeTime first_entry,
                                                                         const EdgeTime first_exit,
                                                                         const Time length,
                                                                         const Node n_increment,
                                                                         const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (status_ == MasterProblemStatus::Infeasible)
    {
        infeasible_solvers_[a].add_rectangle_penalty(first_entry, first_exit, length, n_increment, cost);
    }
    else
    {
        feasible_solvers_[a].add_rectangle_penalty(first_entry, first_exit, length, n_increment, cost);
    }
}

Cost IndependentTimeExpandedAStarPricer::solve()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the master problem and branch-and-bound tree.
    const auto& master = problem_.master();
    const auto& bbtree = problem_.bbtree();

    // Store the master problem status.
    status_ = master.status();

    // Add dual solution to pricing costs.
    for (const auto& constraint : master.all_constraints())
    {
        const auto separator = constraint.separator();
        const auto dual = master.constraint_dual_sol(constraint);
        if (is_ne(dual, 0.0))
        {
            separator->add_pricing_costs(constraint, dual);
        }
    }

    // Impose branching decisions.
    for (const auto& [brancher, decision] : bbtree.all_decisions())
    {
        brancher->add_pricing_costs(decision);
    }

    // Run the pricer for every agent.
    Cost new_lb = std::numeric_limits<Cost>::quiet_NaN();
    if (status_ == MasterProblemStatus::Infeasible)
    {
        // Call every pricer with reduced cost function = 0 - Farkas dual values.
        Bool found = false;
        auto& agents_order = partial_pricing_.agents_order();
        for (auto it = agents_order.begin(); it != agents_order.end() && (!found || it->must_price); ++it)
        {
            problem_.stop_if_timed_out();
            const auto a = it->a;
            const auto agent_reduced_cost = infeasible_solvers_[a].solve();
            const auto agent_has_new_path = is_lt(agent_reduced_cost, 0.0);
            *it->priority += agent_has_new_path;
            found |= agent_has_new_path;
        }
    }
    else
    {
        // Call every pricer with reduced cost function = true cost - dual solution.
        new_lb = master.obj();
        Bool found = false;
        auto& agents_order = partial_pricing_.agents_order();
        auto it = agents_order.begin();
        for (; it != agents_order.end() && (!found || it->must_price); ++it)
        {
            problem_.stop_if_timed_out();
            const auto a = it->a;
            const auto agent_reduced_cost = feasible_solvers_[a].solve();
            const auto agent_has_new_path = is_lt(agent_reduced_cost, 0.0);
            *it->priority += agent_has_new_path;
            found |= agent_has_new_path;
            new_lb += agent_reduced_cost;
        }

        // If some agents were skipped, a lower bound is not available.
        if (it != agents_order.end())
        {
            new_lb = std::numeric_limits<Cost>::quiet_NaN();
        }
    }
    return new_lb;
}
