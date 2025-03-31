/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "pricing/shared_time_interval_astar_pricer.h"
#include "problem/problem.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::DodgerBlue

SharedTimeIntervalAStarPricer::SharedTimeIntervalAStarPricer(const Instance& instance, Problem& problem) :
    map_(instance.map),
    targets_(),
    targets_hash_(),
    h_to_target_(),
    problem_(problem),
    distance_heuristic_(instance.map),

    constants_(instance.agents.size()),
    waypoints_(instance.agents.size()),
    once_off_penalties_(instance.agents.size()),
    rectangle_penalties_(instance.agents.size()),

    shared_intervals_(map_, true),
    agent_intervals_(),
    discarded_agent_intervals_(),
    deferred_end_intervals_(instance.agents.size()),

    partial_pricing_(instance, problem),
    solver_(instance, problem, distance_heuristic_, num_added_)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& agents = instance.agents;
    const Agent A = agents.size();

    // Store the target of each agent.
    targets_.resize(A);
    h_to_target_.resize(A);
    for (Agent a = 0; a < A; ++a)
    {
        const auto target = agents[a].target;
        targets_[a] = target;
        targets_hash_[target] = a;
        h_to_target_[a] = distance_heuristic_.get_h(target);
    }
}

void SharedTimeIntervalAStarPricer::add_waypoint(const Agent a, const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);

    // Add a waypoint.
    waypoints_[a].push_back(nt);
}

void SharedTimeIntervalAStarPricer::set_constant(const Agent a, const Cost constant)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);

    // Overwrite the constasnt.
    constants_[a] = constant;
}

void SharedTimeIntervalAStarPricer::add_nodetime_penalty_all_agents(const NodeTime nt, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Add the penalty to the intervals of all agents.
    for (auto [agent, intervals] : all_intervals_sets())
    {
        // Add the penalty to the five outgoing directions.
        if (const auto to_n = map_.get_north(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::NORTH, nt.t, nt.t + 1, cost);
        }
        if (const auto to_n = map_.get_south(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::SOUTH, nt.t, nt.t + 1, cost);
        }
        if (const auto to_n = map_.get_west(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::WEST, nt.t, nt.t + 1, cost);
        }
        if (const auto to_n = map_.get_east(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::EAST, nt.t, nt.t + 1, cost);
        }
        {
            intervals.add_penalty(nt.n, Direction::WAIT, nt.t, nt.t + 1, cost);
        }
    }

    // Add the penalty when going to the end if applicable.
    if (auto it = targets_hash_.find(nt.n); it != targets_hash_.end())
    {
        const auto agent = it->second;
        deferred_end_intervals_[agent].emplace_back(0, nt.t + 1, cost);
    }
}

void SharedTimeIntervalAStarPricer::add_nodetime_penalty_all_except_one_agent(const Agent a,
                                                                              const NodeTime nt,
                                                                              const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Clone the intervals at the edges that will be modified for the excluded agent.
    {
        auto& intervals = copy_intervals_set(a);
        if (const auto to_n = map_.get_north(nt.n); map_[to_n])
        {
            intervals.duplicate_edge_intervals(nt.n, Direction::NORTH);
        }
        if (const auto to_n = map_.get_south(nt.n); map_[to_n])
        {
            intervals.duplicate_edge_intervals(nt.n, Direction::SOUTH);
        }
        if (const auto to_n = map_.get_west(nt.n); map_[to_n])
        {
            intervals.duplicate_edge_intervals(nt.n, Direction::WEST);
        }
        if (const auto to_n = map_.get_east(nt.n); map_[to_n])
        {
            intervals.duplicate_edge_intervals(nt.n, Direction::EAST);
        }
        {
            intervals.duplicate_edge_intervals(nt.n, Direction::WAIT);
        }
    }

    // Add the penalty to the intervals of all agents except the excluded agent.
    for (auto [agent, intervals] : all_intervals_sets())
        if (agent != a)
        {
            // Add the penalty to the five outgoing directions.
            if (const auto to_n = map_.get_north(nt.n); map_[to_n])
            {
                intervals.add_penalty(nt.n, Direction::NORTH, nt.t, nt.t + 1, cost);
            }
            if (const auto to_n = map_.get_south(nt.n); map_[to_n])
            {
                intervals.add_penalty(nt.n, Direction::SOUTH, nt.t, nt.t + 1, cost);
            }
            if (const auto to_n = map_.get_west(nt.n); map_[to_n])
            {
                intervals.add_penalty(nt.n, Direction::WEST, nt.t, nt.t + 1, cost);
            }
            if (const auto to_n = map_.get_east(nt.n); map_[to_n])
            {
                intervals.add_penalty(nt.n, Direction::EAST, nt.t, nt.t + 1, cost);
            }
            {
                intervals.add_penalty(nt.n, Direction::WAIT, nt.t, nt.t + 1, cost);
            }
        }

    // Add the penalty when going to the end if applicable.
    if (auto it = targets_hash_.find(nt.n); it != targets_hash_.end())
    {
        const auto agent = it->second;
        if (agent != a)
        {
            deferred_end_intervals_[agent].emplace_back(0, nt.t + 1, cost);
        }
    }
}

void SharedTimeIntervalAStarPricer::add_nodetime_penalty_one_agent(const Agent a, const NodeTime nt, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_target_[a][nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Add the penalty to the five outgoing directions.
    auto& intervals = copy_intervals_set(a);
    if (const auto to_n = map_.get_north(nt.n); map_[to_n])
    {
        intervals.add_penalty(nt.n, Direction::NORTH, nt.t, nt.t + 1, cost);
    }
    if (const auto to_n = map_.get_south(nt.n); map_[to_n])
    {
        intervals.add_penalty(nt.n, Direction::SOUTH, nt.t, nt.t + 1, cost);
    }
    if (const auto to_n = map_.get_west(nt.n); map_[to_n])
    {
        intervals.add_penalty(nt.n, Direction::WEST, nt.t, nt.t + 1, cost);
    }
    if (const auto to_n = map_.get_east(nt.n); map_[to_n])
    {
        intervals.add_penalty(nt.n, Direction::EAST, nt.t, nt.t + 1, cost);
    }
    {
        intervals.add_penalty(nt.n, Direction::WAIT, nt.t, nt.t + 1, cost);
    }

    // Add the penalty when going to the end if applicable.
    if (targets_[a] == nt.n)
    {
        intervals.add_end_penalty(0, nt.t + 1, cost);
    }
}

void SharedTimeIntervalAStarPricer::add_edgetime_penalty_all_agents(const EdgeTime et, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(map_[et.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Add the penalty to the intervals of all agents.
    for (auto [agent, intervals] : all_intervals_sets())
    {
        // Add the penalty to the edgetime.
        intervals.add_penalty(et.n, et.d, et.t, et.t + 1, cost);
    }

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT)
    {
        if (auto it = targets_hash_.find(et.n); it != targets_hash_.end())
        {
            const auto agent = it->second;
            deferred_end_intervals_[agent].emplace_back(0, et.t + 1, cost);
        }
    }
}


void SharedTimeIntervalAStarPricer::add_edgetime_penalty_all_except_one_agent(const Agent a,
                                                                              const EdgeTime et,
                                                                              const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(map_[et.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Clone the intervals at the edges that will be modified for the excluded agent.
    {
        auto& intervals = copy_intervals_set(a);
        intervals.duplicate_edge_intervals(et.n, et.d);
    }

    // Add the penalty to the intervals of all agents except the excluded agent.
    for (auto [agent, intervals] : all_intervals_sets())
        if (agent != a)
        {
            // Add the penalty to the edgetime.
            intervals.add_penalty(et.n, et.d, et.t, et.t + 1, cost);
        }

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT)
    {
        if (auto it = targets_hash_.find(et.n); it != targets_hash_.end())
        {
            const auto agent = it->second;
            if (agent != a)
            {
                deferred_end_intervals_[agent].emplace_back(0, et.t + 1, cost);
            }
        }
    }
}


void SharedTimeIntervalAStarPricer::add_edgetime_penalty_one_agent(const Agent a, const EdgeTime et, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(map_[et.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_target_[a][et.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // Add the penalty to the edgetime.
    auto& intervals = copy_intervals_set(a);
    intervals.add_penalty(et.n, et.d, et.t, et.t + 1, cost);

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT && targets_[a] == et.n)
    {
        intervals.add_end_penalty(0, et.t + 1, cost);
    }
}

void SharedTimeIntervalAStarPricer::add_end_penalty_all_agents(const Time earliest,
                                                               const Time latest,
                                                               const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert((earliest == 0) ^ (latest == TIME_MAX));
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Add the penalty to the intervals of all agents.
    for (auto [agent, intervals] : all_intervals_sets())
    {
        // Add the penalty.
        intervals.add_end_penalty(earliest, latest, cost);
    }
}

void SharedTimeIntervalAStarPricer::add_end_penalty_all_except_one_agent(const Agent a,
                                                                         const Time earliest,
                                                                         const Time latest,
                                                                         const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert((earliest == 0) ^ (latest == TIME_MAX));
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Clone the intervals at the edges that will be modified for the excluded agent.
    {
        auto& intervals = copy_intervals_set(a);
        intervals.duplicate_end_intervals();
    }

    // Add the penalty to the intervals of all agents.
    for (auto [agent, intervals] : all_intervals_sets())
        if (agent != a)
        {
            // Add the penalty.
            intervals.add_end_penalty(earliest, latest, cost);
        }
}

void SharedTimeIntervalAStarPricer::add_end_penalty_one_agent(const Agent a,
                                                              const Time earliest,
                                                              const Time latest,
                                                              const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert((earliest == 0) ^ (latest == TIME_MAX));
    debug_assert(cost >= 0.0);

    // Ignore if the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Add the penalty.
    auto& intervals = copy_intervals_set(a);
    intervals.add_end_penalty(earliest, latest, cost);
}

template<OnceOffDirection d>
void SharedTimeIntervalAStarPricer::add_once_off_penalty_all_except_one_agent(const Agent a,
                                                                              const NodeTime nt,
                                                                              const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    debug_assert(a >= 0);
    if (h_to_target_[a][nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // If the node is blocked after a certain time, add it as a penalty. Otherwise handle it in the search because the
    // penalty is only incurred at most once.
    if (cost == INF)
    {
        // TODO
        release_assert(d == OnceOffDirection::GEq,
                       "Once-off penalty with <= time direction is not yet supported for infinite cost");

        // Clone the intervals at the edges that will be modified for the excluded agent.
        {
            auto& intervals = copy_intervals_set(a);
            if (const auto to_n = map_.get_north(nt.n); map_[to_n])
            {
                intervals.duplicate_edge_intervals(nt.n, Direction::NORTH);
            }
            if (const auto to_n = map_.get_south(nt.n); map_[to_n])
            {
                intervals.duplicate_edge_intervals(nt.n, Direction::SOUTH);
            }
            if (const auto to_n = map_.get_west(nt.n); map_[to_n])
            {
                intervals.duplicate_edge_intervals(nt.n, Direction::WEST);
            }
            if (const auto to_n = map_.get_east(nt.n); map_[to_n])
            {
                intervals.duplicate_edge_intervals(nt.n, Direction::EAST);
            }
            {
                intervals.duplicate_edge_intervals(nt.n, Direction::WAIT);
            }
        }

        // Add the penalty to the intervals of all agents except the excluded agent.
        for (auto [agent, intervals] : all_intervals_sets())
            if (agent != a)
            {
                // Add the penalty to the five outgoing directions.
                if (const auto to_n = map_.get_north(nt.n); map_[to_n])
                {
                    intervals.add_penalty(nt.n, Direction::NORTH, nt.t, TIME_MAX, cost);
                }
                if (const auto to_n = map_.get_south(nt.n); map_[to_n])
                {
                    intervals.add_penalty(nt.n, Direction::SOUTH, nt.t, TIME_MAX, cost);
                }
                if (const auto to_n = map_.get_west(nt.n); map_[to_n])
                {
                    intervals.add_penalty(nt.n, Direction::WEST, nt.t, TIME_MAX, cost);
                }
                if (const auto to_n = map_.get_east(nt.n); map_[to_n])
                {
                    intervals.add_penalty(nt.n, Direction::EAST, nt.t, TIME_MAX, cost);
                }
                {
                    intervals.add_penalty(nt.n, Direction::WAIT, nt.t, TIME_MAX, cost);
                }
        }

        // Add the penalty when going to the end if applicable.
        if (auto it = targets_hash_.find(nt.n); it != targets_hash_.end())
        {
            const auto agent = it->second;
            if (agent != a)
            {
                // Add the penalty.
                auto& intervals = copy_intervals_set(agent);
                intervals.add_end_penalty(0, nt.t + 1, cost);
            }
        }
    }
    else
    {
        const Agent A = targets_.size();
        for (Agent agent = 0; agent < A; ++agent)
            if (agent != a)
            {
                once_off_penalties_[agent].add(cost, nt, d);
            }

        // If the once-off penalty can be bypassed by waiting at a neighbour node, add intervals to facilitate this
        // bypass. TODO
        if constexpr (d == OnceOffDirection::LEq)
        {
            err("Simultaneosly adding once-off penalty with <= time direction for more than one agent is not yet "
                "supported");
        }
    }
}
template void SharedTimeIntervalAStarPricer::add_once_off_penalty_all_except_one_agent<OnceOffDirection::LEq>(
    const Agent a, const NodeTime nt, const Cost cost);
template void SharedTimeIntervalAStarPricer::add_once_off_penalty_all_except_one_agent<OnceOffDirection::GEq>(
    const Agent a, const NodeTime nt, const Cost cost);

template<OnceOffDirection d>
void SharedTimeIntervalAStarPricer::add_once_off_penalty_one_agent(const Agent a, const NodeTime nt, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(map_[nt.n]);
    debug_assert(cost >= 0.0);

    // Ignore if the target is unreachable or the penalty is 0.
    if (h_to_target_[a][nt.n] == TIME_MAX || is_le(cost, 0.0))
    {
        return;
    }

    // If the node is blocked after a certain time, add it as a penalty. Otherwise handle it in the search because the
    // penalty is only incurred at most once.
    if (cost == INF)
    {
        // TODO
        release_assert(d == OnceOffDirection::GEq,
                       "Once-off penalty with <= time direction is not yet supported for infinite cost");

        // Add the penalty to the five outgoing directions.
        auto& intervals = copy_intervals_set(a);
        if (const auto to_n = map_.get_north(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::NORTH, nt.t, TIME_MAX, cost);
        }
        if (const auto to_n = map_.get_south(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::SOUTH, nt.t, TIME_MAX, cost);
        }
        if (const auto to_n = map_.get_west(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::WEST, nt.t, TIME_MAX, cost);
        }
        if (const auto to_n = map_.get_east(nt.n); map_[to_n])
        {
            intervals.add_penalty(nt.n, Direction::EAST, nt.t, TIME_MAX, cost);
        }
        {
            intervals.add_penalty(nt.n, Direction::WAIT, nt.t, TIME_MAX, cost);
        }

        // Add the penalty when going to the end if applicable.
        if (targets_[a] == nt.n)
        {
            intervals.add_end_penalty(0, nt.t + 1, cost);
        }
    }
    else
    {
        // Add the once-off penalty.
        once_off_penalties_[a].add(cost, nt, d);

        // If the once-off penalty can be bypassed by waiting at a neighbour node, add intervals to facilitate this
        // bypass.
        if constexpr (d == OnceOffDirection::LEq)
        {
            auto& intervals = copy_intervals_set(a);
            if (const auto from_n = map_.get_north(nt.n); map_[from_n])
            {
                intervals.add_penalty(from_n, Direction::SOUTH, nt.t, TIME_MAX, 0.0);
            }
            if (const auto from_n = map_.get_south(nt.n); map_[from_n])
            {
                intervals.add_penalty(from_n, Direction::NORTH, nt.t, TIME_MAX, 0.0);
            }
            if (const auto from_n = map_.get_west(nt.n); map_[from_n])
            {
                intervals.add_penalty(from_n, Direction::EAST, nt.t, TIME_MAX, 0.0);
            }
            if (const auto from_n = map_.get_east(nt.n); map_[from_n])
            {
                intervals.add_penalty(from_n, Direction::WEST, nt.t, TIME_MAX, 0.0);
            }
        }
    }
}
template void SharedTimeIntervalAStarPricer::add_once_off_penalty_one_agent<OnceOffDirection::LEq>(
    const Agent a, const NodeTime nt, const Cost cost);
template void SharedTimeIntervalAStarPricer::add_once_off_penalty_one_agent<OnceOffDirection::GEq>(
    const Agent a, const NodeTime nt, const Cost cost);


void SharedTimeIntervalAStarPricer::add_rectangle_penalty_one_agent(const Agent a,
                                                                    const EdgeTime first_entry,
                                                                    const EdgeTime first_exit,
                                                                    const Time length,
                                                                    const Node n_increment,
                                                                    const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);
    debug_assert(cost >= 0.0);

    // Ignore if the the penalty is 0.
    if (is_le(cost, 0.0))
    {
        return;
    }

    // Add the penality.
    rectangle_penalties_[a].add(cost, first_entry, first_exit, length, n_increment);

    // Add intervals to bypass crossing the two boundaries.
    auto& intervals = copy_intervals_set(a);
    const auto d = first_entry.d;
    for (Time i = 0; i <= length; ++i)
    {
        const auto n = first_entry.n + i * n_increment;
        const auto next_n = map_.get_destination(n, d);
        if (map_[n] && map_[next_n])
        {
            const auto t = first_entry.t + i;
            intervals.add_penalty(n, d, t + 1, TIME_MAX, 0.0);
        }
    }
    for (Time i = 0; i <= length; ++i)
    {
        const auto n = first_exit.n + i * n_increment;
        const auto next_n = map_.get_destination(n, d);
        if (map_[n] && map_[next_n])
        {
            const auto t = first_exit.t + i;
            intervals.add_penalty(n, d, t + 1, TIME_MAX, 0.0);
        }
    }
}

const SharedIntervals& SharedTimeIntervalAStarPricer::find_intervals_set(const Agent a) const
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (auto it = agent_intervals_.find(a); it != agent_intervals_.end())
    {
        return *it->second;
    }
    else
    {
        return shared_intervals_;
    }
}

SharedIntervals& SharedTimeIntervalAStarPricer::copy_intervals_set(const Agent a)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    auto [it, created] = agent_intervals_.emplace(a, nullptr);
    auto& intervals_ptr = it->second;
    if (created)
    {
        // Reuse an existing object if available.
        if (!discarded_agent_intervals_.empty())
        {
            intervals_ptr = std::move(discarded_agent_intervals_.back());
            discarded_agent_intervals_.pop_back();
        }
        else
        {
            intervals_ptr = std::make_unique<SharedIntervals>(map_, false);
        }
        intervals_ptr->copy_intervals_set(shared_intervals_);
    }
    return *intervals_ptr;
}

Cost SharedTimeIntervalAStarPricer::solve()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const Agent A = waypoints_.size();

    // Get the master problem and branch-and-bound tree.
    const auto& master = problem_.master();
    const auto& bbtree = problem_.bbtree();

    // Clear inputs from previous iteration.
    clear();

    // Add dual solution of constraints spanning all agents to the pricing costs.
    for (const auto& constraint : master.universal_constraints())
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

    // Add dual solution of constraints spanning a subset of agents to the pricing costs.
    for (const auto& constraint : master.subset_constraints())
    {
        const auto separator = constraint.separator();
        const auto dual = master.constraint_dual_sol(constraint);
        if (is_ne(dual, 0.0))
        {
            separator->add_pricing_costs(constraint, dual);
        }
    }

    // Copy the shared intervals to every agent and insert end intervals from universal constraints.
    for (Agent a = 0; a < A; ++a)
        if (!deferred_end_intervals_[a].empty())
        {
            // Insert the end intervals that were deferred while adding shared intervals.
            auto& intervals = copy_intervals_set(a);
            for (const auto& [start, end, cost, _] : deferred_end_intervals_[a])
            {
                intervals.add_end_penalty(start, end, cost);
            }
        }

    // Finalise intervals before solving.
    for (auto [agent, intervals] : all_intervals_sets())
    {
        intervals.finalise();
    }

    // Run the pricer for every agent.
    Cost new_lb = std::numeric_limits<Cost>::quiet_NaN();
    if (master.status() == MasterProblemStatus::Infeasible)
    {
        // Call every pricer with reduced cost function = 0 - Farkas dual values.
        Bool found = false;
        auto& agents_order = partial_pricing_.agents_order();
        for (auto it = agents_order.begin(); it != agents_order.end() && (!found || it->must_price); ++it)
        {
            problem_.stop_if_timed_out();
            const auto a = it->a;
            const auto agent_reduced_cost = solver_.solve<false>(a,
                                                                 waypoints_[a],
                                                                 constants_[a],
                                                                 find_intervals_set(a),
                                                                 once_off_penalties_[a],
                                                                 rectangle_penalties_[a]);
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
            const auto agent_reduced_cost = solver_.solve<true>(a,
                                                                waypoints_[a],
                                                                constants_[a],
                                                                find_intervals_set(a),
                                                                once_off_penalties_[a],
                                                                rectangle_penalties_[a]);
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

void SharedTimeIntervalAStarPricer::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    // Clear constants.
    for (auto& data : constants_)
    {
        data = 0.0;
    }

    // Clear waypoints.
    for (auto& data : waypoints_)
    {
        data.clear();
    }

    // Clear once-off penalties.
    for (auto& data : once_off_penalties_)
    {
        data.clear();
    }

    // Clear rectangle penalties.
    for (auto& data : rectangle_penalties_)
    {
        data.clear();
    }

    // Clear intervals sets. Move the objects for reuse later.
    shared_intervals_.clear();
    for (auto& [agent, intervals_ptr] : agent_intervals_)
    {
        // Clear.
        auto& intervals = *intervals_ptr;
        intervals.clear();

        // Move the pointer.
        discarded_agent_intervals_.push_back(std::move(intervals_ptr));
    }
    agent_intervals_.clear();

    // Clear deferred end intervals.
    for (auto& data : deferred_end_intervals_)
    {
        data.clear();
    }
}
