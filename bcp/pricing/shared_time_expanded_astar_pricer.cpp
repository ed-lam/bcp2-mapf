/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "pricing/shared_time_expanded_astar_pricer.h"
#include "problem/problem.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::DodgerBlue

SharedTimeExpandedAStarPricer::SharedTimeExpandedAStarPricer(const Instance& instance, Problem& problem) :
    map_(instance.map),
    targets_(),
    targets_hash_(),
    h_to_target_(),
    problem_(problem),
    distance_heuristic_(instance.map),

    constants_(instance.num_agents()),
    waypoints_(instance.num_agents()),
    once_off_penalties_(instance.num_agents()),
    rectangle_penalties_(instance.num_agents()),
    finish_time_penalties_(instance.num_agents()),

    shared_edgetime_penalties_(),
    agent_edgetime_penalties_(),
    discarded_edgetime_penalties_(),

    shared_latest_visit_time_(std::make_unique<Time[]>(map_.size())),
    agent_latest_visit_time_(),
    discarded_latest_visit_time_(),
    latest_visit_time_template_(),
    earliest_target_time_(instance.num_agents()),
    latest_target_time_(instance.num_agents()),

    partial_pricing_(instance, problem),
    solver_(instance, problem, distance_heuristic_, num_added_)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& agents = instance.agents;
    const auto A = instance.num_agents();

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

    // Initialise the latest visit time of each node.
    latest_visit_time_template_ = std::make_unique<Time[]>(map_.size());
    for (Node n = 0; n < map_.size(); ++n)
    {
        latest_visit_time_template_[n] = map_[n] * TIME_MAX;
    }
}

void SharedTimeExpandedAStarPricer::add_waypoint(const Agent a, const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);

    // Add a waypoint.
    waypoints_[a].push_back(nt);
}

void SharedTimeExpandedAStarPricer::set_constant(const Agent a, const Cost constant)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check.
    debug_assert(a >= 0);

    // Overwrite the constasnt.
    constants_[a] = constant;
}

void SharedTimeExpandedAStarPricer::add_nodetime_penalty_all_agents(const NodeTime nt, const Cost cost)
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

    // Add the penalty to the edgetime penalties of all agents.
    for (auto [agent, edgetime_penalties] : all_edgetime_penalties())
    {
        // Add the penalty to the five outgoing directions.
        if (const auto to_n = map_.get_north(nt.n); map_[to_n])
        {
            edgetime_penalties.add_edgetime_penalty(nt, Direction::NORTH, cost);
        }
        if (const auto to_n = map_.get_south(nt.n); map_[to_n])
        {
            edgetime_penalties.add_edgetime_penalty(nt, Direction::SOUTH, cost);
        }
        if (const auto to_n = map_.get_west(nt.n); map_[to_n])
        {
            edgetime_penalties.add_edgetime_penalty(nt, Direction::WEST, cost);
        }
        if (const auto to_n = map_.get_east(nt.n); map_[to_n])
        {
            edgetime_penalties.add_edgetime_penalty(nt, Direction::EAST, cost);
        }
        {
            edgetime_penalties.add_edgetime_penalty(nt, Direction::WAIT, cost);
        }
    }

    // Add the penalty when going to the end if applicable.
    if (auto it = targets_hash_.find(nt.n); it != targets_hash_.end())
    {
        const auto agent = it->second;
        finish_time_penalties_[agent].add(nt.t, cost);
    }
}

void SharedTimeExpandedAStarPricer::add_nodetime_penalty_all_except_one_agent(const Agent a,
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

    // Clone the edgetime penalties for the excluded agent.
    copy_edgetime_penalties(a);

    // Add the penalty to the edgetime penalties of all agents except the excluded agent.
    for (auto [agent, edgetime_penalties] : all_edgetime_penalties())
        if (agent != a)
        {
            // Add the penalty to the five outgoing directions.
            if (const auto to_n = map_.get_north(nt.n); map_[to_n])
            {
                edgetime_penalties.add_edgetime_penalty(nt, Direction::NORTH, cost);
            }
            if (const auto to_n = map_.get_south(nt.n); map_[to_n])
            {
                edgetime_penalties.add_edgetime_penalty(nt, Direction::SOUTH, cost);
            }
            if (const auto to_n = map_.get_west(nt.n); map_[to_n])
            {
                edgetime_penalties.add_edgetime_penalty(nt, Direction::WEST, cost);
            }
            if (const auto to_n = map_.get_east(nt.n); map_[to_n])
            {
                edgetime_penalties.add_edgetime_penalty(nt, Direction::EAST, cost);
            }
            {
                edgetime_penalties.add_edgetime_penalty(nt, Direction::WAIT, cost);
            }
        }

    // Add the penalty when going to the end if applicable.
    if (auto it = targets_hash_.find(nt.n); it != targets_hash_.end())
    {
        const auto agent = it->second;
        if (agent != a)
        {
            finish_time_penalties_[agent].add(nt.t, cost);
        }
    }
}

void SharedTimeExpandedAStarPricer::add_nodetime_penalty_one_agent(const Agent a,
                                                                   const NodeTime nt,
                                                                   const Cost cost)
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
    auto& edgetime_penalties = copy_edgetime_penalties(a);
    if (const auto to_n = map_.get_north(nt.n); map_[to_n])
    {
        edgetime_penalties.add_edgetime_penalty(nt, Direction::NORTH, cost);
    }
    if (const auto to_n = map_.get_south(nt.n); map_[to_n])
    {
        edgetime_penalties.add_edgetime_penalty(nt, Direction::SOUTH, cost);
    }
    if (const auto to_n = map_.get_west(nt.n); map_[to_n])
    {
        edgetime_penalties.add_edgetime_penalty(nt, Direction::WEST, cost);
    }
    if (const auto to_n = map_.get_east(nt.n); map_[to_n])
    {
        edgetime_penalties.add_edgetime_penalty(nt, Direction::EAST, cost);
    }
    {
        edgetime_penalties.add_edgetime_penalty(nt, Direction::WAIT, cost);
    }

    // Add the penalty when going to the end if applicable.
    if (targets_[a] == nt.n)
    {
        finish_time_penalties_[a].add(nt.t, cost);
    }
}

void SharedTimeExpandedAStarPricer::add_edgetime_penalty_all_agents(const EdgeTime et, const Cost cost)
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

    // Add the penalty to the edgetime penalties of all agents.
    for (auto [agent, edgetime_penalties] : all_edgetime_penalties())
    {
        // Add the penalty to the edgetime.
        edgetime_penalties.add_edgetime_penalty(et.nt(), et.d, cost);
    }

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT)
    {
        if (auto it = targets_hash_.find(et.n); it != targets_hash_.end())
        {
            const auto agent = it->second;
            finish_time_penalties_[agent].add(et.t, cost);
        }
    }
}

void SharedTimeExpandedAStarPricer::add_edgetime_penalty_all_except_one_agent(const Agent a,
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

    // Clone the edgetime penalties for the excluded agent.
    copy_edgetime_penalties(a);

    // Add the penalty to the edgetime penalties of all agents except the excluded agent.
    for (auto [agent, edgetime_penalties] : all_edgetime_penalties())
        if (agent != a)
        {
            // Add the penalty to the edgetime.
            edgetime_penalties.add_edgetime_penalty(et.nt(), et.d, cost);
        }

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT)
    {
        if (auto it = targets_hash_.find(et.n); it != targets_hash_.end())
        {
            const auto agent = it->second;
            if (agent != a)
            {
                finish_time_penalties_[agent].add(et.t, cost);
            }
        }
    }
}

void SharedTimeExpandedAStarPricer::add_edgetime_penalty_one_agent(const Agent a,
                                                                   const EdgeTime et,
                                                                   const Cost cost)
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
    auto& edgetime_penalties = copy_edgetime_penalties(a);
    edgetime_penalties.add_edgetime_penalty(et.nt(), et.d, cost);

    // Add the penalty when going to the end if applicable.
    if (et.d == Direction::WAIT && targets_[a] == et.n)
    {
        finish_time_penalties_[a].add(et.t, cost);
    }
}

void SharedTimeExpandedAStarPricer::add_end_penalty_all_agents(const Time earliest, const Time latest, const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = constants_.size();
    for (Agent agent = 0; agent < A; ++agent)
    {
        add_end_penalty_one_agent(agent, earliest, latest, cost);
    }
}

void SharedTimeExpandedAStarPricer::add_end_penalty_all_except_one_agent(const Agent a,
                                                                         const Time earliest,
                                                                         const Time latest,
                                                                         const Cost cost)
{
    ZoneScopedC(TRACY_COLOUR);

    const Agent A = constants_.size();
    for (Agent agent = 0; agent < A; ++agent)
        if (agent != a)
        {
            add_end_penalty_one_agent(agent, earliest, latest, cost);
        }
}

void SharedTimeExpandedAStarPricer::add_end_penalty_one_agent(const Agent a,
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

    // Tighten the finish time bounds.
    debug_assert((earliest == 0) ^ (latest == TIME_MAX));
    if (cost == INF)
    {
        if (earliest == 0)
        {
            earliest_target_time_[a] = std::max(earliest_target_time_[a], latest);
        }
        else
        {
            debug_assert(latest == TIME_MAX);
            latest_target_time_[a] = std::min(latest_target_time_[a], earliest - 1);
        }
    }
    else if (earliest == 0)
    {
        debug_assert(latest >= 1);
        finish_time_penalties_[a].add(latest - 1, cost);
    }
    else
    {
        err("Cannot handle adding end penalty for time interval [{},{}]", earliest, latest);
    }
}

template<OnceOffDirection d>
void SharedTimeExpandedAStarPricer::add_once_off_penalty_all_except_one_agent(const Agent a,
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

    // If the node is blocked after a certain time, tighten the latest visit time. Otherwise handle it in the search
    // because the penalty is only incurred at most once.
    if (cost == INF)
    {
        // TODO
        release_assert(d == OnceOffDirection::GEq,
                       "Once-off penalty with <= time direction is not yet supported for infinite cost");

        // Clone the latest visit times for the excluded agent.
        copy_latest_visit_time(a);

        // Tighten the node visit time bounds.
        for (auto [agent, latest_visit_time] : all_latest_visit_times())
            if (agent != a)
            {
                latest_visit_time[nt.n] = std::min(latest_visit_time[nt.n], nt.t - 1);
            }

        // Tighten the finish time bounds if applicable.
        if (auto it = targets_hash_.find(nt.n); it != targets_hash_.end())
        {
            const auto agent = it->second;
            if (agent != a)
            {
                latest_target_time_[agent] = std::min(latest_target_time_[agent], nt.t - 1);
            }
        }
    }
    else
    {
        const Agent A = constants_.size();
        for (Agent agent = 0; agent < A; ++agent)
            if (agent != a)
            {
                once_off_penalties_[agent].add(cost, nt, d);
            }
    }
}
template void SharedTimeExpandedAStarPricer::add_once_off_penalty_all_except_one_agent<OnceOffDirection::LEq>(
    const Agent a, const NodeTime nt, const Cost cost);
template void SharedTimeExpandedAStarPricer::add_once_off_penalty_all_except_one_agent<OnceOffDirection::GEq>(
    const Agent a, const NodeTime nt, const Cost cost);

template<OnceOffDirection d>
void SharedTimeExpandedAStarPricer::add_once_off_penalty_one_agent(const Agent a, const NodeTime nt, const Cost cost)
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

    // If the node is blocked after a certain time, tighten the latest visit time. Otherwise handle it in the search
    // because the penalty is only incurred at most once.
    if (cost == INF)
    {
        // TODO
        release_assert(d == OnceOffDirection::GEq,
                       "Once-off penalty with <= time direction is not yet supported for infinite cost");

        // Tighten the node visit time bounds.
        auto latest_visit_time = copy_latest_visit_time(a);
        latest_visit_time[nt.n] = std::min(latest_visit_time[nt.n], nt.t - 1);

        // Tighten the finish time bounds if applicable.
        if (nt.n == targets_[a])
        {
            latest_target_time_[a] = std::min(latest_target_time_[a], nt.t - 1);
        }
    }
    else
    {
        once_off_penalties_[a].add(cost, nt, d);
    }
}
template void SharedTimeExpandedAStarPricer::add_once_off_penalty_one_agent<OnceOffDirection::LEq>(
    const Agent a, const NodeTime nt, const Cost cost);
template void SharedTimeExpandedAStarPricer::add_once_off_penalty_one_agent<OnceOffDirection::GEq>(
    const Agent a, const NodeTime nt, const Cost cost);

void SharedTimeExpandedAStarPricer::add_rectangle_penalty_one_agent(const Agent a,
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
}

const EdgeTimePenalties& SharedTimeExpandedAStarPricer::find_edgetime_penalties(const Agent a) const
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (auto it = agent_edgetime_penalties_.find(a); it != agent_edgetime_penalties_.end())
    {
        return *it->second;
    }
    else
    {
        return shared_edgetime_penalties_;
    }
}

const Time* SharedTimeExpandedAStarPricer::find_latest_visit_time(const Agent a) const
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    if (auto it = agent_latest_visit_time_.find(a); it != agent_latest_visit_time_.end())
    {
        return it->second.get();
    }
    else
    {
        return shared_latest_visit_time_.get();
    }
}

EdgeTimePenalties& SharedTimeExpandedAStarPricer::copy_edgetime_penalties(const Agent a)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    auto [it, created] = agent_edgetime_penalties_.emplace(a, nullptr);
    auto& edgetime_penalties_ptr = it->second;
    if (created)
    {
        // Reuse an existing object if available.
        if (!discarded_edgetime_penalties_.empty())
        {
            edgetime_penalties_ptr = std::move(discarded_edgetime_penalties_.back());
            discarded_edgetime_penalties_.pop_back();
        }
        else
        {
            edgetime_penalties_ptr = std::make_unique<EdgeTimePenalties>();
        }
        *edgetime_penalties_ptr = shared_edgetime_penalties_;
    }
    return *edgetime_penalties_ptr;
}

Time* SharedTimeExpandedAStarPricer::copy_latest_visit_time(const Agent a)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(a >= 0);
    auto [it, created] = agent_latest_visit_time_.emplace(a, nullptr);
    auto& latest_visit_time_ptr = it->second;
    if (created)
    {
        // Reuse an existing object if available.
        if (!discarded_latest_visit_time_.empty())
        {
            latest_visit_time_ptr = std::move(discarded_latest_visit_time_.back());
            discarded_latest_visit_time_.pop_back();
        }
        else
        {
            latest_visit_time_ptr = std::make_unique<Time[]>(map_.size());
        }
        std::copy(shared_latest_visit_time_.get(),
                  shared_latest_visit_time_.get() + map_.size(),
                  latest_visit_time_ptr.get());
    }
    return latest_visit_time_ptr.get();
}

Cost SharedTimeExpandedAStarPricer::solve()
{
    ZoneScopedC(TRACY_COLOUR);

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
                                                                 find_edgetime_penalties(a),
                                                                 finish_time_penalties_[a],
                                                                 once_off_penalties_[a],
                                                                 rectangle_penalties_[a],
                                                                 find_latest_visit_time(a),
                                                                 earliest_target_time_[a],
                                                                 latest_target_time_[a]);
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
                                                                find_edgetime_penalties(a),
                                                                finish_time_penalties_[a],
                                                                once_off_penalties_[a],
                                                                rectangle_penalties_[a],
                                                                find_latest_visit_time(a),
                                                                earliest_target_time_[a],
                                                                latest_target_time_[a]);
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

void SharedTimeExpandedAStarPricer::clear()
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

    // Clear finish time penalties.
    for (auto& data : finish_time_penalties_)
    {
        data.clear();
    }

    // Clear edgetime_penalties. Move the objects for reuse
    shared_edgetime_penalties_.clear();
    for (auto& [agent, edgetime_penalties_ptr] : agent_edgetime_penalties_)
    {
        discarded_edgetime_penalties_.push_back(std::move(edgetime_penalties_ptr));
    }
    agent_edgetime_penalties_.clear();

    // Clear latest visit time.
    std::copy(latest_visit_time_template_.get(),
              latest_visit_time_template_.get() + map_.size(),
              shared_latest_visit_time_.get());
    for (auto& [agent, latest_visit_time_ptr] : agent_latest_visit_time_)
    {
        discarded_latest_visit_time_.push_back(std::move(latest_visit_time_ptr));
    }
    agent_latest_visit_time_.clear();

    // Clear target time bounds.
    std::fill(earliest_target_time_.begin(), earliest_target_time_.end(), 0);
    std::fill(latest_target_time_.begin(), latest_target_time_.end(), TIME_MAX);
}
