/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "pricing/distance_heuristic.h"
#include "pricing/once_off_penalties.h"
#include "pricing/partial_pricing.h"
#include "pricing/pricer.h"
#include "pricing/rectangle_penalties.h"
#include "pricing/shared_intervals.h"
#include "pricing/shared_time_interval_astar.h"
#include "types/hash_map.h"
#include "types/pointers.h"
#include "types/ranges.h"
#include "types/vector.h"

class Problem;

class SharedTimeIntervalAStarPricer : public PricerBase
{
    // Problem
    const Map& map_;
    Vector<Node> targets_;
    HashMap<Node, Agent> targets_hash_;
    Vector<const Time*> h_to_target_;
    Problem& problem_;
    DistanceHeuristic distance_heuristic_;

    // Costs
    Vector<Float> constants_;
    Vector<Vector<NodeTime>> waypoints_;
    Vector<OnceOffPenalties> once_off_penalties_;
    Vector<RectanglePenalties> rectangle_penalties_;

    // Intervals
    SharedIntervals shared_intervals_;
    HashMap<Agent, UniquePtr<SharedIntervals>> agent_intervals_;
    Vector<UniquePtr<SharedIntervals>> discarded_agent_intervals_;
    Vector<Vector<Interval>> deferred_end_intervals_;

    // Solver
    PartialPricing partial_pricing_;
    SharedTimeIntervalAStar solver_;

  public:
    // Constructors and destructor
    SharedTimeIntervalAStarPricer() = delete;
    SharedTimeIntervalAStarPricer(const Instance& instance, Problem& problem);
    ~SharedTimeIntervalAStarPricer() = default;
    SharedTimeIntervalAStarPricer(const SharedTimeIntervalAStarPricer&) noexcept = delete;
    SharedTimeIntervalAStarPricer(SharedTimeIntervalAStarPricer&&) noexcept = delete;
    SharedTimeIntervalAStarPricer& operator=(const SharedTimeIntervalAStarPricer&) noexcept = delete;
    SharedTimeIntervalAStarPricer& operator=(SharedTimeIntervalAStarPricer&&) noexcept = delete;

    // Pricer type
    constexpr static auto name() { return "Shared time-interval A*"; }

    // Getters
    auto& distance_heuristic() { return distance_heuristic_; }

    // Set nodetime branching decisions and cost constant
    void add_waypoint(const Agent a, const NodeTime nt);
    void set_constant(const Agent a, const Cost constant);

    // Add nodetime costs
    void add_nodetime_penalty_all_agents(const NodeTime nt, const Cost cost);
    void add_nodetime_penalty_all_except_one_agent(const Agent a, const NodeTime nt, const Cost cost);
    void add_nodetime_penalty_one_agent(const Agent a, const NodeTime nt, const Cost cost);

    // Add edgetime costs
    void add_edgetime_penalty_all_agents(const EdgeTime et, const Cost cost);
    void add_edgetime_penalty_all_except_one_agent(const Agent a, const EdgeTime et, const Cost cost);
    void add_edgetime_penalty_one_agent(const Agent a, const EdgeTime et, const Cost cost);

    // Add end costs
    void add_end_penalty_all_agents(const Time earliest, const Time latest, const Cost cost);
    void add_end_penalty_all_except_one_agent(const Agent a, const Time earliest, const Time latest, const Cost cost);
    void add_end_penalty_one_agent(const Agent a, const Time earliest, const Time latest, const Cost cost);

    // Add costs for once-off penalties
    template<OnceOffDirection d = OnceOffDirection::GEq>
    void add_once_off_penalty_all_except_one_agent(const Agent a, const NodeTime nt, const Cost cost);
    template<OnceOffDirection d = OnceOffDirection::GEq>
    void add_once_off_penalty_one_agent(const Agent a, const NodeTime nt, const Cost cost);

    // Add costs for rectangle penalties
    void add_rectangle_penalty_one_agent(const Agent a,
                                         const EdgeTime first_entry,
                                         const EdgeTime first_exit,
                                         const Time length,
                                         const Node n_increment,
                                         const Cost cost);

    // Solve
    Cost solve();

  private:
    // Clear costs
    void clear();

    // Functions for penalties
    auto all_intervals_sets()
    {
        return Ranges::views::concat(
            Ranges::subrange(agent_intervals_.begin(), agent_intervals_.end()) |
            Ranges::views::transform(
                [](auto& pair) -> Pair<Agent, SharedIntervals&> { return {pair.first, *pair.second}; }
            ),
            Ranges::views::single(Pair<Agent, SharedIntervals&>(-1, shared_intervals_))
        );
    }
    const SharedIntervals& find_intervals_set(const Agent a) const;
    SharedIntervals& copy_intervals_set(const Agent a);
};
