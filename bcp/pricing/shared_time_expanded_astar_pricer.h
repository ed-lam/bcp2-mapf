/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "pricing/distance_heuristic.h"
#include "pricing/edgetime_penalties.h"
#include "pricing/finish_time_penalties.h"
#include "pricing/once_off_penalties.h"
#include "pricing/partial_pricing.h"
#include "pricing/pricer.h"
#include "pricing/rectangle_penalties.h"
#include "pricing/shared_time_expanded_astar.h"
#include "types/hash_map.h"
#include "types/pointers.h"
#include "types/ranges.h"
#include "types/vector.h"

class Problem;

class SharedTimeExpandedAStarPricer : public PricerBase
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
    Vector<FinishTimePenalties> finish_time_penalties_;

    // Edgetime penalties
    EdgeTimePenalties shared_edgetime_penalties_;
    HashMap<Agent, UniquePtr<EdgeTimePenalties>> agent_edgetime_penalties_;
    Vector<UniquePtr<EdgeTimePenalties>> discarded_edgetime_penalties_;

    // Visit times
    UniquePtr<Time[]> shared_latest_visit_time_;
    HashMap<Agent, UniquePtr<Time[]>> agent_latest_visit_time_;
    Vector<UniquePtr<Time[]>> discarded_latest_visit_time_;
    UniquePtr<Time[]> latest_visit_time_template_;
    Vector<Time> earliest_target_time_;
    Vector<Time> latest_target_time_;

    // Solvers
    PartialPricing partial_pricing_;
    SharedTimeExpandedAStar solver_;

  public:
    // Constructors and destructor
    SharedTimeExpandedAStarPricer() = delete;
    SharedTimeExpandedAStarPricer(const Instance& instance, Problem& problem);
    ~SharedTimeExpandedAStarPricer() = default;
    SharedTimeExpandedAStarPricer(const SharedTimeExpandedAStarPricer&) noexcept = delete;
    SharedTimeExpandedAStarPricer(SharedTimeExpandedAStarPricer&&) noexcept = delete;
    SharedTimeExpandedAStarPricer& operator=(const SharedTimeExpandedAStarPricer&) noexcept = delete;
    SharedTimeExpandedAStarPricer& operator=(SharedTimeExpandedAStarPricer&&) noexcept = delete;

    // Pricer type
    constexpr static auto name() { return "Shared time-expanded A*"; }

    // Getters
    auto& distance_heuristic() { return distance_heuristic_; }

    // Set nodetime branching decisions and cost constant
    void add_waypoint(const Agent a, const NodeTime nt);
    void set_constant(const Agent a, const Cost cost);

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
      auto all_edgetime_penalties()
      {
          return Ranges::views::concat(
              Ranges::subrange(agent_edgetime_penalties_.begin(), agent_edgetime_penalties_.end()) |
              Ranges::views::transform(
                  [](auto& pair) -> Pair<Agent, EdgeTimePenalties&> { return {pair.first, *pair.second}; }
              ),
              Ranges::views::single(Pair<Agent, EdgeTimePenalties&>(-1, shared_edgetime_penalties_))
          );
      }
      auto all_latest_visit_times()
      {
          return Ranges::views::concat(
              Ranges::subrange(agent_latest_visit_time_.begin(), agent_latest_visit_time_.end()) |
              Ranges::views::transform(
                  [](auto& pair) -> Pair<Agent, Time*> { return {pair.first, pair.second.get()}; }
              ),
              Ranges::views::single(Pair<Agent, Time*>(-1, shared_latest_visit_time_.get()))
          );
      }
      const EdgeTimePenalties& find_edgetime_penalties(const Agent a) const;
      const Time* find_latest_visit_time(const Agent a) const;
      EdgeTimePenalties& copy_edgetime_penalties(const Agent a);
      Time* copy_latest_visit_time(const Agent a);
};
