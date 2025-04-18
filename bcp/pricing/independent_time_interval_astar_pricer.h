/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/master.h"
#include "pricing/distance_heuristic.h"
#include "pricing/independent_time_interval_astar.h"
#include "pricing/partial_pricing.h"
#include "pricing/pricer.h"

class IndependentTimeIntervalAStarPricer : public PricerBase
{
    // Problem
    Problem& problem_;
    DistanceHeuristic distance_heuristic_;

    // Solvers
    MasterProblemStatus status_;
    PartialPricing partial_pricing_;
    Vector<IndependentTimeIntervalAStar<false>> infeasible_solvers_;
    Vector<IndependentTimeIntervalAStar<true>> feasible_solvers_;

  public:
    // Constructors and destructor
    IndependentTimeIntervalAStarPricer() = delete;
    IndependentTimeIntervalAStarPricer(const Instance& instance, Problem& problem);
    ~IndependentTimeIntervalAStarPricer() = default;
    IndependentTimeIntervalAStarPricer(const IndependentTimeIntervalAStarPricer&) noexcept = delete;
    IndependentTimeIntervalAStarPricer(IndependentTimeIntervalAStarPricer&&) noexcept = delete;
    IndependentTimeIntervalAStarPricer& operator=(const IndependentTimeIntervalAStarPricer&) noexcept = delete;
    IndependentTimeIntervalAStarPricer& operator=(IndependentTimeIntervalAStarPricer&&) noexcept = delete;

    // Pricer type
    constexpr static auto name() { return "Independent time-interval A*"; }

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
};
