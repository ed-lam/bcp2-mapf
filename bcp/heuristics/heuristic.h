/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/clock.h"
#include "types/tuple.h"
#include "types/vector.h"

class Problem;
class Variable;
struct Instance;

class PrimalHeuristic
{
    Clock clock_;

  protected:
    const Instance& instance_;
    Problem& problem_;
    Size num_feasible_;
    Size num_improving_;

  public:
    // Constructors and destructor
    PrimalHeuristic() = delete;
    PrimalHeuristic(const Instance& instance, Problem& problem) :
        clock_(), instance_(instance), problem_(problem), num_feasible_(0), num_improving_(0) {}
    virtual ~PrimalHeuristic() = default;
    PrimalHeuristic(const PrimalHeuristic&) = default;
    PrimalHeuristic(PrimalHeuristic&&) = default;
    PrimalHeuristic& operator=(const PrimalHeuristic&) = delete;
    PrimalHeuristic& operator=(PrimalHeuristic&&) = delete;

    // Statistics
    auto num_feasible() const { return num_feasible_; }
    auto num_improving() const { return num_improving_; }
    auto run_time() const { return clock_.total_duration(); }
    void increment_num_improving() { ++num_improving_; }

    // Run
    auto run()
    {
        auto timer = clock_.start_timer();
        return execute();
    }
    virtual Pair<Cost, Vector<Variable*>> execute() = 0;
};
