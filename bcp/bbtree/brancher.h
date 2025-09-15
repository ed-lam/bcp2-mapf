/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "problem/instance.h"
#include "types/basic_types.h"
#include "types/clock.h"
#include "types/map_types.h"
#include "types/pointers.h"
#include "types/tuple.h"

class Problem;

enum class BranchDirection : Bool
{
    Down = false,
    Up = true
};

struct BrancherData
{
    // Constructors and destructor
    BrancherData() = default;
    virtual ~BrancherData() = default;
    BrancherData(const BrancherData&) = default;
    BrancherData(BrancherData&&) = default;
    BrancherData& operator=(const BrancherData&) = default;
    BrancherData& operator=(BrancherData&&) = default;
};

using Decisions = Pair<UniquePtr<BrancherData>, UniquePtr<BrancherData>>;

class Brancher
{
    Clock clock_;

  protected:
    const Instance& instance_;
    Problem& problem_;
    Size64 num_added_;

  public:
    // Constructors and destructor
    Brancher() = delete;
    Brancher(const Instance& instance, Problem& problem) :
        clock_(),
        instance_(instance),
        problem_(problem),
        num_added_(0)
    {
    }
    virtual ~Brancher() = default;
    Brancher(const Brancher&) = default;
    Brancher(Brancher&&) = default;
    Brancher& operator=(const Brancher&) = delete;
    Brancher& operator=(Brancher&&) = delete;

    // Statistics
    auto num_added() const
    {
        return num_added_;
    }
    auto run_time() const
    {
        return clock_.total_duration();
    }

    // Branch
    auto run()
    {
        auto timer = clock_.start_timer();
        return branch();
    }
    virtual Decisions branch() = 0;

    // Prevent future incompatible edges in the pricing problem
    virtual void add_pricing_costs(const BrancherData* const data) = 0;

    // Disable existing incompatible paths in the master problem
    virtual void disable_vars(const BrancherData* const data) = 0;

    // Debug
    virtual void print(const BrancherData* const data) const = 0;
};
