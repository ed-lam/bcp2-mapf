/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/clock.h"

class PricerBase
{
    Clock clock_;

  protected:
    Size64 num_added_;

  public:
    // Constructors and destructor
    PricerBase() :
        clock_(),
        num_added_(0)
    {
    }
    ~PricerBase() = default;
    PricerBase(const PricerBase&) noexcept = delete;
    PricerBase(PricerBase&&) noexcept = delete;
    PricerBase& operator=(const PricerBase&) noexcept = delete;
    PricerBase& operator=(PricerBase&&) noexcept = delete;

    // Statistics
    auto num_added() const
    {
        return num_added_;
    }
    auto run_time() const
    {
        return clock_.total_duration();
    }

    // Solve
    auto run()
    {
        auto timer = clock_.start_timer();
        return solve();
    }
    virtual Cost solve() = 0;
};
