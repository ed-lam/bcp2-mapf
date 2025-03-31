/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "problem/debug.h"
#include "types/basic_types.h"
#include <ctime>
#include <limits>

class Clock
{
    class Timer
    {
        Clock& clock_;

      public:
        // Constructors and destructor
        Timer(Clock& clock) : clock_(clock) {}
        ~Timer() { clock_.accumulate_elapsed_time(); }
        Timer(const Timer&) = delete;
        Timer(Timer&&) = delete;
        Timer& operator=(const Timer&) = delete;
        Timer& operator=(Timer&&) = delete;
    };

    Float time_limit_ = std::numeric_limits<Float>::infinity();
    std::clock_t start_time_ = null_time();
    Float total_duration_ = 0.0;

  public:
    // Constructors and destructor
    Clock() = default;
    ~Clock() = default;
    Clock(const Clock&) = default;
    Clock(Clock&&) = default;
    Clock& operator=(const Clock&) = default;
    Clock& operator=(Clock&&) = default;

    // Clock functions
    auto start_timer(const Float time_limit = std::numeric_limits<Float>::infinity())
    {
        time_limit_ = time_limit;
        debug_assert(start_time_ == null_time());
        start_time_ = std::clock();
        return Timer(*this);
    }
    auto elapsed_time() const
    {
        debug_assert(start_time_ != null_time());
        const auto current_time = std::clock();
        const auto duration = static_cast<Float>(current_time - start_time_) / CLOCKS_PER_SEC;
        return duration;
    }
    void accumulate_elapsed_time()
    {
        total_duration_ += elapsed_time();
        start_time_ = null_time();
    }
    auto time_remaining() const { return time_limit_ - elapsed_time(); }
    auto timed_out() const { return time_remaining() < 0.0; }
    auto total_duration() const { return total_duration_; }

  protected:
    static std::clock_t null_time() { return static_cast<std::clock_t>(-1); }
};
