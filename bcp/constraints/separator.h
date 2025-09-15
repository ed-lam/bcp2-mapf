/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/clock.h"
#include "types/debug.h"
#include "types/map_types.h"
#include "types/path.h"

class Constraint;
class Problem;
struct Instance;

class Separator
{
    Clock clock_;

  protected:
    const Instance& instance_;
    Problem& problem_;
    Size64 num_added_;

  public:
    // Constructors and destructor
    Separator() = delete;
    Separator(const Instance& instance, Problem& problem) :
        clock_(),
        instance_(instance),
        problem_(problem),
        num_added_(0)
    {
    }
    virtual ~Separator() = default;
    Separator(const Separator&) = default;
    Separator(Separator&&) = default;
    Separator& operator=(const Separator&) = delete;
    Separator& operator=(Separator&&) = delete;

    // Statistics
    auto num_added() const
    {
        return num_added_;
    }
    auto run_time() const
    {
        return clock_.total_duration();
    }

    // Separate
    void run()
    {
        auto timer = clock_.start_timer();
        separate();
    }
    virtual void separate() = 0;

  protected:
    // Calculate coefficients
    static inline Bool calculate_nodetime_coeff(const NodeTime nt, const Path& path)
    {
        const auto t = std::min<Time>(path.size() - 1, nt.t);
        return path[t].n == nt.n;
    }
    static inline Bool calculate_edgetime_coeff(const EdgeTime et, const Path& path)
    {
        return (et.t < path.size() - 1 && path[et.t] == et.e()) ||
               (et.d == Direction::WAIT && et.t >= path.size() - 1 && path.back().n == et.n);
    }
    static inline Bool calculate_move_edgetime_coeff(const EdgeTime et, const Path& path)
    {
        DEBUG_ASSERT(et.d != Direction::WAIT);
        return (et.t < path.size() - 1 && path[et.t] == et.e());
    }
    static inline Bool calculate_wait_edgetime_coeff(const EdgeTime et, const Path& path)
    {
        DEBUG_ASSERT(et.d == Direction::WAIT);
        return (et.t < path.size() - 1 && path[et.t] == et.e()) ||
               (et.t >= path.size() - 1 && path.back().n == et.n);
    }
};
