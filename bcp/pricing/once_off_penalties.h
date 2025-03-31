/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "output/formatting.h"
#include "problem/debug.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/vector.h"

enum class OnceOffDirection : UInt64
{
    LEq = 0,
    GEq = 1
};

class OnceOffPenalties
{
    struct OnceOffPenalty
    {
        Cost cost;
        NodeTime nt;
        OnceOffDirection d;
    };

    Vector<OnceOffPenalty> data_;

  public:
    // Constructors and destructor
    OnceOffPenalties() noexcept = default;
    OnceOffPenalties(const OnceOffPenalties& other) = default;
    OnceOffPenalties(OnceOffPenalties&& other) noexcept = default;
    OnceOffPenalties& operator=(const OnceOffPenalties& other) = default;
    OnceOffPenalties& operator=(OnceOffPenalties&& other) noexcept = default;
    ~OnceOffPenalties() noexcept = default;

    // Getters
    auto empty() const { return data_.empty(); }
    Size size() const { return data_.size(); }
    const auto& operator[](const Size index) const { return data_[index]; }

    // Clear
    inline void clear()
    {
        data_.clear();
    }

    // Add a penalty for visiting a node (e.g., the target of another agent) at or later than nt.t
    void add(const Cost cost, const NodeTime nt, const OnceOffDirection d)
    {
        debug_assert(nt.t >= 0);
        debug_assert(cost >= 0);
        const Pair<OnceOffDirection, NodeTime> search_item{d, nt};
        auto it = std::lower_bound(
            data_.begin(),
            data_.end(),
            search_item,
            [](const OnceOffPenalty& lhs, const Pair<OnceOffDirection, NodeTime>& rhs)
            {
                return std::tie(lhs.d, lhs.nt.id) < std::tie(rhs.first, rhs.second.id);
            }
        );
        if (it == data_.end() || it->nt != nt || it->d != d)
        {
            data_.insert(it, {cost, nt, d});
        }
        else
        {
            it->cost += cost;
        }
#ifdef DEBUG
        for (Size index = 0; index < data_.size() - 1; ++index)
            debug_assert((data_[index].d <  data_[index + 1].d) ||
                         (data_[index].d == data_[index + 1].d && data_[index].nt.id < data_[index + 1].nt.id));
#endif
    }

    // Print
    void print(const Map& map) const
    {
        println("Once-off penalties:");
        println("{:>12s}{:>8s}{:>12s}{:>18s}", "XY", "T", "Direction", "Cost");
        for (Size index = 0; index < data_.size(); ++index)
        {
            const auto& [cost, nt, d] = data_[index];
            println("{:>12s}{:>8d}{:>12s}{:>18.2f}",
                    format_node(nt.n, map),
                    nt.t,
                    d == OnceOffDirection::GEq ? ">=" : "<=",
                    cost);
        }
        println("");
    }
};
