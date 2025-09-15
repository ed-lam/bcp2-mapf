/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "output/formatting.h"
#include "types/debug.h"
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
    auto empty() const
    {
        return data_.empty();
    }
    Size64 size() const
    {
        return data_.size();
    }
    const auto& operator[](const Size64 index) const
    {
        return data_[index];
    }

    // Clear
    inline void clear()
    {
        data_.clear();
    }

    // Add a penalty for visiting a node (e.g., the target of another agent) at or later than nt.t
    void add(const Cost cost, const NodeTime nt, const OnceOffDirection d)
    {
        DEBUG_ASSERT(nt.t >= 0);
        DEBUG_ASSERT(cost >= 0);
        const Pair<OnceOffDirection, NodeTime> search_item{d, nt};
        auto it = std::lower_bound(
            data_.begin(),
            data_.end(),
            search_item,
            [](const OnceOffPenalty& lhs, const Pair<OnceOffDirection, NodeTime>& rhs)
            {
                return std::make_tuple(lhs.d, lhs.nt.id()) <
                       std::make_tuple(rhs.first, rhs.second.id());
            });
        if (it == data_.end() || it->nt != nt || it->d != d)
        {
            data_.insert(it, {cost, nt, d});
        }
        else
        {
            it->cost += cost;
        }
#ifdef DEBUG
        for (Size64 index = 0; index < data_.size() - 1; ++index)
        {
            DEBUG_ASSERT((data_[index].d < data_[index + 1].d) ||
                         (data_[index].d == data_[index + 1].d &&
                          data_[index].nt.id() < data_[index + 1].nt.id()));
        }
#endif
    }

    // Print
    void print(const Map& map) const
    {
        PRINTLN("Once-off penalties:");
        PRINTLN("{:>12s}{:>8s}{:>12s}{:>18s}", "XY", "T", "Direction", "Cost");
        for (Size64 index = 0; index < data_.size(); ++index)
        {
            const auto& [cost, nt, d] = data_[index];
            PRINTLN("{:>12s}{:>8d}{:>12s}{:>18.2f}",
                    format_node(nt.n, map),
                    nt.t,
                    d == OnceOffDirection::GEq ? ">=" : "<=",
                    cost);
        }
        PRINTLN("");
    }
};
