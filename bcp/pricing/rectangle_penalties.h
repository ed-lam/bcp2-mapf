/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "output/formatting.h"
#include "types/debug.h"
#include "types/map_types.h"
#include "types/vector.h"

class RectanglePenalties
{
    struct RectanglePenalty
    {
        Cost cost;
        EdgeTime first_ets[2];
        Time length;
        Node n_increment;
    };

    Vector<RectanglePenalty> data_;

  public:
    // Constructors and destructor
    RectanglePenalties() noexcept = default;
    RectanglePenalties(const RectanglePenalties& other) = default;
    RectanglePenalties(RectanglePenalties&& other) noexcept = default;
    RectanglePenalties& operator=(const RectanglePenalties& other) = default;
    RectanglePenalties& operator=(RectanglePenalties&& other) noexcept = default;
    ~RectanglePenalties() noexcept = default;

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

    // Add a penalty for crossing the boundaries of a rectangle
    void add(const Cost cost, const EdgeTime first_entry, const EdgeTime first_exit,
             const Time length, const Node n_increment)
    {
        DEBUG_ASSERT(cost >= 0);
        DEBUG_ASSERT(first_entry.t >= 0);
        data_.push_back({cost, {first_entry, first_exit}, length, n_increment});
    }

    // Print
    void print(const Map& map) const
    {
        PRINTLN("Rectangle penalties:");
        PRINTLN("{:>26s}{:>26s}{:>26s}{:>26s}{:>10s}{:>12s}{:>18s}",
                "First Entry",
                "Last Entry",
                "First Exit",
                "Last Exit",
                "Length",
                "Direction",
                "Cost");
        for (Size64 index = 0; index < data_.size(); ++index)
        {
            const auto& [cost, first_ets, length, n_increment] = data_[index];
            const auto d = first_ets[0].d;
            const auto first_entry = first_ets[0];
            const auto first_exit = first_ets[1];
            const EdgeTime last_entry{
                first_entry.n + n_increment * length, d, first_entry.t + length};
            const EdgeTime last_exit{first_exit.n + n_increment * length, d, first_exit.t + length};
            PRINTLN("{:>26s}{:>26s}{:>26s}{:>26s}{:>10d}{:>12s}{:>18.2f}",
                    format_edgetime(first_entry, map),
                    format_edgetime(last_entry, map),
                    format_edgetime(first_exit, map),
                    format_edgetime(last_exit, map),
                    length,
                    d,
                    cost);
        }
        PRINTLN("");
    }
};
