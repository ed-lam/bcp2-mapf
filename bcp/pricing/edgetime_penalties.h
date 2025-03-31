/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "output/formatting.h"
#include "pricing/directional_costs.h"
#include "problem/map.h"
#include "types/basic_types.h"
#include "types/hash_map.h"

class EdgeTimePenalties
{
    HashMap<NodeTime, DirectionalCosts> data_;

  public:
    // Constructors and destructor
    EdgeTimePenalties() = default;
    EdgeTimePenalties(const EdgeTimePenalties& other) = default;
    EdgeTimePenalties(EdgeTimePenalties&& other) noexcept = default;
    EdgeTimePenalties& operator=(const EdgeTimePenalties& other) = default;
    EdgeTimePenalties& operator=(EdgeTimePenalties&& other) noexcept = default;
    ~EdgeTimePenalties() noexcept = default;

    // Getters
    // inline Bool empty() const { return data_.empty(); }

    // Iterators
    inline auto begin() { return data_.begin(); }
    inline auto begin() const { return data_.begin(); }
    inline auto end() { return data_.end(); }
    inline auto end() const { return data_.end(); }
    // inline auto erase(HashMap<NodeTime, DirectionalCosts>::iterator& it) { return data_.erase(it); }

    // Get penalties incoming to a nodetime
    inline DirectionalCosts operator[](const NodeTime nt) const
    {
        debug_assert(nt.t >= 0);
        const auto it = data_.find(nt);
        return it != data_.end() ? it->second : DirectionalCosts{};
    }

    // Clear
    inline void clear() { data_.clear(); }
    inline void move_to_old() { /* Nothing to do. */ }

    // Add a penalty into a nodetime
    void add_nodetime_penalty(const NodeTime nt, const Cost cost)
    {
        debug_assert(nt.t >= 0);
        debug_assert(cost >= 0);
        auto& penalties = data_[nt];
        penalties.north += cost;
        penalties.south += cost;
        penalties.west += cost;
        penalties.east += cost;
        penalties.wait += cost;
    }

    // Add a penalty to an edgetime
    // The penalty is stored in the direction of movement into the nodetime
    void add_edgetime_penalty(const NodeTime nt, const Direction d, const Cost cost)
    {
        debug_assert(cost >= 0);
        auto& penalties = data_[nt];
        penalties[d] += cost;
    }

    // Prepare data structures for efficient lookup before solving
    inline void finalise() { /* Nothing to do. */ }

    // Print
    void print_outgoing(const Map& map) const
    {
        println("Outgoing edgetime penalties:");
        println("{:>20s}{:>8s}{:>15s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}",
                "NT", "N", "XYT", "To North", "Penalty", "To South", "Penalty", "To West", "Penalty", "To East", "Penalty", "To Wait", "Penalty");
        for (const auto& [nt, penalties] : data_)
        {
            if (penalties.north != 0 || penalties.south != 0 || penalties.west != 0 || penalties.east != 0 ||
                penalties.wait != 0)
            {
                const NodeTime north_nt{map.get_destination(nt.n, Direction::NORTH), nt.t + 1};
                const NodeTime south_nt{map.get_destination(nt.n, Direction::SOUTH), nt.t + 1};
                const NodeTime west_nt{map.get_destination(nt.n, Direction::WEST), nt.t + 1};
                const NodeTime east_nt{map.get_destination(nt.n, Direction::EAST), nt.t + 1};
                const NodeTime wait_nt{map.get_destination(nt.n, Direction::WAIT), nt.t + 1};
                println("{:>20d}{:>8d}{:>15s}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}",
                        nt.id, nt.n,
                        format_nodetime(nt, map),
                        format_nodetime(north_nt, map),
                        penalties.north,
                        format_nodetime(south_nt, map),
                        penalties.south,
                        format_nodetime(west_nt, map),
                        penalties.west,
                        format_nodetime(east_nt, map),
                        penalties.east,
                        format_nodetime(wait_nt, map),
                        penalties.wait);
            }
        }
        println("");
    }
    // void print(const Map& map) const
    // {
    //     println("Incoming edgetime penalties:");
    //     println("{:>20s}{:>8s}{:>15s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}{:>16s}{:>14s}",
    //             "NT", "N", "XYT", "To North", "Penalty", "To South", "Penalty", "To West", "Penalty", "To East", "Penalty", "To Wait", "Penalty");
    //     for (const auto& [nt, penalties] : data_)
    //     {
    //         if (penalties.north != 0 || penalties.south != 0 || penalties.west != 0 || penalties.east != 0 ||
    //             penalties.wait != 0)
    //         {
    //             const NodeTime north_nt{map.get_destination(nt.n, Direction::NORTH), nt.t-1};
    //             const NodeTime south_nt{map.get_destination(nt.n, Direction::SOUTH), nt.t-1};
    //             const NodeTime west_nt{map.get_destination(nt.n, Direction::WEST), nt.t-1};
    //             const NodeTime east_nt{map.get_destination(nt.n, Direction::EAST), nt.t-1};
    //             const NodeTime wait_nt{map.get_destination(nt.n, Direction::WAIT), nt.t-1};
    //             println("{:>20d}{:>8d}{:>15s}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}{:>16s}{:>14.4f}",
    //                     nt.id, nt.n,
    //                     format_nodetime(nt, map),
    //                     format_nodetime(south_nt, map),
    //                     penalties.north,
    //                     format_nodetime(north_nt, map),
    //                     penalties.south,
    //                     format_nodetime(east_nt, map),
    //                     penalties.west,
    //                     format_nodetime(west_nt, map),
    //                     penalties.east,
    //                     format_nodetime(wait_nt, map),
    //                     penalties.wait);
    //         }
    //     }
    //     println("");
    // }
};
