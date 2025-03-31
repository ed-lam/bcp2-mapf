/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "problem/debug.h"
#include "problem/map.h"
#include "types/map_types.h"
#include "types/span.h"
#include "types/string.h"

class Variable;

String format_node(const Node n, const Map& map);
String format_nodetime(const NodeTime nt, const Map& map);
String format_edge(const Edge e, const Map& map);
String format_edgetime(const EdgeTime et, const Map& map);

// Make a string of the coordinates in a path
String format_path(
    const Span<const Edge>& path,    // Path
    const Map& map,                  // Map
    const String& separator = " "    // Separator
);
String format_path_with_time(
    const Span<const Edge>& path,    // Path
    const Map& map,                  // Map
    const String& separator = " "    // Separator
);
// String format_path(
//     const Edge* const path,          // Path
//     const PathLength path_length,    // Path length
//     const Map& map,                  // Map
//     const String& separator = " "    // Separator
// );

// Make a string of the coordinates in a path in columns
String format_path_spaced(
    const Span<const Edge>& path,    // Path
    const Map& map                   // Map
);
String format_path_with_time_spaced(
    const Span<const Edge>& path,    // Path
    const Map& map                   // Map
);

// Make a string of 0 and 1 of a bitset
String format_bitset(const void* const bitset, const Size size);

// template<>
// struct fmt::formatter<BranchDirection> : formatter<string_view>
// {
//     template<class FormatContext>
//     auto format(const BranchDirection dir, FormatContext& ctx)
//     {
//         return formatter<string_view>::format(dir == BranchDirection::Down ? "down" : "up", ctx);
//     }
// };

template<>
struct fmt::formatter<Direction> : formatter<string_view>
{
    template<class FormatContext>
    inline auto format(const Direction d, FormatContext& ctx) const
    {
        string_view str = "invalid";
        switch (d)
        {
            case Direction::NORTH: str = "north"; break;
            case Direction::SOUTH: str = "south"; break;
            case Direction::EAST:  str = "east";  break;
            case Direction::WEST:  str = "west";  break;
            case Direction::WAIT:  str = "wait";  break;
            default: break;
        }
        return formatter<string_view>::format(str, ctx);
    }
};

void print_solution(const Map& map, const Vector<Variable*>& paths);
