/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "master/variable.h"
#include "output/formatting.h"
#include "problem/instance.h"
#include "types/bitset.h"
#include "types/float_compare.h"
#include "types/hash_map.h"
#include <fmt/color.h>

String format_node(const Node n, const Map& map)
{
    return fmt::format("({},{})", map.get_x(n), map.get_y(n));
}

String format_nodetime(const NodeTime nt, const Map& map)
{
    return fmt::format("({},{})", format_node(nt.n, map), nt.t);
}

String format_edge(const Edge e, const Map& map)
{
    const auto dest = map.get_destination(e);
    return fmt::format("({},{})", format_node(e.n, map), format_node(dest, map));
}

String format_edgetime(const EdgeTime et, const Map& map)
{
    const auto dest = map.get_destination(et);
    return fmt::format("({},{},{})", format_node(et.n, map), et.t, format_node(dest, map));
}

// Make a string of the coordinates in a path
String format_path(
    const Span<const Edge>& path,    // Path
    const Map& map,                  // Map
    const String& separator          // Separator
)
{
    String str;
    Time t = 0;
    str.append(format_node(path[t].n, map));
    for (++t; t < path.size(); ++t)
    {
        str.append(fmt::format("{}{}", separator, format_node(path[t].n, map)));
    }
    return str;
}
String format_path_with_time(
    const Span<const Edge>& path,    // Path
    const Map& map,                  // Map
    const String& separator          // Separator
)
{
    String str;
    Time t = 0;
    str.append(format_nodetime(NodeTime{path[t].n, t}, map));
    for (++t; t < path.size(); ++t)
    {
        str.append(fmt::format("{}{}", separator, format_nodetime(NodeTime{path[t].n, t}, map)));
    }
    return str;
}

// Make a string of the coordinates in a path in columns
String format_path_spaced(
    const Span<const Edge>& path,    // Path
    const Map& map                   // Map
)
{
    String str;
    for (Time t = 0; t < path.size(); ++t)
    {
        str.append(fmt::format("{:>9s}", format_node(path[t].n, map)));
    }
    return str;
}
String format_path_with_time_spaced(
    const Span<const Edge>& path,    // Path
    const Map& map                   // Map
)
{
    String str;
    for (Time t = 0; t < path.size(); ++t)
    {
        str.append(fmt::format("{:>13s}", format_nodetime(NodeTime{path[t].n, t}, map)));
    }
    return str;
}

// Make a string of 0 and 1 of a bitset
String format_bitset(const void* const bitset, const Size size)
{
    const auto max_length = (size + 7) & (-8);
    const auto num_bytes = max_length / 8;
    String str;
    for (Size i = num_bytes - 1; i >= 0; --i)
    {
        str += fmt::format("{:08b}", static_cast<const char*>(bitset)[i]);
    }
    str.erase(0, max_length - size);
    return str;
}

void print_solution(const Map& map, const Vector<Variable*>& paths)
{
    // Get the problem data.
    const Agent A = paths.size();

    // Find the makespan.
    Time makespan = 0;
    for (Agent a = 0; a < A; ++a)
    {
        const auto& path = paths[a]->path();
        makespan = std::max<Time>(makespan, path.size());
    }

    // Print time header line.
    fmt::print("              ");
    for (Time t = 0; t < makespan; ++t)
    {
        fmt::print("{:>11d}", t);
    }
    println("");

    // Print paths.
    for (Agent a = 0; a < A; ++a)
    {
        // Print value.
        fmt::print("    ");
        fmt::print("Agent {:3d}:", a);

        // Print path.
        const auto& path = paths[a]->path();
        for (Time t = 0; t < path.size(); ++t)
        {
            // Get the nodetime and edgetime.
            const auto x = map.get_x(path[t].n) - 1;
            const auto y = map.get_y(path[t].n) - 1;
            const NodeTime nt{path[t].n, t};
            const EdgeTime et{map.get_undirected_edge(path[t]), t};

            // Print.
            fmt::print("{:>11}", fmt::format("({},{})", x, y));
        }
        println("");
    }
}
