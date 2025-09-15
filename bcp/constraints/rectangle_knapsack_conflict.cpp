/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/rectangle_knapsack_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::IndianRed

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

template <class T>
inline T signum(const T val)
{
    return (T{0} < val) - (val < T{0});
}

inline void append_edge(const Map& map,         // Map
                        const Time t,           // Time of the edge
                        const Position x,       // Coordinate
                        const Position y,       // Coordinate
                        const Direction d,      // Direction
                        Vector<EdgeTime>& edges // Output edges of the rectangle
)
{
    ZoneScopedC(TRACY_COLOUR);

    const auto from_n = map.get_n(x, y);
    const auto to_n = map.get_destination(from_n, d);
    if (map[from_n] && map[to_n])
    {
        edges.emplace_back(from_n, d, t);
    }
}

inline Bool append_horizontal_boundary(
    const Map& map,         // Map
    const Time t0,          // Time at reference location
    const Position x0,      // Coordinate of reference location
    const Position y0,      // Coordinate of reference location
    const Direction d,      // Direction
    const Position x1,      // Start coordinate of the horizontal boundary
    const Position x2,      // End coordinate of the horizontal boundary
    Vector<EdgeTime>& edges // Output edges of the rectangle
)
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate the direction the agent is moving.
    const auto x_direction = signum(x2 - x1);
    if (x_direction == 0)
    {
        return false;
    }

    // Add the edges crossing into the boundary of the rectangle.
    for (Position x = x1; x != x2 + x_direction; x += x_direction)
    {
        const auto t = t0 + x_direction * (x - x0);
        if (0 <= t && 0 <= x && x < map.width() && 0 <= y0 && y0 < map.height())
        {
            append_edge(map, t, x, y0, d, edges);
        }
    }
    return true;
}

inline Bool append_vertical_boundary(const Map& map,    // Map
                                     const Time t0,     // Time at reference location
                                     const Position x0, // Coordinate of reference location
                                     const Position y0, // Coordinate of reference location
                                     const Direction d, // Direction
                                     const Position y1, // Start coordinate of the vertical boundary
                                     const Position y2, // End coordinate of the vertical boundary
                                     Vector<EdgeTime>& edges // Output edges of the rectangle
)
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate the direction the agent is moving.
    const auto y_direction = signum(y2 - y1);
    if (y_direction == 0)
    {
        return false;
    }

    // Add the edges crossing into the boundary of the rectangle.
    for (Position y = y1; y != y2 + y_direction; y += y_direction)
    {
        const auto t = t0 + y_direction * (y - y0);
        if (0 <= t && 0 <= x0 && x0 < map.width() && 0 <= y && y < map.height())
        {
            append_edge(map, t, x0, y, d, edges);
        }
    }
    return true;
}

Bool find_rectangle_conflict(const Map& map,               // Map
                             const Projection& projection, // Solution in the original space
                             const Agent a1,               // Agent 1
                             const Agent a2,               // Agent 2
                             const Time conflict_time,     // Time of the conflict
                             const Path& a1_path,          // Path of agent 1
                             const Path& a2_path,          // Path of agent 1
                             const Time min_path_length,   // Length of the shorter path
                             Vector<EdgeTime>& rectangle,  // Edgetimes of the rectangle
                             Size32& num_a1_ets,           // Number of edgetimes for agent 1
                             Size32& num_a2_ets,           // Number of edgetimes for agent 2
                             NodeTime& a1_start,           // Start nodetime of agent 1
                             NodeTime& a2_start,           // Start nodetime of agent 2
                             NodeTime& a1_end,             // End nodetime of agent 1
                             NodeTime& a2_end              // End nodetime of agent 2
#if defined(DEBUG) or defined(PRINT_DEBUG)
                             ,
                             Real64& output_lhs // LHS
#endif
)
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN("    Checking conflict at {} for agents {} and {}",
            format_nodetime(NodeTime{a1_path[conflict_time].n, conflict_time}, map),
            a1,
            a2);

    // Get the movement directions.
    Direction x_dir = Direction::INVALID;
    Direction y_dir = Direction::INVALID;
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (a1_path[t].d == Direction::EAST || a1_path[t].d == Direction::WEST)
        {
            x_dir = a1_path[t].d;
            break;
        }
        else if (a2_path[t].d == Direction::EAST || a2_path[t].d == Direction::WEST)
        {
            x_dir = a2_path[t].d;
            break;
        }
        else if (a1_path[t].d == Direction::WAIT || a2_path[t].d == Direction::WAIT)
        {
            return false;
        }
    }
    for (Time t = conflict_time; t < min_path_length; ++t)
    {
        if (a1_path[t].d == Direction::NORTH || a1_path[t].d == Direction::SOUTH)
        {
            y_dir = a1_path[t].d;
            break;
        }
        else if (a2_path[t].d == Direction::NORTH || a2_path[t].d == Direction::SOUTH)
        {
            y_dir = a2_path[t].d;
            break;
        }
        else if (a1_path[t].d == Direction::WAIT || a2_path[t].d == Direction::WAIT)
        {
            return false;
        }
    }
    if (x_dir == Direction::INVALID || y_dir == Direction::INVALID)
    {
        return false;
    }

    // Find the first time when the direction changes.
    Time start_t = conflict_time;
    Time end_t = conflict_time;
    for (; (start_t >= 0) && (a1_path[start_t].d == x_dir || a1_path[start_t].d == y_dir) &&
           (a2_path[start_t].d == x_dir || a2_path[start_t].d == y_dir);
         --start_t)
        ;
    start_t++;
    for (; (end_t < min_path_length - 1) &&
           (a1_path[end_t].d == x_dir || a1_path[end_t].d == y_dir) &&
           (a2_path[end_t].d == x_dir || a2_path[end_t].d == y_dir);
         ++end_t)
        ;
    if (end_t <= start_t + 2)
    {
        return false;
    }
    DEBUG_ASSERT(0 <= start_t && end_t > start_t + 2 && end_t < min_path_length);

    // Cannot find a rectangle conflict if the start location of the two agents are the same.
    if (a1_path[start_t].n == a2_path[start_t].n)
    {
        return false;
    }

    // Check.
#ifdef DEBUG
    for (Time t = start_t; t < end_t; ++t)
    {
        DEBUG_ASSERT(a1_path[t].d != Direction::WAIT);
        DEBUG_ASSERT(a2_path[t].d != Direction::WAIT);
    }
#endif

    // Get the coordinates of those times.
    a1_start = NodeTime{a1_path[start_t].n, start_t};
    a2_start = NodeTime{a2_path[start_t].n, start_t};
    a1_end = NodeTime{a1_path[end_t].n, end_t};
    a2_end = NodeTime{a2_path[end_t].n, end_t};
    const auto [a1_start_x, a1_start_y] = map.get_xy(a1_start.n);
    const auto [a2_start_x, a2_start_y] = map.get_xy(a2_start.n);
    const auto [a1_end_x, a1_end_y] = map.get_xy(a1_end.n);
    const auto [a2_end_x, a2_end_y] = map.get_xy(a2_end.n);

    // Check that there is no wait inside the rectangle.
    DEBUG_ASSERT(std::abs(a1_end_x - a1_start_x) + std::abs(a1_end_y - a1_start_y) ==
                 end_t - start_t);
    DEBUG_ASSERT(std::abs(a2_end_x - a2_start_x) + std::abs(a2_end_y - a2_start_y) ==
                 end_t - start_t);

    // Calculate the direction of the two agents.
    Direction a1_dir;
    Direction a2_dir;
    if (a1_start_x <= a2_start_x && a1_start_y <= a2_start_y && a1_end_x >= a2_end_x &&
        a1_end_y >= a2_end_y && a1_start_x <= a1_end_x && a1_start_y >= a1_end_y &&
        a2_start_x <= a2_end_x && a2_start_y >= a2_end_y)
    {
        a1_dir = Direction::EAST;
        a2_dir = Direction::NORTH;
    }
    else if (a2_start_x <= a1_start_x && a2_start_y <= a1_start_y && a2_end_x >= a1_end_x &&
             a2_end_y >= a1_end_y && a2_start_x <= a2_end_x && a2_start_y >= a2_end_y &&
             a1_start_x <= a1_end_x && a1_start_y >= a1_end_y)
    {
        a1_dir = Direction::NORTH;
        a2_dir = Direction::EAST;
    }
    else if (a1_start_x >= a2_start_x && a1_start_y <= a2_start_y && a1_end_x <= a2_end_x &&
             a1_end_y >= a2_end_y && a1_start_x >= a1_end_x && a1_start_y >= a1_end_y &&
             a2_start_x >= a2_end_x && a2_start_y >= a2_end_y)
    {
        a1_dir = Direction::WEST;
        a2_dir = Direction::NORTH;
    }
    else if (a2_start_x >= a1_start_x && a2_start_y <= a1_start_y && a2_end_x <= a1_end_x &&
             a2_end_y >= a1_end_y && a2_start_x >= a2_end_x && a2_start_y >= a2_end_y &&
             a1_start_x >= a1_end_x && a1_start_y >= a1_end_y)
    {
        a1_dir = Direction::NORTH;
        a2_dir = Direction::WEST;
    }
    else if (a1_start_x <= a2_start_x && a1_start_y >= a2_start_y && a1_end_x >= a2_end_x &&
             a1_end_y <= a2_end_y && a1_start_x <= a1_end_x && a1_start_y <= a1_end_y &&
             a2_start_x <= a2_end_x && a2_start_y <= a2_end_y)
    {
        a1_dir = Direction::EAST;
        a2_dir = Direction::SOUTH;
    }
    else if (a2_start_x <= a1_start_x && a2_start_y >= a1_start_y && a2_end_x >= a1_end_x &&
             a2_end_y <= a1_end_y && a2_start_x <= a2_end_x && a2_start_y <= a2_end_y &&
             a1_start_x <= a1_end_x && a1_start_y <= a1_end_y)
    {
        a1_dir = Direction::SOUTH;
        a2_dir = Direction::EAST;
    }
    else if (a1_start_x >= a2_start_x && a1_start_y >= a2_start_y && a1_end_x <= a2_end_x &&
             a1_end_y <= a2_end_y && a1_start_x >= a1_end_x && a1_start_y <= a1_end_y &&
             a2_start_x >= a2_end_x && a2_start_y <= a2_end_y)
    {
        a1_dir = Direction::WEST;
        a2_dir = Direction::SOUTH;
    }
    else if (a2_start_x >= a1_start_x && a2_start_y >= a1_start_y && a2_end_x <= a1_end_x &&
             a2_end_y <= a1_end_y && a2_start_x >= a2_end_x && a2_start_y <= a2_end_y &&
             a1_start_x >= a1_end_x && a1_start_y <= a1_end_y)
    {
        a1_dir = Direction::SOUTH;
        a2_dir = Direction::WEST;
    }
    else
    {
        return false;
    }

    // Compute exit time.
    const auto exit_t = end_t - 1;

    // Append the edges of agent 1.
    rectangle.clear();
    if (a1_dir == Direction::NORTH || a1_dir == Direction::SOUTH)
    {
        const auto x_bound = a2_start_x + (a2_dir == Direction::EAST ? 1 : -1);
        const auto y_bound = a1_end_y + (a1_dir == Direction::NORTH ? 1 : -1);
        if (!append_horizontal_boundary(
                map, start_t, a1_start_x, a1_start_y, a1_dir, x_bound, a2_end_x, rectangle))
        {
            return false;
        }

        if (!append_horizontal_boundary(
                map, exit_t, a1_end_x, y_bound, a1_dir, x_bound, a2_end_x, rectangle))
        {
            return false;
        }
    }
    else
    {
        DEBUG_ASSERT(a1_dir == Direction::EAST || a1_dir == Direction::WEST);

        const auto y_bound = a2_start_y + (a2_dir == Direction::NORTH ? -1 : 1);
        const auto x_bound = a1_end_x + (a1_dir == Direction::EAST ? -1 : 1);

        if (!append_vertical_boundary(
                map, start_t, a1_start_x, a1_start_y, a1_dir, y_bound, a2_end_y, rectangle))
        {
            return false;
        }

        if (!append_vertical_boundary(
                map, exit_t, x_bound, a1_end_y, a1_dir, y_bound, a2_end_y, rectangle))
        {
            return false;
        }
    }
    num_a1_ets = rectangle.size();

    // Append the edges of agent 2.
    if (a2_dir == Direction::NORTH || a2_dir == Direction::SOUTH)
    {
        const auto x_bound = a1_start_x + (a1_dir == Direction::EAST ? 1 : -1);
        const auto y_bound = a2_end_y + (a2_dir == Direction::NORTH ? 1 : -1);

        if (!append_horizontal_boundary(
                map, start_t, a2_start_x, a2_start_y, a2_dir, x_bound, a1_end_x, rectangle))
        {
            return false;
        }

        if (!append_horizontal_boundary(
                map, exit_t, a2_end_x, y_bound, a2_dir, x_bound, a1_end_x, rectangle))
        {
            return false;
        }
    }
    else
    {
        DEBUG_ASSERT(a2_dir == Direction::EAST || a2_dir == Direction::WEST);

        const auto y_bound = a1_start_y + (a1_dir == Direction::NORTH ? -1 : 1);
        const auto x_bound = a2_end_x + (a2_dir == Direction::EAST ? -1 : 1);

        if (!append_vertical_boundary(
                map, start_t, a2_start_x, a2_start_y, a2_dir, y_bound, a1_end_y, rectangle))
        {
            return false;
        }

        if (!append_vertical_boundary(
                map, exit_t, x_bound, a2_end_y, a2_dir, y_bound, a1_end_y, rectangle))
        {
            return false;
        }
    }
    num_a2_ets = rectangle.size() - num_a1_ets;

    // Determine if the cut is violated.
    Real64 lhs = 0.0;
    for (Size32 idx = 0; idx < num_a1_ets; ++idx)
    {
        const auto& et = rectangle[idx];
        const auto val = projection.find_agent_move_edgetime(a1, et);
        lhs += val;
        DEBUGLN("        Agent {}, edge {}, val {}", a1, format_edgetime(et, map), val);
    }
    for (Size32 idx = num_a1_ets; idx < num_a1_ets + num_a2_ets; ++idx)
    {
        const auto& et = rectangle[idx];
        const auto val = projection.find_agent_move_edgetime(a2, et);
        lhs += val;
        DEBUGLN("        Agent {}, edge {}, val {}", a2, format_edgetime(et, map), val);
    }
    DEBUG_ASSERT(is_le(lhs, 4.0));
    DEBUGLN("        LHS: {:.4f}", lhs);

    // Store outputs.
#if defined(DEBUG) or defined(PRINT_DEBUG)
    output_lhs = lhs;
#endif

    // Check if the cut is violated.
    return is_gt(lhs, 3.0 + CUT_VIOLATION);
}

void RectangleKnapsackConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    DEBUGLN("Starting separator for rectangle knapsack conflicts");

    // Get the problem data.
    const auto A = instance_.num_agents();
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    Vector<EdgeTime> rectangle;
    Size32 num_a1_ets;
    Size32 num_a2_ets;
    NodeTime a1_start;
    NodeTime a2_start;
    NodeTime a1_end;
    NodeTime a2_end;
#if defined(DEBUG) or defined(PRINT_DEBUG)
    Real64 lhs;
#endif
    Size64 num_separated_this_run = 0;
    for (Agent a1 = 0; a1 < A - 1; ++a1)
        for (Agent a2 = a1 + 1; a2 < A; ++a2)
        {
            for (const auto& variable1 : master.agent_variables(a1))
                if (const auto val1 = master.variable_primal_sol(variable1); is_gt(val1, 0.0))
                    for (const auto& variable2 : master.agent_variables(a2))
                        if (const auto val2 = master.variable_primal_sol(variable2);
                            is_gt(val2, 0.0))
                        {
                            // Find a vertex conflict and search outward to find a rectangle
                            // conflict.
                            const auto& path1 = variable1.path();
                            const auto& path2 = variable2.path();
                            const auto min_path_length = std::min(path1.size(), path2.size());
                            for (Time conflict_time = 1; conflict_time < min_path_length - 1;
                                 ++conflict_time)
                                if (path1[conflict_time].n == path2[conflict_time].n &&
                                    find_rectangle_conflict(map,
                                                            projection,
                                                            a1,
                                                            a2,
                                                            conflict_time,
                                                            path1,
                                                            path2,
                                                            min_path_length,
                                                            rectangle,
                                                            num_a1_ets,
                                                            num_a2_ets,
                                                            a1_start,
                                                            a2_start,
                                                            a1_end,
                                                            a2_end
#if defined(DEBUG) or defined(PRINT_DEBUG)
                                                            ,
                                                            lhs
#endif
                                                            ))
                                {
                                    // Print.
                                    DEBUGLN("        Creating rectangle knapsack cut for agents {} "
                                            "and {} from corners {}, {}, {} and {} with LHS {}",
                                            a1,
                                            a2,
                                            format_nodetime(a1_start, map),
                                            format_nodetime(a2_start, map),
                                            format_nodetime(a1_end, map),
                                            format_nodetime(a2_end, map),
                                            lhs);

                                    // Create the row.
                                    create_row(a1,
                                               a2,
                                               a1_start,
                                               a2_start,
                                               a1_end,
                                               a2_end,
                                               num_a1_ets,
                                               num_a2_ets,
                                               rectangle);

                                    // Exit if enough cuts are found.
                                    ++num_separated_this_run;
                                    if (num_separated_this_run >= MAX_CUTS_PER_RUN)
                                    {
                                        return;
                                    }

                                    // Skip to the next pair of agents.
                                    goto NEXT_AGENT_PAIR;
                                }
                        }
        NEXT_AGENT_PAIR:;
        }
}

void RectangleKnapsackConflictSeparator::create_row(const Agent a1, const Agent a2,
                                                    const NodeTime a1_start,
                                                    const NodeTime a2_start, const NodeTime a1_end,
                                                    const NodeTime a2_end, const Size32 num_a1_ets,
                                                    const Size32 num_a2_ets,
                                                    const Vector<EdgeTime>& rectangle)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Create the row.
    auto name = fmt::format("rectangle({},{},{},{},{},{})",
                            a1,
                            format_nodetime(a1_start, map),
                            format_nodetime(a1_end, map),
                            a2,
                            format_nodetime(a2_start, map),
                            format_nodetime(a2_end, map));
    const auto num_ets = num_a1_ets + num_a2_ets;
    DEBUG_ASSERT(num_ets == rectangle.size());
    const auto data_size = sizeof(ConstraintData) + sizeof(EdgeTime) * num_ets;
    const auto hash_size = data_size;
    auto constraint = Constraint::construct(
        '<', 3.0, 2, data_size, hash_size, &apply_in_pricer, &get_coeff, name);
    auto data = new (constraint->data()) ConstraintData;
    data->a1 = a1;
    data->a2 = a2;
    data->a1_start = a1_start;
    data->a2_start = a2_start;
    data->a1_end = a1_end;
    data->a2_end = a2_end;
    data->num_a1_ets = num_a1_ets;
    data->num_ets = num_ets;
    new (data->ets) EdgeTime[rectangle.size()];
    std::copy(rectangle.begin(), rectangle.end(), data->ets);
    master.add_row(std::move(constraint));
    ++num_added_;
}

void RectangleKnapsackConflictSeparator::apply_in_pricer(const Constraint& constraint,
                                                         const Real64 dual, Pricer& pricer)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& data = *reinterpret_cast<const ConstraintData*>(constraint.data());
    const auto a1 = data.a1;
    const auto a2 = data.a2;

    // Add the dual solution to the reduced cost function.
    for (const auto& et : data.a1_ets())
    {
        // DEBUG_ASSERT(instance_.map[et.n]);
        // DEBUG_ASSERT(instance_.map[instance_.map.get_destination(et)]);
        pricer.add_edgetime_penalty_one_agent(a1, et, -dual);
    }
    for (const auto& et : data.a2_ets())
    {
        // DEBUG_ASSERT(instance_.map[et.n]);
        // DEBUG_ASSERT(instance_.map[instance_.map.get_destination(et)]);
        pricer.add_edgetime_penalty_one_agent(a2, et, -dual);
    }
}

Real64 RectangleKnapsackConflictSeparator::get_coeff(const Constraint& constraint, const Agent a,
                                                     const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& data = *reinterpret_cast<const ConstraintData*>(constraint.data());
    const auto a1 = data.a1;
    const auto a2 = data.a2;

    // Calculate coefficient.
    if (a == a1)
    {
        return calculate_coeff(data.a1_ets(), path);
    }
    else if (a == a2)
    {
        return calculate_coeff(data.a2_ets(), path);
    }
    else
    {
        return 0.0;
    }
}

Real64 RectangleKnapsackConflictSeparator::calculate_coeff(const Span<const EdgeTime>& ets,
                                                           const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    Real64 coeff = 0.0;
    for (const auto& et : ets)
    {
        coeff += calculate_move_edgetime_coeff(et, path);
        // PRINTLN("{} {}", format_edgetime(et, problem_.instance().map),
        // calculate_move_edgetime_coeff(et, path));
    }
    return coeff;
}
