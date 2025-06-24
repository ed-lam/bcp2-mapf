/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/rectangle_clique_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"
#include <deque>

#define TRACY_COLOUR tracy::Color::ColorType::IndianRed

#define CUT_VIOLATION (0.1)
#define MAX_CUTS_PER_RUN (1000)

// template<class T>
// inline T signum(const T val)
// {
//     return (T{0} < val) - (val < T{0});
// }

struct MDDNode
{
    Agent a;
    NodeTime nt;
    MDDNode* prev[5];
    MDDNode* next[5];
    MDDNode* next_agent;
};

RectangleCliqueConflictSeparator::RectangleCliqueConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    memory_pool_(),
    agent_mdd_(instance_.agents.size()),
    mdd_next_agent_(),
    candidates_(),
    num_separated_(instance.agents.size(), instance.agents.size())
{
}

void RectangleCliqueConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for rectangle clique conflicts");

    // Get the problem data.
    const Agent A = instance_.agents.size();
    const auto& map = instance_.map;
    const auto& master = problem_.master();
    // master.print_paths();

    // Get the MDD of the paths.
    memory_pool_.reset(sizeof(MDDNode));
    mdd_next_agent_.clear();
    for (Agent a = A - 1; a >= 0; --a)
    {
        auto& mdd = agent_mdd_[a];
        mdd.clear();
        for (const auto& variable : master.agent_variables(a))
        {
            const auto val = master.variable_primal_sol(variable);
            if (is_gt(val, 0.0))
            {
                Direction prev_d;
                MDDNode* prev = nullptr;
                const auto& path = variable.path();
                for (Time t = 0; t < path.size(); ++t)
                {
                    const auto e = path[t];
                    const NodeTime nt{e.n, t};
                    auto& node = mdd[nt];
                    if (!node)
                    {
                        // Store the agent and nodetime.
                        node = static_cast<MDDNode*>(memory_pool_.get_buffer<true, true>());
                        node->a = a;
                        node->nt = nt;

                        // Link the new node to the predecessor.
                        if (prev)
                        {
                            debug_assert(prev_d < 5);
                            node->prev[prev_d] = prev;
                            prev->next[prev_d] = node;
                        }

                        // Link the new node to the node of the previous agent visiting the same nodetime.
                        auto& next_agent_node = mdd_next_agent_[nt];
                        if (!next_agent_node || next_agent_node->a != a)
                        {
                            node->next_agent = next_agent_node;
                            debug_assert(!node->next_agent || node->next_agent->a != a);
                            debug_assert(!node->next_agent || node->next_agent->nt == nt);
                            next_agent_node = node;
                        }
                    }
                    prev = node;
                    prev_d = e.d;
                }
            }
        }
    }

    // Find cuts.
    candidates_.clear();
    std::deque<MDDNode*> mdd_queue;
    std::deque<Pair<MDDNode*, MDDNode*>> conflict_queue;
    HashSet<NodeTime> rect_starts;
    HashSet<NodeTime> rect_ends;
    for (Agent a1 = 0; a1 < A; ++a1)
    {
        // Loop through the MDD nodes of the first agent.
        auto root = agent_mdd_[a1].at(NodeTime{instance_.agents[a1].start, 0});
        mdd_queue.clear();
        mdd_queue.push_back(root);
        while (!mdd_queue.empty())
        {
            // Get an MDD node.
            const auto a1_conflict = mdd_queue.front();
            debug_assert(a1 == a1_conflict->a);
            mdd_queue.pop_front();

            // Loop through nodetime conflicts of the second agent.
            for (auto a2_conflict = a1_conflict->next_agent; a2_conflict; a2_conflict = a2_conflict->next_agent)
            {
                // Get the second agent.
                const auto a2 = a2_conflict->a;
                debug_assert(a2 > a1);
                debug_assert(a1_conflict->nt == a2_conflict->nt);
                debugln("Node conflict for agents {} and {} at {}", a1, a2, format_nodetime(a1_conflict->nt, map));

                // Loop through the four diagonal directions of a potential rectangle.
                for (const auto& [x_d, y_d] : rectangle_directions())
                {
                    // Get the increment amount for the movement directions.
                    const auto x_step = 2 * static_cast<Time>(x_d == EAST) - 1;
                    const auto y_step = 2 * static_cast<Time>(y_d == SOUTH) - 1;
                    const auto x_n_increment = map.get_direction_n_offset(x_d);
                    const auto y_n_increment = map.get_direction_n_offset(y_d);

                    // Find the start coordinates and time of a rectangle.
                    rect_starts.clear();
                    conflict_queue.clear();
                    conflict_queue.push_back({a1_conflict, a2_conflict});
                    while (!conflict_queue.empty())
                    {
                        // Get the MDD nodes of the two agents.
                        const auto [a1_node, a2_node] = conflict_queue.front();
                        debug_assert(a1_node->nt.t == a2_node->nt.t);
                        conflict_queue.pop_front();

                        // Stop traversing the MDD if time 0 is reached or if the agent uses a direction that is not in
                        // the direction of the rectangle.
                        Bool stop = (a1_node->nt.t == 0);
                        for (Size d = 0; d < 4; ++d)
                            if (d != x_d && d != y_d && (a1_node->prev[d] || a2_node->prev[d]))
                            {
                                stop = true;
                                break;
                            }
                        if (!stop)
                        {
                            for (Size a1_d = 0; a1_d < 4; ++a1_d)
                                for (Size a2_d = 0; a2_d < 4; ++a2_d)
                                    if (a1_node->prev[a1_d] && a2_node->prev[a2_d])
                                    {
                                        debug_assert(a1_d == x_d || a1_d == y_d);
                                        debug_assert(a2_d == x_d || a2_d == y_d);
                                        conflict_queue.push_back({a1_node->prev[a1_d], a2_node->prev[a2_d]});
                                    }
                        }
                        else if (a1_node != a1_conflict)
                        {
                            // Found the start coordinates of a rectangle if the exploration has progressed beyond the
                            // initial nodetime conflict.
                            const auto [a1_x, a1_y] = map.get_xy(a1_node->nt.n);
                            const auto [a2_x, a2_y] = map.get_xy(a2_node->nt.n);
                            const auto x = x_d == WEST ? std::max(a1_x, a2_x) : std::min(a1_x, a2_x);
                            const auto y = y_d == NORTH ? std::max(a1_y, a2_y) : std::min(a1_y, a2_y);
                            const auto t = a1_node->nt.t - std::abs(a1_x - x) - std::abs(a1_y - y);
                            rect_starts.emplace(map.get_n(x, y), t);
                            debugln("Found rectangle start ({},{},{}) induced by {} of agent {} and {} of agent {}",
                                    x, y, t,
                                    format_nodetime(a1_node->nt, map), a1,
                                    format_nodetime(a2_node->nt, map), a2);
                        }
                    }

                    // Find the end coordinates and time of a rectangle.
                    rect_ends.clear();
                    conflict_queue.clear();
                    conflict_queue.push_back({a1_conflict, a2_conflict});
                    while (!conflict_queue.empty())
                    {
                        // Get the MDD nodes of the two agents.
                        const auto [a1_node, a2_node] = conflict_queue.front();
                        debug_assert(a1_node->nt.t == a2_node->nt.t);
                        conflict_queue.pop_front();

                        // Stop traversing the MDD if the target is reached or if the agent uses a direction that is not
                        // in the direction of the rectangle.
                        Bool stop = (!a1_node->next[0] &&
                                     !a1_node->next[1] &&
                                     !a1_node->next[2] &&
                                     !a1_node->next[3] &&
                                     !a1_node->next[4]) ||
                                    (!a2_node->next[0] &&
                                     !a2_node->next[1] &&
                                     !a2_node->next[2] &&
                                     !a2_node->next[3] &&
                                     !a2_node->next[4]);
                        for (Size d = 0; d < 4; ++d)
                            if (d != x_d && d != y_d && (a1_node->next[d] || a2_node->next[d]))
                            {
                                stop = true;
                                break;
                            }
                        if (!stop)
                        {
                            for (Size a1_d = 0; a1_d < 4; ++a1_d)
                                for (Size a2_d = 0; a2_d < 4; ++a2_d)
                                    if (a1_node->next[a1_d] && a2_node->next[a2_d])
                                    {
                                        debug_assert(a1_d == x_d || a1_d == y_d);
                                        debug_assert(a2_d == x_d || a2_d == y_d);
                                        conflict_queue.push_back({a1_node->next[a1_d], a2_node->next[a2_d]});
                                    }
                        }
                        else if (a1_node != a1_conflict)
                        {
                            // Found the end coordinates of a rectangle if the exploration has progressed beyond the
                            // initial nodetime conflict.
                            const auto [a1_x, a1_y] = map.get_xy(a1_node->nt.n);
                            const auto [a2_x, a2_y] = map.get_xy(a2_node->nt.n);
                            const auto x = x_d == WEST ? std::min(a1_x, a2_x) : std::max(a1_x, a2_x);
                            const auto y = y_d == NORTH ? std::min(a1_y, a2_y) : std::max(a1_y, a2_y);
                            const auto t = a1_node->nt.t + std::abs(a1_x - x) + std::abs(a1_y - y);
                            rect_ends.emplace(map.get_n(x, y), t);
                            debugln("Found rectangle end ({},{},{}) induced by {} of agent {} and {} of agent {}",
                                    x, y, t,
                                    format_nodetime(a1_node->nt, map), a1,
                                    format_nodetime(a2_node->nt, map), a2);
                        }
                    }

                    // Check for violations and create cuts.
                    for (const auto rect_start : rect_starts)
                        for (const auto rect_end : rect_ends)
                        {
                            // Stop if the rectangle does not have any width or height.
                            const auto rect_start_xy = map.get_xy(rect_start.n);
                            const auto rect_end_xy = map.get_xy(rect_end.n);
                            if (std::abs(rect_start_xy.x - rect_end_xy.x) <= 1 ||
                                std::abs(rect_start_xy.y - rect_end_xy.y) <= 1)
                            {
                                continue;
                            }

                            // Compute the exit coordinates of the rectangle.
                            const XY rect_exit_xy{rect_end_xy.x - x_step, rect_end_xy.y - y_step};
                            const NodeTime rect_exit{map.get_n(rect_exit_xy), rect_end.t - 2};
                            debug_assert(rect_exit.t == rect_start.t +
                                                       std::abs(rect_exit_xy.x - rect_start_xy.x) +
                                                       std::abs(rect_exit_xy.y - rect_start_xy.y));

                            // Get the first edgetime of the two entry boundaries.
                            const EdgeTime x_first_entry{rect_start.n, x_d, rect_start.t};
                            const EdgeTime y_first_entry{rect_start.n, y_d, rect_start.t};

                            // Get the first edgetime of the two exit boundaries.
                            const auto x_length = std::abs(rect_exit_xy.x - rect_start_xy.x);
                            const auto y_length = std::abs(rect_exit_xy.y - rect_start_xy.y);
                            const EdgeTime x_first_exit{map.get_n(rect_exit_xy.x, rect_start_xy.y),
                                                        x_d,
                                                        rect_start.t + x_length};
                            const EdgeTime y_first_exit{map.get_n(rect_start_xy.x, rect_exit_xy.y),
                                                        y_d,
                                                        rect_start.t + y_length};
                            debug_assert(x_first_entry != x_first_exit);
                            debug_assert(y_first_entry != y_first_exit);

                            // Print.
#ifdef PRINT_DEBUG
                            debugln("Horizontal entry:");
                            for (Time i = 0; i <= y_length; ++i)
                            {
                                const auto n = x_first_entry.n + i * y_n_increment;
                                const auto t = x_first_entry.t + i;
                                const EdgeTime et{n, x_d, t};
                                debugln("    {}", format_edgetime(et, map));
                            }
                            debugln("Horizontal exit:");
                            for (Time i = 0; i <= y_length; ++i)
                            {
                                const auto n = x_first_exit.n + i * y_n_increment;
                                const auto t = x_first_exit.t + i;
                                const EdgeTime et{n, x_d, t};
                                debugln("    {}", format_edgetime(et, map));
                            }
                            debugln("Vertical entry:");
                            for (Time i = 0; i <= x_length; ++i)
                            {
                                const auto n = y_first_entry.n + i * x_n_increment;
                                const auto t = y_first_entry.t + i;
                                const EdgeTime et{n, y_d, t};
                                debugln("    {}", format_edgetime(et, map));
                            }
                            debugln("Vertical exit:");
                            for (Time i = 0; i <= x_length; ++i)
                            {
                                const auto n = y_first_exit.n + i * x_n_increment;
                                const auto t = y_first_exit.t + i;
                                const EdgeTime et{n, y_d, t};
                                debugln("    {}", format_edgetime(et, map));
                            }
#endif

                            // Compute the LHS if the first agent moves hozirontal and the second agent moves vertical.
                            Float xy_lhs = 0.0;
                            for (const auto& variable : master.agent_variables(a1))
                            {
                                const auto val = master.variable_primal_sol(variable);
                                const auto& path = variable.path();
                                Size num_crossings = 0;
                                for (Time i = 0; i <= y_length; ++i)
                                {
                                    const Edge e{x_first_entry.n + i * y_n_increment, x_d};
                                    const auto t = x_first_entry.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                for (Time i = 0; i <= y_length; ++i)
                                {
                                    const Edge e{x_first_exit.n + i * y_n_increment, x_d};
                                    const auto t = x_first_exit.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                xy_lhs += (num_crossings == 2) * val;

                                // if (num_crossings == 2)
                                {
                                    debugln("XY, agent {:3d}, val {:8.4f}, crossings {}, path {}",
                                            a1, val, num_crossings, format_path_with_time_spaced(path, map));
                                }
                            }
                            for (const auto& variable : master.agent_variables(a2))
                            {
                                const auto val = master.variable_primal_sol(variable);
                                const auto& path = variable.path();
                                Size num_crossings = 0;
                                for (Time i = 0; i <= x_length; ++i)
                                {
                                    const Edge e{y_first_entry.n + i * x_n_increment, y_d};
                                    const auto t = y_first_entry.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                for (Time i = 0; i <= x_length; ++i)
                                {
                                    const Edge e{y_first_exit.n + i * x_n_increment, y_d};
                                    const auto t = y_first_exit.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                xy_lhs += (num_crossings == 2) * val;

                                // if (num_crossings == 2)
                                {
                                    debugln("XY, agent {:3d}, val {:8.4f}, crossings {}, path {}",
                                            a2, val, num_crossings, format_path_with_time_spaced(path, map));
                                }
                            }

                            // Compute the LHS if the first agent moves vertical and the second agent moves hozirontal.
                            Float yx_lhs = 0.0;
                            for (const auto& variable : master.agent_variables(a1))
                            {
                                const auto val = master.variable_primal_sol(variable);
                                const auto& path = variable.path();
                                Size num_crossings = 0;
                                for (Time i = 0; i <= x_length; ++i)
                                {
                                    const Edge e{y_first_entry.n + i * x_n_increment, y_d};
                                    const auto t = y_first_entry.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                for (Time i = 0; i <= x_length; ++i)
                                {
                                    const Edge e{y_first_exit.n + i * x_n_increment, y_d};
                                    const auto t = y_first_exit.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                yx_lhs += (num_crossings == 2) * val;

                                // if (num_crossings == 2)
                                {
                                    debugln("XY, agent {:3d}, val {:8.4f}, crossings {}, path {}",
                                            a1, val, num_crossings, format_path_with_time_spaced(path, map));
                                }
                            }
                            for (const auto& variable : master.agent_variables(a2))
                            {
                                const auto val = master.variable_primal_sol(variable);
                                const auto& path = variable.path();
                                Size num_crossings = 0;
                                for (Time i = 0; i <= y_length; ++i)
                                {
                                    const Edge e{x_first_entry.n + i * y_n_increment, x_d};
                                    const auto t = x_first_entry.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                for (Time i = 0; i <= y_length; ++i)
                                {
                                    const Edge e{x_first_exit.n + i * y_n_increment, x_d};
                                    const auto t = x_first_exit.t + i;
                                    num_crossings += (0 <= t && t < path.size() - 1 && path[t] == e);
                                }
                                yx_lhs += (num_crossings == 2) * val;

                                // if (num_crossings == 2)
                                {
                                    debugln("YX, agent {:3d}, val {:8.4f}, crossings {}, path {}",
                                            a2, val, num_crossings, format_path_with_time_spaced(path, map));
                                }
                            }

                            // Store a cut if violated.
                            if (is_gt(xy_lhs, 1.0 + CUT_VIOLATION))
                            {
                                ZoneScopedNC("Create candidate", TRACY_COLOUR);

                                debugln("Agent {} moving horizontal, agent {} moving vertical", a1, a2);

                                candidates_.push_back({xy_lhs,
                                                       rect_exit.t - rect_start.t,
                                                       a1, a2,
                                                       x_first_entry, x_first_exit, y_length, y_n_increment,
                                                       y_first_entry, y_first_exit, x_length, x_n_increment});
                            }
                            else if (is_gt(yx_lhs, 1.0 + CUT_VIOLATION))
                            {
                                ZoneScopedNC("Create candidate", TRACY_COLOUR);

                                debugln("Agent {} moving vertical, agent {} moving horizontal", a1, a2);

                                candidates_.push_back({yx_lhs,
                                                       rect_exit.t - rect_start.t,
                                                       a1, a2,
                                                       y_first_entry, y_first_exit, x_length, x_n_increment,
                                                       x_first_entry, x_first_exit, y_length, y_n_increment});
                            }
                        }
                }
            }

            // Continue exploring the MDD.
            for (Size d = 0; d < 5; ++d)
                if (a1_conflict->next[d])
                {
                    mdd_queue.push_back(a1_conflict->next[d]);
                }
        }
    }

    // Create the most violated cuts.
    Size num_separated_this_run = 0;
    num_separated_.set(0);
    std::sort(
        candidates_.begin(),
        candidates_.end(),
        [](const auto& a, const auto& b)
        {
            return std::tie(a.lhs, b.a1_first_entry.t, a.area) >
                   std::tie(b.lhs, a.a1_first_entry.t, b.area);
        }
    );
    for (auto& candidate : candidates_)
    {
        auto& [lhs, area, a1, a2, a1_first_entry, a1_first_exit, a1_length, a1_n_increment,
            a2_first_entry, a2_first_exit, a2_length, a2_n_increment] = candidate;
        auto& num_separated = num_separated_(a1, a2);
        if (num_separated == 0)
        {
            // Print.
#ifdef PRINT_DEBUG
            {
                const auto earliest_corner = a1_first_entry.nt();
                const NodeTime latest_corner{earliest_corner.n + a1_length * a1_n_increment + a2_length * a2_n_increment,
                                            earliest_corner.t + a1_length + a2_length};
                debugln("    Creating rectangle clique cut for agents {} going {} and {} going {} from corners {} "
                        "and {} with LHS {}",
                        a1, Direction{a1_first_entry.d}, a2, Direction{a2_first_entry.d},
                        format_nodetime(earliest_corner, map),
                        format_nodetime(latest_corner, map),
                        lhs);
            }
#endif

            // Create the row.
            create_row(a1, a2,
                       a1_first_entry, a1_first_exit, a1_length, a1_n_increment,
                       a2_first_entry, a2_first_exit, a2_length, a2_n_increment);
            ++num_separated;

            // Exit if enough cuts are found.
            ++num_separated_this_run;
            if (num_separated_this_run >= MAX_CUTS_PER_RUN)
            {
                break;
            }
        }
    }
}

void RectangleCliqueConflictSeparator::create_row(const Agent a1,
                                                  const Agent a2,
                                                  EdgeTime a1_first_entry,
                                                  EdgeTime a1_first_exit,
                                                  Time a1_length,
                                                  const Node a1_n_increment,
                                                  EdgeTime a2_first_entry,
                                                  EdgeTime a2_first_exit,
                                                  Time a2_length,
                                                  const Node a2_n_increment)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Create the name.
    const auto earliest_corner = a1_first_entry.nt();
    const NodeTime latest_corner{earliest_corner.n + a1_length * a1_n_increment + a2_length * a2_n_increment,
                                 earliest_corner.t + a1_length + a2_length};
    auto name = fmt::format("rectangle({},{},{},{},{},{})",
                            a1, Direction{a1_first_entry.d},
                            a2, Direction{a2_first_entry.d},
                            format_nodetime(earliest_corner, map),
                            format_nodetime(latest_corner, map));

    // Shift the first entry edgetime to non-negative times.
    while (a1_first_entry.t < 0)
    {
        ++a1_first_entry.t;
        a1_first_entry.n += a1_n_increment;

        ++a1_first_exit.t;
        a1_first_exit.n += a1_n_increment;

        --a1_length;
    }
    while (a2_first_entry.t < 0)
    {
        ++a2_first_entry.t;
        a2_first_entry.n += a2_n_increment;

        ++a2_first_exit.t;
        a2_first_exit.n += a2_n_increment;

        --a2_length;
    }
    debug_assert(a1_first_entry != a1_first_exit);
    debug_assert(a2_first_entry != a2_first_exit);

    // Create the row.
    const auto object_size = sizeof(RectangleCliqueConstraint);
    const auto hash_size = sizeof(Agent) * 2 + sizeof(EdgeTime) * 4 + sizeof(Time) * 2 + sizeof(Node) * 2;
    auto constraint = Constraint::construct<RectangleCliqueConstraint>(object_size,
                                                                       hash_size,
                                                                       ConstraintFamily::RectangleClique,
                                                                       this,
                                                                       std::move(name),
                                                                       2,
                                                                       '<',
                                                                       1.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->a1) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    constraint->a1 = a1;
    constraint->a2 = a2;
    constraint->a1_first_entry = a1_first_entry;
    constraint->a1_first_exit = a1_first_exit;
    constraint->a1_length = a1_length;
    constraint->a1_n_increment = a1_n_increment;
    constraint->a2_first_entry = a2_first_entry;
    constraint->a2_first_exit = a2_first_exit;
    constraint->a2_length = a2_length;
    constraint->a2_n_increment = a2_n_increment;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void RectangleCliqueConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& rectangle_constraint = *static_cast<const RectangleCliqueConstraint*>(&constraint);
    const auto a1 = rectangle_constraint.a1;
    const auto a2 = rectangle_constraint.a2;
    const auto a1_first_entry = rectangle_constraint.a1_first_entry;
    const auto a1_first_exit = rectangle_constraint.a1_first_exit;
    const auto a1_length = rectangle_constraint.a1_length;
    const auto a1_n_increment = rectangle_constraint.a1_n_increment;
    const auto a2_first_entry = rectangle_constraint.a2_first_entry;
    const auto a2_first_exit = rectangle_constraint.a2_first_exit;
    const auto a2_length = rectangle_constraint.a2_length;
    const auto a2_n_increment = rectangle_constraint.a2_n_increment;

    // Add the dual solution to the reduced cost function.
    auto& pricer = problem_.pricer();
    pricer.add_rectangle_penalty_one_agent(a1, a1_first_entry, a1_first_exit, a1_length, a1_n_increment, -dual);
    pricer.add_rectangle_penalty_one_agent(a2, a2_first_entry, a2_first_exit, a2_length, a2_n_increment, -dual);
}

Float RectangleCliqueConflictSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& rectangle_constraint = *static_cast<const RectangleCliqueConstraint*>(&constraint);
    const auto a1 = rectangle_constraint.a1;
    const auto a2 = rectangle_constraint.a2;

    // Calculate coefficient.
    if (a == a1)
    {
        const auto a1_first_entry = rectangle_constraint.a1_first_entry;
        const auto a1_first_exit = rectangle_constraint.a1_first_exit;
        const auto a1_length = rectangle_constraint.a1_length;
        const auto a1_n_increment = rectangle_constraint.a1_n_increment;
        return calculate_coeff(a1_first_entry, a1_length, a1_n_increment, path) &&
               calculate_coeff(a1_first_exit, a1_length, a1_n_increment, path);
    }
    else if (a == a2)
    {
        const auto a2_first_entry = rectangle_constraint.a2_first_entry;
        const auto a2_first_exit = rectangle_constraint.a2_first_exit;
        const auto a2_length = rectangle_constraint.a2_length;
        const auto a2_n_increment = rectangle_constraint.a2_n_increment;
        return calculate_coeff(a2_first_entry, a2_length, a2_n_increment, path) &&
               calculate_coeff(a2_first_exit, a2_length, a2_n_increment, path);
    }
    else
    {
        return 0.0;
    }
}

Bool RectangleCliqueConflictSeparator::calculate_coeff(const EdgeTime first_et,
                                                       const Time length,
                                                       const Node n_increment,
                                                       const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    const auto d = first_et.d;
    for (Time i = 0; i <= length; ++i)
    {
        const auto n = first_et.n + i * n_increment;
        const auto t = first_et.t + i;
        const Edge e{n, d};
        if (0 <= t && t < path.size() - 1 && path[t] == e)
        {
            return true;
        }
    }
    return false;
}
