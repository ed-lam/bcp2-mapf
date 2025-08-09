/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "bbtree/nodetime_brancher.h"
#include "constraints/rectangle_clique_conflict.h"
#include "constraints/rectangle_knapsack_conflict.h"
#include "master/master.h"
#include "problem/debug.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/hash_map.h"
#include "types/tracy.h"
#include <numeric>

#define TRACY_COLOUR tracy::Color::ColorType::DarkSeaGreen

struct Score
{
    Agent a;
    Float val;
    Time shortest_path_length;
};

struct SuccessorDirection
{
    Bool has_move{false};
    Bool has_wait{false};
};

Decisions NodeTimeBrancher::branch()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();
    auto& master = problem_.master();

    // Calculate branching candidates.
    Vector<Time> path_lengths(A, -1);
    Vector<Size> num_paths(A);
    HashMap<NodeTime, Pair<Vector<Score>, Vector<Score>>> candidates;
    HashMap<AgentTime, SuccessorDirection> successor_directions;
    for (Agent a = 0; a < A; ++a)
    {
        for (const auto& variable : master.agent_variables(a))
        {
            const auto val = master.variable_primal_sol(variable);
            if (!is_integral(val))
            {
                // Store the path length.
                const auto& path = variable.path();
                const Time path_length = path.size();
                debug_assert(path_lengths[a] < 0 || path_lengths[a] == path_length);
                path_lengths[a] = path_length;

                // Increment the number of paths used by the agent.
                ++num_paths[a];

                // Calculate a score for each nodetime.
                for (Time t = 1; t < path_length; ++t)
                {
                    // Store the candidate nodetime.
                    const NodeTime nt{path[t].n, t};
                    auto& scores = candidates[nt].first;
                    auto it = std::find_if(scores.begin(),
                                           scores.end(),
                                           [a](const Score& score){ return score.a == a; });
                    if (it == scores.end())
                    {
                        scores.push_back(Score{a, val, path_length});
                    }
                    else
                    {
                        auto& score = *it;
                        score.val += val;
                        score.shortest_path_length = std::min(score.shortest_path_length, path_length);
                    }

                    // Store whether the agent is waiting before this time.
                    auto& prev = successor_directions[{a, t - 1}];
                    prev.has_move |= (path[t - 1].d != Direction::WAIT);
                    prev.has_wait |= (path[t - 1].d == Direction::WAIT);
                }
            }
        }
    }
    release_assert(!candidates.empty(), "No candidates for nodetime branching");

    // Get the solution.
    const auto& projection = problem_.projection();

    // Determine the timesteps when an agent is within a rectangle. This is crude because the time interval encompasses
    // the earlest and latest rectangle.
    Vector<Pair<Time, Time>> rectangle_times(A, {TIME_MAX, 0});
#ifdef USE_RECTANGLE_CLIQUE_CUTS
    for (const auto& constraint : master.subset_constraints())
    {
        const auto& name = constraint.name();
        const auto slack = master.constraint_slack(constraint);
        if (is_eq(slack, 0.0) && name.substr(0, 9) == "rectangle")
        {
            // Get the constraint data.
            const auto& rectangle_constraint =
                *static_cast<const RectangleCliqueConflictSeparator::RectangleCliqueConstraint*>(&constraint);
            const auto a1 = rectangle_constraint.a1;
            const auto a2 = rectangle_constraint.a2;

            // Get the earliest and latest time that an agent enters and exits a rectangle.
            {
                const auto first_entry = rectangle_constraint.a1_first_entry;
                const auto first_exit = rectangle_constraint.a1_first_exit;
                const auto length = rectangle_constraint.a1_length;
                const auto n_increment = rectangle_constraint.a1_n_increment;
                const auto d = first_entry.d;
                for (Time i = 0; i <= length; ++i)
                {
                    const auto n = first_entry.n + i * n_increment;
                    const auto t = first_entry.t + i;
                    const EdgeTime et{n, d, t};
                    if (is_gt(projection.find_agent_move_edgetime(a1, et), 0.0))
                    {
                        rectangle_times[a1].first = std::min(rectangle_times[a1].first, et.t);
                        rectangle_times[a1].second = std::max(rectangle_times[a1].second, et.t);
                        goto FOUND_A1_RECTANGLE;
                    }
                }
                for (Time i = 0; i <= length; ++i)
                {
                    const auto n = first_exit.n + i * n_increment;
                    const auto t = first_exit.t + i;
                    const EdgeTime et{n, d, t};
                    if (is_gt(projection.find_agent_move_edgetime(a1, et), 0.0))
                    {
                        rectangle_times[a1].first = std::min(rectangle_times[a1].first, et.t);
                        rectangle_times[a1].second = std::max(rectangle_times[a1].second, et.t);
                        goto FOUND_A1_RECTANGLE;
                    }
                }
                FOUND_A1_RECTANGLE:;
            }
            {
                const auto first_entry = rectangle_constraint.a2_first_entry;
                const auto first_exit = rectangle_constraint.a2_first_exit;
                const auto length = rectangle_constraint.a2_length;
                const auto n_increment = rectangle_constraint.a2_n_increment;
                const auto d = first_entry.d;
                for (Time i = 0; i <= length; ++i)
                {
                    const auto n = first_entry.n + i * n_increment;
                    const auto t = first_entry.t + i;
                    const EdgeTime et{n, d, t};
                    if (is_gt(projection.find_agent_move_edgetime(a2, et), 0.0))
                    {
                        rectangle_times[a2].first = std::min(rectangle_times[a2].first, et.t);
                        rectangle_times[a2].second = std::max(rectangle_times[a2].second, et.t);
                        goto FOUND_A2_RECTANGLE;
                    }
                }
                for (Time i = 0; i <= length; ++i)
                {
                    const auto n = first_exit.n + i * n_increment;
                    const auto t = first_exit.t + i;
                    const EdgeTime et{n, d, t};
                    if (is_gt(projection.find_agent_move_edgetime(a2, et), 0.0))
                    {
                        rectangle_times[a2].first = std::min(rectangle_times[a2].first, et.t);
                        rectangle_times[a2].second = std::max(rectangle_times[a2].second, et.t);
                        goto FOUND_A2_RECTANGLE;
                    }
                }
                FOUND_A2_RECTANGLE:;
            }
        }
    }
#else
    for (const auto& constraint : master.subset_constraints())
    {
        const auto& name = constraint.name();
        const auto slack = master.constraint_slack(constraint);
        if (is_eq(slack, 0.0) && name.substr(0, 9) == "rectangle")
        {
            // Get the constraint data.
            const auto& rectangle_constraint =
                *static_cast<const RectangleKnapsackConflictSeparator::RectangleKnapsackConstraint*>(&constraint);
            const auto a1 = rectangle_constraint.a1;
            const auto a2 = rectangle_constraint.a2;

            // Get the earliest and latest time that an agent enters and exits a rectangle.
            for (const auto& et : rectangle_constraint.a1_ets())
                if (is_gt(projection.find_agent_move_edgetime(a1, et), 0.0))
                {
                    rectangle_times[a1].first = std::min(rectangle_times[a1].first, et.t);
                    rectangle_times[a1].second = std::max(rectangle_times[a1].second, et.t);
                    goto FOUND_A1_RECTANGLE;
                }
            FOUND_A1_RECTANGLE:;
            for (const auto& et : rectangle_constraint.a2_ets())
                if (is_gt(projection.find_agent_move_edgetime(a2, et), 0.0))
                {
                    rectangle_times[a2].first = std::min(rectangle_times[a2].first, et.t);
                    rectangle_times[a2].second = std::max(rectangle_times[a2].second, et.t);
                    goto FOUND_A2_RECTANGLE;
                }
            FOUND_A2_RECTANGLE:;
        }
    }
#endif

    // Categorise nodetimes with integral and fractional values.
    for (auto it = candidates.begin(); it != candidates.end();)
    {
        // Sum the nodetime usage.
        auto& scores = it->second;
        auto& integral_scores = scores.first;
        auto& fractional_scores = scores.second;
        const auto nt_val = std::accumulate(integral_scores.begin(),
                                            integral_scores.end(),
                                            0.0,
                                            [](const Float val, const Score& score) { return val + score.val; });

        // Move to list of fractional vertices.
        debug_assert(integral_scores.size() >= 1);
        if (!is_integral(nt_val))
        {
            // Fractional.
            fractional_scores = std::move(integral_scores);
            ++it;
        }
        else if (integral_scores.size() == 1)
        {
            // Integral and used by one agent.
            it = candidates.erase(it);
        }
        else
        {
            // Integral and used by more than one agent.
            ++it;
        }
    }

    // Prepare decision.
    Decisions output = {std::make_unique<NodeTimeBrancherData>(),
                        std::make_unique<NodeTimeBrancherData>()};
    auto& left_decision = *static_cast<NodeTimeBrancherData*>(output.first.get());
    auto& right_decision = *static_cast<NodeTimeBrancherData*>(output.second.get());
    Bool best_in_rectangle = false;
    Bool best_move_and_wait = false;
    Time best_shortest_path_length = TIME_MAX;
    Size best_num_paths = 0;
    NodeTime best_nt{0, TIME_MAX};
    Float best_val = 0.0;
    Agent best_a = -1;

    // Prefer a nodetime that allows an agent to reach its target the earliest.
    for (const auto& [nt, scores] : candidates)
        for (const auto& [a, val, shortest_path_length] : scores.first)
        {
            const auto in_rectangle = (rectangle_times[a].first <= nt.t && nt.t <= rectangle_times[a].second);
            const auto successor_directions_t = successor_directions.at({a, nt.t - 1});
            const auto move_and_wait = (successor_directions_t.has_move && successor_directions_t.has_wait);
            if (std::tie(best_in_rectangle, best_move_and_wait, shortest_path_length,      best_num_paths, nt.t,      best_val) <
                std::tie(in_rectangle,      move_and_wait,      best_shortest_path_length, num_paths[a],   best_nt.t, val))
            {
                best_in_rectangle = in_rectangle;
                best_move_and_wait = move_and_wait;
                best_shortest_path_length = shortest_path_length;
                best_num_paths = num_paths[a];
                best_nt = nt;
                best_val = val;
                best_a = a;
            }
        }
    if (best_a >= 0)
    {
        debugln("Nodetime brancher branching on agent {} and integer nodetime {} in B&B node {}",
                best_a,
                format_nodetime(best_nt, instance_.map),
                problem_.bbtree().current_id());
        goto BRANCH;
    }

    // No integer nodetime was found so find a fractional nodetime, arising from edgetime conflicts.
    for (const auto& [nt, scores] : candidates)
        for (const auto& [a, val, shortest_path_length] : scores.second)
        {
            const auto in_rectangle = (rectangle_times[a].first <= nt.t && nt.t <= rectangle_times[a].second);
            const auto successor_directions_t = successor_directions.at({a, nt.t - 1});
            const auto move_and_wait = (successor_directions_t.has_move && successor_directions_t.has_wait);
            if (std::tie(best_in_rectangle, best_move_and_wait, shortest_path_length,      best_num_paths, nt.t,      best_val) <
                std::tie(in_rectangle,      move_and_wait,      best_shortest_path_length, num_paths[a],   best_nt.t, val))
            {
                best_in_rectangle = in_rectangle;
                best_move_and_wait = move_and_wait;
                best_shortest_path_length = shortest_path_length;
                best_num_paths = num_paths[a];
                best_nt = nt;
                best_val = val;
                best_a = a;
            }
        }
    if (best_a >= 0)
    {
        debugln("Nodetime brancher branching on agent {} and fractional nodetime {} in B&B node {}",
                best_a,
                format_nodetime(best_nt, instance_.map),
                problem_.bbtree().current_id());
        goto BRANCH;
    }

    // Unreachable.
    err("Failed to find a nodetime branching decision");

    // Store decision.
    BRANCH:
    left_decision.a = best_a;
    left_decision.nt = best_nt;
    left_decision.dir = BranchDirection::Down;
    right_decision.a = best_a;
    right_decision.nt = best_nt;
    right_decision.dir = BranchDirection::Up;
    num_added_ += 2;
    return output;
}

void NodeTimeBrancher::add_pricing_costs(const BrancherData* const data)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [decision_a, nt, dir] = *static_cast<const NodeTimeBrancherData*>(data);

    // Impose the branching decision.
    auto& pricer = problem_.pricer();
    if (dir == BranchDirection::Down)
    {
        pricer.add_nodetime_penalty_one_agent(decision_a, nt, INF);
    }
    else
    {
        // Add a waypoint for the agent.
        pricer.add_waypoint(decision_a, nt);

        // Disable all other agents from visiting the nodetime.
        pricer.add_nodetime_penalty_all_except_one_agent(decision_a, nt, INF);
    }
}

void NodeTimeBrancher::disable_vars(const BrancherData* const data)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [decision_a, nt, dir] = *static_cast<const NodeTimeBrancherData*>(data);

    // Disable incompatible paths.
    debugln("Nodetime brancher disabling paths incompatible with decision on agent {} {} {} in B&B node {}:",
            decision_a,
            (dir == BranchDirection::Up ? "requiring" : "forbidding"),
            format_nodetime(nt, instance_.map),
            problem_.bbtree().current_id());
    for (const auto& variable : master.agent_variables(decision_a))
    {
        // Disable a path of the same agent if it doesn't satisfy the nodetime branching decision.
        const auto& path = variable.path();
        const auto t = std::min<Time>(nt.t, path.size() - 1);
        const auto path_visits_nt = (path[t].n == nt.n);
        if (path_visits_nt != static_cast<Bool>(dir))
        {
            debugln("Agent {}, path {}", decision_a, format_path_with_time_spaced(path, instance_.map));
            master.disable_variable(variable);
        }
    }
    if (dir == BranchDirection::Up)
    {
        for (Agent a = 0; a < A; ++a)
            if (a != decision_a)
            {
                for (const auto& variable : master.agent_variables(a))
                {
                    // Disable a path of a different agent if the branch direction requires the nodetime and
                    // the other agent uses the nodetime.
                    const auto& path = variable.path();
                    const auto t = std::min<Time>(nt.t, path.size() - 1);
                    const auto path_visits_nt = (path[t].n == nt.n);
                    if (path_visits_nt)
                    {
                        debugln("Agent {}, path {}", a, format_path_with_time_spaced(path, instance_.map));
                        master.disable_variable(variable);
                    }
                }
            }
    }
    debugln("");
}

void NodeTimeBrancher::print(const BrancherData* const data) const
{
    // Get the constraint data.
    const auto& [a, nt, dir] = *static_cast<const NodeTimeBrancherData*>(data);

    // Print.
    println("    Agent {} {} {}",
            a,
            (dir == BranchDirection::Up ? "requiring" : "forbidding"),
            format_nodetime(nt, instance_.map));
}
