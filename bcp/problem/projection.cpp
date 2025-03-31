/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "problem/problem.h"
#include "problem/projection.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::SlateGray

#define PROJECT_FRACTIONAL_ONLY

Projection::Projection(const Instance& instance, Problem& problem) :
    instance_(instance),
    master_(problem.master()),
    clock_(),

    agent_nodetimes_(instance_.agents.size()),
    agent_edgetimes_(instance_.agents.size()),

    summed_nodetimes_(),
    summed_undirected_edgetimes_(),

    agent_node_finish_(),
    node_finish_(),

    memory_pool_(),
    zeros_(nullptr),
    list_fractional_nodetimes_(),
    list_fractional_edgetimes_(),
    list_node_finish_()
{
    ZoneScopedC(TRACY_COLOUR);

    // Initialise hash tables.
    const Agent A = instance_.agents.size();
    for (Agent a = 0; a < A; ++a)
    {
        const auto n = instance_.agents[a].target;
        agent_node_finish_[{a, n}];
    }
    for (Agent a = 0; a < A; ++a)
    {
        const auto n = instance_.agents[a].target;
        auto& [agent, values] = node_finish_[n];
        agent = a;
        values = &agent_node_finish_.at({a, n});

        list_node_finish_[n];
    }
}

void Projection::update()
{
    ZoneScopedC(TRACY_COLOUR);

    // Clear and compute compute if feasible.
    auto timer = clock_.start_timer();
    clear();
    if (master_.status() != MasterProblemStatus::Infeasible)
    {
        compute();
    }
}

void Projection::clear()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const Agent A = instance_.agents.size();

    // Clear data.
    for (Agent a = 0; a < A; ++a)
    {
        agent_nodetimes_[a].clear();
        agent_edgetimes_[a].clear();
    }
    summed_nodetimes_.clear();
    summed_undirected_edgetimes_.clear();
    for (auto& [_, finish_sums] : agent_node_finish_)
    {
        finish_sums.resize(1);
        debug_assert(finish_sums[0] == 0.0);
    }
    memory_pool_.reset(sizeof(Float) * (A + 1));
    zeros_ = static_cast<ProjectionValues*>(memory_pool_.get_buffer<true, true>());
    list_fractional_nodetimes_.clear();
    list_fractional_edgetimes_.clear();
    // No need to clear list_node_finish_ because it is overwritten.
}

void Projection::compute()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const Agent A = instance_.agents.size();
    const auto& map = instance_.map;

    {
        ZoneScopedNC("Compute projection A", TRACY_COLOUR);

        // Calculate solution on the nodetimes and edgetimes from the paths.
        for (Agent a = 0; a < A; ++a)
        {
            // Sum values from every path.
            for (const auto& variable : master_.agent_variables(a))
            {
                const auto val = master_.variable_primal_sol(variable);
                if (is_gt(val, 0.0))
                {
                    const auto& path = variable.path();

                    Time t = 0;
                    for (; t < path.size() - 1; ++t)
                    {
                        const NodeTime nt{path[t].n, t};
                        const EdgeTime et{path[t], t};

                        agent_nodetimes_[a][nt] += val;
                        agent_edgetimes_[a][et] += val;
                    }
                    {
                        const NodeTime nt{path[t].n, t};
                        const EdgeTime et{path[t].n, Direction::WAIT, t};

                        agent_nodetimes_[a][nt] += val;
                        agent_edgetimes_[a][et] += val;

                        ++t;
                        auto& finish_times = agent_node_finish_[{a, nt.n}];
                        finish_times.resize(std::max<Size>(finish_times.size(), t + 1));
                        finish_times[t] += val;
                    }
                }
            }

            // Sum values across all agents.
            for (const auto& [nt, val] : agent_nodetimes_[a])
            {
                summed_nodetimes_[nt] += val;
            }
            for (const auto& [et, val] : agent_edgetimes_[a])
                if (et.d != Direction::WAIT)
                {
                    const EdgeTime undirected_et{map.get_undirected_edge(et.et.e), et.t};
                    summed_undirected_edgetimes_[undirected_et] += val;
                }
        }
    }

    // Check projection against paths without waiting at the target.
#ifdef DEBUG
    for (const auto& [nt, val] : summed_nodetimes_)
    {
        Float check_val = 0.0;
        for (Agent a = 0; a < A; ++a)
            for (const auto& variable : master_.agent_variables(a))
            {
                const auto& path = variable.path();
                const auto var_val = master_.variable_primal_sol(variable);
                check_val += (nt.t < path.size() && path[nt.t].n == nt.n) * var_val;
            }
        debug_assert(is_eq(check_val, val));
    }
#endif

    {
        ZoneScopedNC("Compute projection B", TRACY_COLOUR);

        // Copy the values into lists.
        for (Agent a = 0; a < A; ++a)
            for (const auto& [nt, val] : agent_nodetimes_[a])
            {
                debug_assert(is_gt(val, 0.0));
#ifdef PROJECT_FRACTIONAL_ONLY
                if (is_lt(val, 1.0))
#endif
                {
                    auto [it, created] = list_fractional_nodetimes_.try_emplace(nt);
                    auto& agent_vals = it->second;
                    if (created)
                    {
                        debug_assert(!agent_vals);
                        agent_vals = static_cast<ProjectionValues*>(memory_pool_.get_buffer<true, true>());
                    }
                    agent_vals->total += val;
                    agent_vals->agent[a] += val;
                }
            }
        for (Agent a = 0; a < A; ++a)
            for (const auto& [et, val] : agent_edgetimes_[a])
            {
                debug_assert(is_gt(val, 0.0));
#ifdef PROJECT_FRACTIONAL_ONLY
                if (is_lt(val, 1.0))
#endif
                {
                    auto [it, created] = list_fractional_edgetimes_.try_emplace(et);
                    auto& agent_vals = it->second;
                    if (created)
                    {
                        debug_assert(!agent_vals);
                        agent_vals = static_cast<ProjectionValues*>(memory_pool_.get_buffer<true, true>());
                    }
                    agent_vals->total += val;
                    agent_vals->agent[a] += val;
                }
            }
    }
    {
        ZoneScopedNC("Compute projection C", TRACY_COLOUR);

        // Sum finish time values.
        for (auto& [an, finish_times] : agent_node_finish_)
        {
            // Sum the finish time values.
            debug_assert(finish_times.size() >= 2);
            for (Time t = 1; t < finish_times.size(); ++t)
            {
                finish_times[t] += finish_times[t - 1];
            }

            // Create projection value objects for quick look-up.
            const auto a = an.a;
            const auto n = an.n;
            auto& finish_time_lists = list_node_finish_.at(n);
            finish_time_lists.resize(finish_times.size());
            for (Time t = 0; t < finish_time_lists.size(); ++t)
            {
                auto& finish_time_list = finish_time_lists[t];
                if (t == 0 && finish_times[t] == 0)
                {
                    finish_time_list = zeros_;
                }
                else if (t == 0 || finish_times[t] != finish_times[t - 1])
                {
                    finish_time_list = static_cast<ProjectionValues*>(memory_pool_.get_buffer<true, true>());
                    finish_time_list->total = finish_times[t];
                    finish_time_list->agent[a] = finish_times[t];
                }
                else
                {
                    finish_time_list = finish_time_lists[t - 1];
                }
            }
        }
    }
    {
        ZoneScopedNC("Compute projection D", TRACY_COLOUR);

        // Include the finish time values in the projections.
        for (Agent a = 0; a < A; ++a)
        {
            for (auto& [nt, val] : agent_nodetimes_[a])
                if (const auto it = agent_node_finish_.find({a, nt.n}); it != agent_node_finish_.end())
                {
                    const auto& finish_times = it->second;
                    debug_assert(finish_times.size() >= 1);
                    const auto t = std::min<Time>(finish_times.size() - 1, nt.t);
                    val += finish_times[t];
                }
            for (auto& [et, val] : agent_edgetimes_[a])
                if (et.d == Direction::WAIT)
                    if (const auto it = agent_node_finish_.find({a, et.n}); it != agent_node_finish_.end())
                    {
                        const auto& finish_times = it->second;
                        debug_assert(finish_times.size() >= 1);
                        const auto t = std::min<Time>(finish_times.size() - 1, et.t);
                        val += finish_times[t];
                    }
        }
        for (auto& [nt, val] : summed_nodetimes_)
            if (const auto it = node_finish_.find(nt.n); it != node_finish_.end())
            {
                const auto& finish_times = *it->second.second;
                debug_assert(finish_times.size() >= 1);
                const auto t = std::min<Time>(finish_times.size() - 1, nt.t);
                val += finish_times[t];
            }
        // for (auto& [et, val] : summed_undirected_edgetimes_)
        //     if (et.d == Direction::WAIT)
        //         if (const auto it = node_finish_.find(et.n); it != node_finish_.end())
        //         {
        //             const auto& finish_times = *it->second.second;
        //             debug_assert(finish_times.size() >= 1);
        //             const auto t = std::min<Time>(finish_times.size() - 1, et.t);
        //             val += finish_times[t];
        //         }
        for (auto& [nt, vals_ptr] : list_fractional_nodetimes_)
            if (const auto it = node_finish_.find(nt.n); it != node_finish_.end())
            {
                const auto a = it->second.first;
                const auto& finish_times = *it->second.second;
                debug_assert(finish_times.size() >= 1);
                const auto t = std::min<Time>(finish_times.size() - 1, nt.t);

                auto& vals = *vals_ptr;
                vals.total += finish_times[t];
                vals.agent[a] += finish_times[t];
            }
        for (auto& [et, vals_ptr] : list_fractional_edgetimes_)
            if (et.d == Direction::WAIT)
                if (const auto it = node_finish_.find(et.n); it != node_finish_.end())
                {
                    const auto a = it->second.first;
                    const auto& finish_times = *it->second.second;
                    debug_assert(finish_times.size() >= 1);
                    const auto t = std::min<Time>(finish_times.size() - 1, et.t);

                    auto& vals = *vals_ptr;
                    vals.total += finish_times[t];
                    vals.agent[a] += finish_times[t];
                }
    }

    // Check projection against paths with waiting at the target.
#ifdef DEBUG
    for (const auto& [nt, val] : summed_nodetimes_)
    {
        Float check_val = 0.0;
        for (Agent a = 0; a < A; ++a)
            for (const auto& variable : master_.agent_variables(a))
            {
                const auto& path = variable.path();
                const auto var_val = master_.variable_primal_sol(variable);
                const auto t = std::min<Time>(path.size() - 1, nt.t);
                check_val += (path[t].n == nt.n) * var_val;
            }
        debug_assert(is_eq(val, check_val));
    }
#endif

    // Check the lists.
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
    for (const auto& [nt, vals] : list_fractional_nodetimes_)
    {
        Float check_total = 0.0;
        for (Agent a = 0; a < A; ++a)
        {
            Float check_val = 0.0;
            for (const auto& variable : master_.agent_variables(a))
            {
                const auto& path = variable.path();
                const auto var_val = master_.variable_primal_sol(variable);
                const auto t = std::min<Time>(path.size() - 1, nt.t);
                check_val += (path[t].n == nt.n) * var_val;
            }
            debug_assert(is_eq(check_val, find_agent_nodetime(a, nt)));
            debug_assert(is_eq(check_val, vals->agent[a]));
            check_total += check_val;
        }
        debug_assert(is_eq(check_total, vals->total));
    }
    for (const auto& [et, vals] : list_fractional_edgetimes_)
    {
        Float check_total = 0.0;
        for (Agent a = 0; a < A; ++a)
        {
            Float check_val = 0.0;
            for (const auto& variable : master_.agent_variables(a))
            {
                const auto& path = variable.path();
                const auto var_val = master_.variable_primal_sol(variable);
                const auto t = std::min<Time>(path.size() - 1, et.t);
                check_val += ((t <  path.size() - 1 && path[t] == et.et.e) ||
                              (t >= path.size() - 1 && path[t].n == et.n && et.d == Direction::WAIT)) * var_val;
            }
            debug_assert(is_eq(check_val, et.d != Direction::WAIT ?
                                            find_agent_move_edgetime(a, et) : find_agent_wait_edgetime(a, et)));
            debug_assert(is_eq(check_val, vals->agent[a]));
            check_total += check_val;
        }
        debug_assert(is_eq(check_total, vals->total));
    }
#endif
#endif
}

Float Projection::find_summed_nodetime(const NodeTime nt) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate value for verification.
#ifdef DEBUG
    Float check_val = 0.0;
    for (Agent a = 0; a < instance_.agents.size(); ++a)
        for (const auto& variable : master_.agent_variables(a))
        {
            const auto& path = variable.path();
            const auto var_val = master_.variable_primal_sol(variable);
            const auto t = std::min<Time>(path.size() - 1, nt.t);
            check_val += (path[t].n == nt.n) * var_val;
        }
#endif

    // Find the value from several places.
    if (auto it = summed_nodetimes_.find(nt); it != summed_nodetimes_.end())
    {
        debug_assert(is_eq(check_val, it->second));
        return it->second;
    }
    else if (const auto it = node_finish_.find(nt.n); it != node_finish_.end())
    {
        const auto& finish_times = *(it->second.second);
        debug_assert(finish_times.size() >= 1);
        const auto t = std::min<Time>(finish_times.size() - 1, nt.t);
        debug_assert(is_eq(check_val, finish_times[t]));
        return finish_times[t];
    }
    else
    {
        debug_assert(is_eq(check_val, 0.0));
        return 0.0;
    }
}

// Float Projection::find_summed_wait_edgetime(const EdgeTime et) const
// {
//     ZoneScopedC(TRACY_COLOUR);
//
//     // Calculate value for verification.
//     debug_assert(et.d == Direction::WAIT);
// #ifdef DEBUG
//     Float check_val = 0.0;
//     for (Agent a = 0; a < instance_.agents.size(); ++a)
//         for (const auto& variable : master_.agent_variables(a))
//         {
//             const auto& path = variable.path();
//             const auto var_val = master_.variable_primal_sol(variable);
//             const auto t = std::min<Time>(path.size() - 1, et.t);
//             check_val += ((t <  path.size() - 1 && path[t] == et.et.e) ||
//                           (t >= path.size() - 1 && path[t].n == et.n && et.d == Direction::WAIT)) * var_val;
//         }
// #endif
//
//     // Find the value from several places.
//     if (auto it = summed_undirected_edgetimes_.find(et); it != summed_undirected_edgetimes_.end())
//     {
//         debug_assert(is_eq(check_val, it->second));
//         return it->second;
//     }
//     else if (const auto it = node_finish_.find(et.n); it != node_finish_.end())
//     {
//         const auto& finish_times = *(it->second.second);
//         debug_assert(finish_times.size() >= 1);
//         const auto t = std::min<Time>(finish_times.size() - 1, et.t);
//         debug_assert(is_eq(check_val, finish_times[t]));
//         return finish_times[t];
//     }
//     else
//     {
//         debug_assert(is_eq(check_val, 0.0));
//         return 0.0;
//     }
// }

Float Projection::find_agent_nodetime(const Agent a, const NodeTime nt) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate value for verification.
#ifdef DEBUG
    Float check_val = 0.0;
    for (const auto& variable : master_.agent_variables(a))
    {
        const auto& path = variable.path();
        const auto var_val = master_.variable_primal_sol(variable);
        const auto t = std::min<Time>(path.size() - 1, nt.t);
        check_val += (path[t].n == nt.n) * var_val;
    }
#endif

    // Find the value from several places.
    if (auto it = agent_nodetimes_[a].find(nt); it != agent_nodetimes_[a].end())
    {
        debug_assert(is_eq(check_val, it->second));
        return it->second;
    }
    else if (const auto it = agent_node_finish_.find({a, nt.n}); it != agent_node_finish_.end())
    {
        const auto& finish_times = it->second;
        debug_assert(finish_times.size() >= 1);
        const auto t = std::min<Time>(finish_times.size() - 1, nt.t);
        debug_assert(is_eq(check_val, finish_times[t]));
        return finish_times[t];
    }
    else
    {
        debug_assert(is_eq(check_val, 0.0));
        return 0.0;
    }
}

Float Projection::find_agent_move_edgetime(const Agent a, const EdgeTime et) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate value for verification.
    debug_assert(et.d != Direction::WAIT);
#ifdef DEBUG
    Float check_val = 0.0;
    for (const auto& variable : master_.agent_variables(a))
    {
        const auto& path = variable.path();
        const auto var_val = master_.variable_primal_sol(variable);
        const auto t = std::min<Time>(path.size() - 1, et.t);
        check_val += (t < path.size() - 1 && path[t] == et.et.e) * var_val;
    }
#endif

    // Find the value from several places.
    if (auto it = agent_edgetimes_[a].find(et); it != agent_edgetimes_[a].end())
    {
        debug_assert(is_eq(check_val, it->second));
        return it->second;
    }
    else
    {
        debug_assert(is_eq(check_val, 0.0));
        return 0.0;
    }
}

Float Projection::find_agent_wait_edgetime(const Agent a, const EdgeTime et) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate value for verification.
    debug_assert(et.d == Direction::WAIT);
#ifdef DEBUG
    Float check_val = 0.0;
    for (const auto& variable : master_.agent_variables(a))
    {
        const auto& path = variable.path();
        const auto var_val = master_.variable_primal_sol(variable);
        const auto t = std::min<Time>(path.size() - 1, et.t);
        check_val += ((t <  path.size() - 1 && path[t] == et.et.e) ||
                      (t >= path.size() - 1 && path[t].n == et.n && et.d == Direction::WAIT)) * var_val;
    }
#endif

    // Find the value from several places.
    if (auto it = agent_edgetimes_[a].find(et); it != agent_edgetimes_[a].end())
    {
        debug_assert(is_eq(check_val, it->second));
        return it->second;
    }
    else if (const auto it = agent_node_finish_.find({a, et.n}); it != agent_node_finish_.end())
    {
        const auto& finish_vals = it->second;
        debug_assert(finish_vals.size() >= 1);
        const auto t = std::min<Time>(finish_vals.size() - 1, et.t);
        debug_assert(is_eq(check_val, finish_vals[t]));
        return finish_vals[t];
    }
    else
    {
        debug_assert(is_eq(check_val, 0.0));
        return 0.0;
    }
}

const ProjectionValues& Projection::find_list_fractional_nodetime(const NodeTime nt) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate values for verification.
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
    const Agent A = instance_.agents.size();
    Vector<Float> check_vals(A);
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : master_.agent_variables(a))
        {
            const auto& path = variable.path();
            const auto var_val = master_.variable_primal_sol(variable);
            const auto t = std::min<Time>(path.size() - 1, nt.t);
            check_vals[a] += (path[t].n == nt.n) * var_val;
        }
#endif
#endif

    // Find the value from several places.
    if (auto it = list_fractional_nodetimes_.find(nt); it != list_fractional_nodetimes_.end())
    {
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*it->second)[a])); }
#endif
#endif
        return *it->second;
    }
    else if (const auto it = list_node_finish_.find(nt.n); it != list_node_finish_.end())
    {
        const auto& finish_vals = it->second;
        const auto t = std::min<Time>(finish_vals.size() - 1, nt.t);
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*finish_vals[t])[a])); }
#endif
#endif
        return *finish_vals[t];
    }
    else
    {
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*zeros_)[a])); }
#endif
#endif
        return *zeros_;
    }
}

const ProjectionValues& Projection::find_list_fractional_move_edgetime(const EdgeTime et) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate values for verification.
    debug_assert(et.d != Direction::WAIT);
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
    const Agent A = instance_.agents.size();
    Vector<Float> check_vals(A);
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : master_.agent_variables(a))
        {
            const auto& path = variable.path();
            const auto var_val = master_.variable_primal_sol(variable);
            const auto t = std::min<Time>(path.size() - 1, et.t);
            check_vals[a] += (t < path.size() - 1 && path[t] == et.et.e) * var_val;
        }
#endif
#endif

    // Find the value from several places.
    if (auto it = list_fractional_edgetimes_.find(et); it != list_fractional_edgetimes_.end())
    {
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*it->second)[a])); }
#endif
#endif
        return *it->second;
    }
    else
    {
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*zeros_)[a])); }
#endif
#endif
        return *zeros_;
    }
}

const ProjectionValues& Projection::find_list_fractional_wait_edgetime(const EdgeTime et) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate values for verification.
    debug_assert(et.d == Direction::WAIT);
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
    const Agent A = instance_.agents.size();
    Vector<Float> check_vals(A);
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : master_.agent_variables(a))
        {
            const auto& path = variable.path();
            const auto var_val = master_.variable_primal_sol(variable);
            const auto t = std::min<Time>(path.size() - 1, et.t);
            check_vals[a] += ((t <  path.size() - 1 && path[t] == et.et.e) ||
                              (t >= path.size() - 1 && path[t].n == et.n && et.d == Direction::WAIT)) * var_val;
        }
#endif
#endif

    // Find the value from several places.
    if (auto it = list_fractional_edgetimes_.find(et); it != list_fractional_edgetimes_.end())
    {
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*it->second)[a])); }
#endif
#endif
        return *it->second;
    }
    else if (const auto it = list_node_finish_.find(et.n); it != list_node_finish_.end())
    {
        const auto& finish_vals = it->second;
        const auto t = std::min<Time>(finish_vals.size() - 1, et.t);
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*finish_vals[t])[a])); }
#endif
#endif
        return *finish_vals[t];
    }
    else
    {
#ifdef DEBUG
#ifndef PROJECT_FRACTIONAL_ONLY
        for (Agent a = 0; a < A; ++a) { debug_assert(is_eq(check_vals[a], (*zeros_)[a])); }
#endif
#endif
        return *zeros_;
    }
}
