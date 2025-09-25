/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "problem/problem.h"
#include "output/formatting.h"
#include "problem/runtime_tests.h"
#include "types/debug.h"
#include "types/tracy.h"
#include <fmt/ranges.h>

#define STALLED_NUM_ROUNDS (20)
#define STALLED_ABSOLUTE_CHANGE (-0.2)

#define PRINT_LOG_HEADER_LINES 20
#define PRINT_LOG_ROOT_ITERATIONS 1
#define PRINT_LOG_NODE_ITERATIONS 1

#define TRACY_COLOUR tracy::Color::ColorType::Silver

volatile std::sig_atomic_t raised_signal = 0;

void signal_handler(const int signal)
{
    raised_signal = signal;
}

Problem::Problem(const FilePath& scenario_path, const Agent agent_limit) :
    instance_(scenario_path, agent_limit),

    iter_(0),
    bbtree_(instance_, *this),
    master_(instance_, *this),
    master_obj_history_(STALLED_NUM_ROUNDS),
    pricer_(instance_, *this),
    initial_constraints_(instance_, *this),
    lazy_constraints_(instance_, *this),
    branchers_(instance_, *this),
    heuristics_(instance_, *this),

    ub_(COST_INF),
    sol_(),
    projection_(instance_, *this),

    lp_num_feasible_(0),
    lp_num_improving_(0),
    clock_(),
    num_log_lines_(0),
    next_log_iter_(0)
{
    ZoneScopedC(TRACY_COLOUR);

    // Perform run-time tests.
    test_flexible_array_pointers();

    // Create initial constraints.
    initial_constraints_.apply([](auto&... separator) { return (separator.run(), ...); });
}

Cost Problem::lb() const
{
    ZoneScopedC(TRACY_COLOUR);

    return bbtree_.empty() ? ub() : bbtree_.lb();
}

Cost Problem::ub() const
{
    ZoneScopedC(TRACY_COLOUR);

    return ub_;
}

Real64 Problem::gap() const
{
    ZoneScopedC(TRACY_COLOUR);

    const auto ub_val = ub();
    const auto lb_val = lb();
    return (ub_val - lb_val) / ub_val;
}

void Problem::solve(const Real64 time_limit)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();

    // Solve.
    try
    {
        // Set-up signal handlers to exit gracefully.
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);

        // Start timer.
        auto timer = clock_.start_timer(time_limit);

        // Branch-and-bound node loop.
        while (bbtree_.next())
        {
            ZoneScopedNC("Node loop", TRACY_COLOUR);

            // Skip the current node if its lower bound is higher than the upper bound.
            if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
            {
                DEBUGLN("Discarding BB tree node {}, lb {}",
                        bbtree_.current_id(),
                        bbtree_.current_lb());
                if constexpr (BBTree::is_best_first_search())
                {
                    print_node_log();
                    bbtree_.clear();
                    break;
                }
                else
                {
                    goto NODE_COMPLETED;
                }
            }

            // Print statistics.
            DEBUGLN(
                "BB tree has {} nodes remaining, lb {}, ub {}", bbtree_.size(), bbtree_.lb(), ub());
            DEBUGLN("Solving node {} (lb {}, depth {}, parent {})",
                    bbtree_.current_id(),
                    bbtree_.current_depth(),
                    bbtree_.current_lb(),
                    bbtree_.current_parent_id());

            // Set up the basis and disable variables incompatible with branching decisions.
            master_.start_node(bbtree_.current());

            // Reset history of objective value to check for stalled LP.
            std::fill(master_obj_history_.begin(), master_obj_history_.end(), COST_NAN);

            // LP, price and cut loop.
            do
            {
                // Update iteration count.
                ++iter_;
                ZoneScopedNC("Price and cut loop", TRACY_COLOUR);
                ZoneValue(iter_);

                // Solve the master problem.
                master_.solve();
                DEBUG_ASSERT(master_.status() != MasterProblemStatus::Unknown);

                // Print log.
                if (bbtree_.current_id() == 0 && iter_ >= next_log_iter_)
                {
                    print_node_log();
                    next_log_iter_ = iter_ + PRINT_LOG_ROOT_ITERATIONS;
                }

                // Print paths.
#ifdef PRINT_DEBUG
                if (master_.status() != MasterProblemStatus::Infeasible)
                {
                    print_paths();
                }
#endif

                // Store the objective value history of the master problem.
                store_master_obj_history(master_.obj());

                // Get the nodetimes and edgetimes in use.
                projection_.update();

                // Interrupt and proceed to branch if the LP objective value is stalled.
                if (master_.status() == MasterProblemStatus::Fractional &&
                    branch_early(master_.obj()))
                {
                    goto NODE_STALLED;
                }

                // Add debug paths in the first iteration.
                // static Bool added_debug_paths = false;
                // if (!added_debug_paths)
                // {
                //     add_debug_paths();
                // }
                // added_debug_paths = true;
                // const auto new_lb = COST_NAN;

                // Generate columns.
                const auto new_lb = pricer_.run();

                // Update the node lower bound.
                if (!std::isnan(new_lb))
                {
                    ZoneScopedNC("Update lower bound", TRACY_COLOUR);

                    // Update the node lower bound.
                    bbtree_.update_node_lb(new_lb);

                    // Skip the current node if its lower bound is higher than the upper
                    // bound.
                    if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
                    {
                        DEBUGLN("Discarding BB tree node {}, lb {}",
                                bbtree_.current_id(),
                                bbtree_.current_lb());
                        goto NODE_COMPLETED;
                    }
                }

                // Generate cuts. Update upper bound if integer and cuts not created.
                if (master_.status() != MasterProblemStatus::Infeasible && !master_.vars_added())
                {
                    // Create cuts.
                    lazy_constraints_.apply([](auto&... separator)
                                            { return (separator.run(), ...); });

                    // Store the result if the LP is integer feasible.
                    if (master_.status() == MasterProblemStatus::Integer &&
                        !master_.constrs_added())
                    {
                        // Increment the number of feasible solutions found by the LP.
                        ++lp_num_feasible_;

                        // Store the solution if improving.
                        if (master_.obj() < ub())
                        {
                            // Store the solution.
                            update_ub(master_.obj());
                            sol_.resize(A);
                            for (Agent a = 0; a < A; ++a)
                                for (const auto& variable_ptr : master_.agent_variable_ptrs(a))
                                {
                                    const auto val = master_.variable_primal_sol(*variable_ptr);
                                    if (is_gt(val, 0.5))
                                    {
                                        sol_[a] = variable_ptr;
                                        break;
                                    }
                                }
#ifdef DEBUG
                            check_solution(sol_, ub());
#endif
                            ++lp_num_improving_;

                            // Print log.
                            print_sol_log("LP relaxation");
                            next_log_iter_ =
                                iter_ + (bbtree_.current_id() == 0 ? PRINT_LOG_ROOT_ITERATIONS :
                                                                     PRINT_LOG_NODE_ITERATIONS);

                            // Skip the current node if its lower bound is higher than the upper
                            // bound.
                            if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
                            {
                                DEBUGLN("Discarding BB tree node {}, lb {}",
                                        bbtree_.current_id(),
                                        bbtree_.current_lb());
                                goto NODE_COMPLETED;
                            }
                        }
                    }
                }

                // Run primal heuristics.
                if (master_.status() != MasterProblemStatus::Infeasible)
                {
                    const auto cutoff =
                        heuristics_.apply([&](auto&... heuristic)
                                          { return (run_primal_heuristic(heuristic) || ...); });
                    if (cutoff)
                    {
                        goto NODE_COMPLETED;
                    }
                }
            } while (master_.changed());

            // Update the node lower bound.
            bbtree_.update_node_lb(master_.obj());

            // Skip the current node if its lower bound is higher than the upper bound.
            if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
            {
                DEBUGLN("Discarding BB tree node {}, lb {}",
                        bbtree_.current_id(),
                        bbtree_.current_lb());
                goto NODE_COMPLETED;
            }

            // Branch.
            if (master_.status() == MasterProblemStatus::Fractional)
            {
            NODE_STALLED:

                // Store basis in the node. Also delete old columns and rows.
                master_.store_basis(bbtree_.current());

                // Run the branchers.
                const auto success = branchers_.apply([&](auto&... brancher)
                                                      { return (run_brancher(brancher) || ...); });
                if (success)
                {
                    goto BRANCHED;
                }
                ERROR("All branchers failed to make a decision");
            }
        BRANCHED:

            // Print log.
            if (bbtree_.current_id() == 0 || iter_ >= next_log_iter_)
            {
                print_node_log();
                next_log_iter_ = iter_ + PRINT_LOG_NODE_ITERATIONS;
            }

        // Completed solving the node.
        NODE_COMPLETED:;

            // Print paths.
            // print_paths();
        }
    }
    catch (const TerminationException& e)
    {
    }
    print_log_separator();

    // Print final results.
    print_results();
}

void Problem::store_master_obj_history(const Cost master_obj)
{
    ZoneScopedC(TRACY_COLOUR);

    // Insert the latest objective value into the circular buffer.
    std::memmove(master_obj_history_.data(),
                 master_obj_history_.data() + 1,
                 sizeof(Cost) * (STALLED_NUM_ROUNDS - 1));
    master_obj_history_.back() = master_obj;

    // Print.
    DEBUGLN("");
    DEBUGLN("    LP history: {}",
            fmt::join(master_obj_history_.begin(), master_obj_history_.end(), " "));
}

Bool Problem::branch_early(const Cost master_obj)
{
    ZoneScopedC(TRACY_COLOUR);

    // Store the master objective value.
    store_master_obj_history(master_obj);

    // Check if stalled.
    for (Size64 idx = 0; idx < STALLED_NUM_ROUNDS - 1; ++idx)
    {
        const auto change = master_obj_history_[idx + 1] - master_obj_history_[idx];
        DEBUGLN("    LP absolute change {}", change);

        // Stalled if change is positive (due to cuts) or less than a given amount. Handle NaN using
        // double negation.
        if (!(change <= 0 && change > STALLED_ABSOLUTE_CHANGE))
        {
            return false;
        }
    }
    DEBUGLN("    Stalled");
    return true;
}

void Problem::update_ub(const Cost ub)
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(is_integral(ub));
    ub_ = std::min(ub_, eps_round(ub));
    DEBUGLN("Updated UB to {}", ub_);
}

template <class T>
Bool Problem::run_primal_heuristic(T& heuristic)
{
    ZoneScopedC(TRACY_COLOUR);

    // Exit if timed out.
    stop_if_timed_out();

    // Run the primal heuristic.
    auto [sol_cost, sol] = heuristic.run();
    DEBUG_ASSERT(is_ge(sol_cost, bbtree_.current_lb()));

    // Store improving solution.
    if (sol_cost < ub())
    {
        // Store the solution.
        heuristic.increment_num_improving();
        update_ub(sol_cost);
        sol_ = std::move(sol);
#ifdef DEBUG
        check_solution(sol_, ub());
#endif

        // Print log.
        print_sol_log(heuristic.name());
        next_log_iter_ = iter_ + (bbtree_.current_id() == 0 ? PRINT_LOG_ROOT_ITERATIONS :
                                                              PRINT_LOG_NODE_ITERATIONS);

        // Skip the current node if its lower bound is higher than the upper bound.
        if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
        {
            DEBUGLN(
                "Discarding BB tree node {}, lb {}", bbtree_.current_id(), bbtree_.current_lb());
            return true;
        }
    }
    return false;
}

template <class T>
Bool Problem::run_brancher(T& brancher)
{
    ZoneScopedC(TRACY_COLOUR);

    auto decisions = brancher.run();
    if (decisions.first && decisions.second)
    {
        // Create children nodes.
        bbtree_.branch(&brancher, std::move(decisions));

        // The node is completed.
        return true;
    }
    return false;
}

void Problem::print_log_separator() const
{
    ZoneScopedC(TRACY_COLOUR);

#ifdef DEBUG
    PRINTLN("{:-^156}", "");
#else
    PRINTLN("{:-^143}", "");
#endif
}

void Problem::print_log_header()
{
    ZoneScopedC(TRACY_COLOUR);

    print_log_separator();
    PRINTLN("{:>8s} | "
            "{:>9s} | "
            "{:>8s} | "
            "{:>8s} | "
            "{:>8s} | "
            "{:>7s} | "
            "{:>7s} | "
            "{:>10s} | "
#ifdef DEBUG
            "{:>10s} | "
#endif
            "{:>11s} | "
            "{:>11s} | "
            "{:>15s} | "
            "{:>8s}",
            "Time",
            "Iteration",
            "Closed",
            "Open",
            "Depth",
            "Columns",
            "Rows",
            "Master Obj",
#ifdef DEBUG
            "Kappa",
#endif
            "Lower Bound",
            "Upper Bound",
            "Found By",
            "Gap");
    print_log_separator();
}

void Problem::print_node_log()
{
    ZoneScopedC(TRACY_COLOUR);

    if (num_log_lines_ % PRINT_LOG_HEADER_LINES == 0)
    {
        print_log_header();
    }

    const auto master_obj = master_.obj();
    const auto master_obj_str =
        (std::isfinite(master_obj) ? fmt::format("{:10.2f}", master_obj) : "-");
    const auto ub_val = ub();
    const auto lb_val = lb();
    const auto gap = (ub_val - lb_val) / ub_val;
    const auto gap_str = std::isfinite(gap) ? fmt::format("{:7.2f}%", gap * 100.0) : "-";
    PRINTLN("{:>8.2f} | "
            "{:>9d} | "
            "{:>8d} | "
            "{:>8d} | "
            "{:>8d} | "
            "{:>7d} | "
            "{:>7d} | "
            "{:>10s} | "
#ifdef DEBUG
            "{:>10.2f} | "
#endif
            "{:>11.2f} | "
            "{:>11.2f} | "
            "{:>15s} | "
            "{:>8s}",
            clock_.elapsed_time(),
            iter_,
            bbtree_.num_closed(),
            bbtree_.size(),
            bbtree_.current_depth(),
            master_.num_cols(),
            master_.num_rows(),
            master_obj_str,
#ifdef DEBUG
            master_.approx_condition_number(),
#endif
            lb_val,
            ub_val,
            "-",
            gap_str);
    ++num_log_lines_;
}

void Problem::print_sol_log(const String& found_by)
{
    ZoneScopedC(TRACY_COLOUR);

    if (num_log_lines_ % PRINT_LOG_HEADER_LINES == 0)
    {
        print_log_header();
    }

    const auto master_obj = master_.obj();
    const auto master_obj_str =
        (std::isfinite(master_obj) ? fmt::format("{:10.2f}", master_obj) : "-");
    const auto ub_val = ub();
    const auto bbtree_lb_val = bbtree_.lb();
    const auto lb_val = std::min(bbtree_lb_val, ub_val);
    const auto gap = (ub_val - lb_val) / ub_val;
    const auto gap_str = std::isfinite(gap) ? fmt::format("{:7.2f}%", gap * 100.0) : "-";
    PRINTLN("{:>8.2f} | "
            "{:>9d} | "
            "{:>8d} | "
            "{:>8d} | "
            "{:>8d} | "
            "{:>7d} | "
            "{:>7d} | "
            "{:>10s} | "
#ifdef DEBUG
            "{:>10.2f} | "
#endif
            "{:>11.2f} | "
            "{:>11.2f} | "
            "{:>15s} | "
            "{:>8s}",
            clock_.elapsed_time(),
            iter_,
            bbtree_.num_closed(),
            bbtree_.size(),
            bbtree_.current_depth(),
            master_.num_cols(),
            master_.num_rows(),
            master_obj_str,
#ifdef DEBUG
            master_.approx_condition_number(),
#endif
            lb_val,
            ub_val,
            found_by,
            gap_str);
    ++num_log_lines_;
}

void Problem::print_results() const
{
    ZoneScopedC(TRACY_COLOUR);

    // Print solve statistics.
    {
        const auto solve_time = clock_.total_duration();
        const auto num_nodes = bbtree_.num_closed();
        const auto lb_val = eps_ceil(lb());
        const auto ub_val = ub();
        const auto status = (lb_val == COST_INF ? "Infeasible" :
                             ub_val == COST_INF ? "Unknown" :
                             lb_val == ub_val   ? "Optimal" :
                                                  "Feasible");
        const auto gap = (ub_val - lb_val) / ub_val;
        const auto gap_str = std::isfinite(gap) ? fmt::format("{:.2f}%", gap * 100.0) : "-";
        PRINTLN("");
        PRINTLN("{:<13} {:<.2f} seconds", "Solve time:", solve_time);
        PRINTLN("{:<13} {}", "Status:", status);
        PRINTLN("{:<13} {}", "Nodes solved:", num_nodes);
        PRINTLN("{:<13} {:.0f}", "Upper bound:", ub_val);
        PRINTLN("{:<13} {:.0f}", "Lower bound:", lb_val);
        PRINTLN("{:<13} {}", "Gap:", gap_str);
    }

    // Print pricers statistics.
    {
        PRINTLN("");
        PRINTLN("{:-^57}", "");
        PRINTLN("{:<28s} | "
                "{:>8s} | "
                "{:>15s}",
                "Pricer",
                "Time",
                "Columns Added");
        PRINTLN("{:-^57}", "");
        PRINTLN("{:<28s} | "
                "{:8.2f} | "
                "{:15d}",
                pricer_.name(),
                pricer_.run_time(),
                pricer_.num_added());
        PRINTLN("{:-^57}", "");
    }

    // Print separators statistics.
    {
        PRINTLN("");
        PRINTLN("{:-^57}", "");
        PRINTLN("{:<28s} | "
                "{:>8s} | "
                "{:>15s}",
                "Separator",
                "Time",
                "Rows Added");
        PRINTLN("{:-^57}", "");
        constexpr auto f = [](const auto& x)
        {
            PRINTLN("{:<28s} | "
                    "{:8.2f} | "
                    "{:15d}",
                    x.name(),
                    x.run_time(),
                    x.num_added());
        };
        initial_constraints_.apply([&](auto&... subroutine) { (f(subroutine), ...); });
        lazy_constraints_.apply([&](auto&... subroutine) { (f(subroutine), ...); });
        PRINTLN("{:-^57}", "");
    }

    // Print branchers statistics.
    {
        PRINTLN("");
        PRINTLN("{:-^57}", "");
        PRINTLN("{:<28s} | "
                "{:>8s} | "
                "{:>15s}",
                "Brancher",
                "Time",
                "Nodes Added");
        PRINTLN("{:-^57}", "");
        constexpr auto f = [](const auto& x)
        {
            PRINTLN("{:<28s} | "
                    "{:8.2f} | "
                    "{:15d}",
                    x.name(),
                    x.run_time(),
                    x.num_added());
        };
        branchers_.apply([&](auto&... subroutine) { (f(subroutine), ...); });
        PRINTLN("{:-^57}", "");
    }

    // Print primal heuristics statistics.
    {
        PRINTLN("");
        PRINTLN("{:-^75}", "");
        PRINTLN("{:<28s} | "
                "{:>8s} | "
                "{:>15s} | "
                "{:>15s}",
                "Solutions",
                "Time",
                "Improving Found",
                "Feasible Found");
        PRINTLN("{:-^75}", "");
        PRINTLN("{:<28s} | "
                "{:8.2f} | "
                "{:15d} | "
                "{:15d}",
                "LP relaxation",
                master_.run_time(),
                lp_num_improving_,
                lp_num_feasible_);
        constexpr auto f = [](const auto& x)
        {
            PRINTLN("{:<28s} | "
                    "{:8.2f} | "
                    "{:15d} | "
                    "{:15d}",
                    x.name(),
                    x.run_time(),
                    x.num_improving(),
                    x.num_feasible());
        };
        heuristics_.apply([&](auto&... subroutine) { (f(subroutine), ...); });
        PRINTLN("{:-^75}", "");
    }

    // Print statistics for other activities.
    {
        PRINTLN("");
        PRINTLN("{:-^39}", "");
        PRINTLN("{:<28s} | "
                "{:>8s}",
                "Other Activities",
                "Time");
        PRINTLN("{:-^39}", "");
        PRINTLN("{:<28s} | "
                "{:8.2f}",
                "Projection",
                projection_.run_time());
        PRINTLN("{:-^39}", "");
    }

    // Print solution.
#ifndef DEBUG
    if (!sol_.empty())
    {
        PRINTLN("");
        PRINTLN("Solution:");
        print_solution(instance_.map, sol_);
    }
#endif
}

void Problem::print_paths()
{
    PRINTLN("");
    PRINTLN("Paths with cost {:.4f} in B&B node {}, lb {:.4f}, depth {}, "
            "condition number {}",
            master_.obj(),
            bbtree_.current_id(),
            bbtree_.current_lb(),
            bbtree_.current_depth(),
            master_.condition_number());
    master_.print_paths();
    PRINTLN("");
}

#ifdef DEBUG

void Problem::check_solution(const Vector<Variable*>& solution, const Cost cost) const
{
    // Get the problem data.
    const auto& map = instance_.map;
    const auto A = instance_.num_agents();

    // Check size.
    ASSERT(
        solution.size() == A, "Solution has {} paths but there are {} agents", solution.size(), A);

    // Check for conflicts.
    Cost check_cost = 0.0;
    HashSet<NodeTime> nodetime_in_use;
    HashSet<EdgeTime> edgetime_in_use;
    HashMap<Node, Time> target_crossed;
    HashMap<Node, Time> target_blocked;
    for (Agent a = 0; a < A; ++a)
    {
        const auto target = instance_.agents[a].target;
        target_blocked[target] = TIME_MAX;
        target_crossed[target] = -1;
    }
    for (Agent a = 0; a < A; ++a)
    {
        // Check the start and target.
        const auto& path = solution[a]->path();
        ASSERT(path.front().n == instance_.agents[a].start,
               "The path for agent {} starts at {} but its starting location is {}",
               a,
               format_node(path.front().n, map),
               format_node(instance_.agents[a].start, map));
        ASSERT(path.back().n == instance_.agents[a].target,
               "The path for agent {} ends at {} but its target location is {}",
               a,
               format_node(path.back().n, map),
               format_node(instance_.agents[a].target, map));

        // Check if the path is in conflict with paths already selected.
        {
            NodeTime nt{0, 0};
            for (; nt.t < path.size() - 1; ++nt.t)
            {
                nt.n = path[nt.t].n;
                ASSERT(nodetime_in_use.find(nt) == nodetime_in_use.end(),
                       "Agent {} has a nodetime conflict at {}",
                       a,
                       format_nodetime(nt, map));

                auto it = target_blocked.find(nt.n);
                ASSERT(it == target_blocked.end() || it->second > nt.t,
                       "Agent {} is crossing {} at time {} but it is blocked from time {}",
                       a,
                       format_node(nt.n, map),
                       nt.t,
                       it->second);

                EdgeTime et{map.get_undirected_edge(path[nt.t]), nt.t};
                ASSERT(edgetime_in_use.find(et) == edgetime_in_use.end(),
                       "Agent {} has an edgetime conflict at {}",
                       a,
                       format_edgetime(et, map));
            }
            {
                nt.n = path[nt.t].n;
                ASSERT(target_crossed.at(nt.n) < nt.t,
                       "Agent {} wants to finish at time {} but another agent "
                       "is crossing its target at time {}",
                       a,
                       nt.t,
                       target_crossed.at(nt.n));
            }
        }

        // Store nodetimes and edgetimes of the selected path.
        {
            NodeTime nt{0, 0};
            for (; nt.t < path.size() - 1; ++nt.t)
            {
                nt.n = path[nt.t].n;
                nodetime_in_use.insert(nt);

                if (auto it = target_crossed.find(nt.n); it != target_crossed.end())
                {
                    auto& target_crossed_time = it->second;
                    target_crossed_time = std::max(target_crossed_time, nt.t);
                }

                EdgeTime et{map.get_undirected_edge(path[nt.t]), nt.t};
                edgetime_in_use.insert(et);
            }
            {
                nt.n = path[nt.t].n;
                auto& target_blocked_time = target_blocked.at(nt.n);
                DEBUG_ASSERT(target_blocked_time == TIME_MAX);
                target_blocked_time = nt.t;
            }

            // Update the cost.
            check_cost += path.size() - 1;
        }
    }

    // Check the cost.
    ASSERT(check_cost == cost,
           "The solution cost is {} and does not match the expected cost {}",
           check_cost,
           cost);
}

#endif

// void Problem::add_debug_paths()
// {
//     Vector<Vector<XY>> paths_xy{
// #include "debug_paths.txt"
//     };
//     for (auto& path_xy : paths_xy)
//         for (auto& xy : path_xy)
//         {
//             ++xy.x;
//             ++xy.y;
//         }
//     for (Agent a = 0; a < paths_xy.size(); ++a)
//     {
//         const auto& path_xy = paths_xy[a];
//
//         auto variable_ptr = Variable::construct(a, path_xy.size());
//         auto path = variable_ptr->path();
//
//         Time t = 0;
//         for (; t < path_xy.size() - 1; ++t)
//         {
//             const auto from_n = instance_.map.get_n(path_xy[t]);
//             const auto to_n = instance_.map.get_n(path_xy[t + 1]);
//             const auto d = instance_.map.get_direction(from_n, to_n);
//             path[t] = Edge{from_n, d};
//         }
//         {
//             const auto from_n = instance_.map.get_n(path_xy[t]);
//             path[t] = Edge{from_n, Direction::INVALID};
//         }
//         ASSERT(path.front().n == instance_.agents[a].start);
//         ASSERT(path.back().n == instance_.agents[a].target);
//
//         PRINTLN("Agent {:3d}: {}", a, format_path_with_time_spaced(path,
//         instance_.map));
//
//         const auto reduced_cost = master_.calculate_reduced_cost(a, path);
//         PRINTLN("Reduced cost of path for agent {} = {}", a, reduced_cost);
//         if (is_lt(reduced_cost, 0.0))
//         {
//             master_.add_column(std::move(variable_ptr), reduced_cost);
//         }
//     }
//     exit(0);
// }
