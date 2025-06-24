/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "output/formatting.h"
#include "problem/debug.h"
#include "problem/problem.h"
#include "problem/runtime_tests.h"
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

    ub_(INF),
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
    std::apply([](auto&... separator) { return (separator.run(), ...); }, initial_constraints_.subroutines);
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

Float Problem::gap() const
{
    ZoneScopedC(TRACY_COLOUR);

    const auto ub_val = ub();
    const auto lb_val = lb();
    return (ub_val - lb_val) / ub_val;
}

void Problem::solve(const Float time_limit)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const Agent A = instance_.agents.size();

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
                debugln("Discarding BB tree node {}, lb {}", bbtree_.current_id(), bbtree_.current_lb());
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
            debugln("BB tree has {} nodes remaining, lb {}, ub {}", bbtree_.size(), bbtree_.lb(), ub());
            debugln("Solving node {} (lb {}, depth {}, parent {})",
                    bbtree_.current_id(), bbtree_.current_depth(), bbtree_.current_lb(), bbtree_.current_parent_id());

            // Start the node.
            master_.start_node(bbtree_.current());

            // Disable variables removed by branching.
            for (const auto& [brancher, decision] : bbtree_.all_decisions())
            {
                brancher->disable_vars(decision);
            }

            // Reset history of objective value to check for stalled LP.
            std::fill(master_obj_history_.begin(), master_obj_history_.end(), std::numeric_limits<Cost>::quiet_NaN());

            // LP, price and cut loop.
            do
            {
                // Update iteration count.
                ++iter_;
                ZoneScopedNC("Price and cut loop", TRACY_COLOUR);
                ZoneValue(iter_);

                // Solve the master problem.
                master_.solve();
                debug_assert(master_.status() != MasterProblemStatus::Unknown);

                // Print log.
                if (bbtree_.current_id() == 0 && iter_ >= next_log_iter_)
                {
                    print_node_log();
                    next_log_iter_ = iter_ + PRINT_LOG_ROOT_ITERATIONS;
                }

                // Store the objective value history of the master problem.
                store_master_obj_history(master_.obj());

                // Get the nodetimes and edgetimes in use.
                projection_.update();

                // Interrupt and proceed to branch if the LP objective value is stalled.
                if (master_.status() == MasterProblemStatus::Fractional && branch_early(master_.obj()))
                {
                    goto NODE_STALLED;
                }

                // Print paths.
// #ifdef PRINT_DEBUG
//                 if (master_.status() != MasterProblemStatus::Infeasible)
//                 {
//                     print_paths();
//                 }
// #endif

                // Add debug paths in the first iteration.
                // static Bool added_debug_paths = false;
                // if (!added_debug_paths)
                // {
                //     add_debug_paths();
                // }
                // added_debug_paths = true;
                // const auto new_lb = std::numeric_limits<Cost>::quiet_NaN();

                // Generate columns.
                const auto new_lb = pricer_.run();

                // Update the node lower bound.
                if (!std::isnan(new_lb))
                {
                    ZoneScopedNC("Update lower bound", TRACY_COLOUR);

                    // Update the node lower bound.
                    bbtree_.update_node_lb(new_lb);

                    // Skip the current node if its lower bound is higher than the upper bound.
                    if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
                    {
                        debugln("Discarding BB tree node {}, lb {}", bbtree_.current_id(), bbtree_.current_lb());
                        goto NODE_COMPLETED;
                    }
                }

                // Generate cuts. Update upper bound if integer and cuts not created.
                if (master_.status() != MasterProblemStatus::Infeasible && !master_.vars_added())
                {
                    // Create cuts.
                    std::apply([](auto&... separator) { return (separator.run(), ...); },
                               lazy_constraints_.subroutines);

                    // Store the result if the LP is integer feasible.
                    if (master_.status() == MasterProblemStatus::Integer && !master_.constrs_added())
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
                            next_log_iter_ = iter_ +
                                (bbtree_.current_id() == 0 ? PRINT_LOG_ROOT_ITERATIONS : PRINT_LOG_NODE_ITERATIONS);

                            // Skip the current node if its lower bound is higher than the upper bound.
                            if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
                            {
                                debugln("Discarding BB tree node {}, lb {}", bbtree_.current_id(), bbtree_.current_lb());
                                goto NODE_COMPLETED;
                            }
                        }
                    }
                }

                // Run primal heuristics.
                if (master_.status() != MasterProblemStatus::Infeasible)
                {
                    static_assert(std::tuple_size<decltype(heuristics_.subroutines)>() == 2);
                    if (run_primal_heuristic<0>())
                    {
                        goto NODE_COMPLETED;
                    }
                    if (run_primal_heuristic<1>())
                    {
                        goto NODE_COMPLETED;
                    }
                }
            }
            while (master_.changed());

            // Update the node lower bound.
            bbtree_.update_node_lb(master_.obj());

            // Skip the current node if its lower bound is higher than the upper bound.
            if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
            {
                debugln("Discarding BB tree node {}, lb {}", bbtree_.current_id(), bbtree_.current_lb());
                goto NODE_COMPLETED;
            }

            // Branch.
            if (master_.status() == MasterProblemStatus::Fractional)
            {
                NODE_STALLED:

                // Store basis in the node. Also delete old columns and rows.
                master_.store_basis(bbtree_.current());

                // Run the branchers.
                static_assert(std::tuple_size<decltype(branchers_.subroutines)>() == 2);
                if (run_brancher<0>())
                {
                    goto BRANCHED;
                }
                if (run_brancher<1>())
                {
                    goto BRANCHED;
                }
                err("All branchers failed to make a decision");
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
    debugln("");
    debugln("    LP history: {}", fmt::join(master_obj_history_.begin(), master_obj_history_.end(), " "));
}

Bool Problem::branch_early(const Cost master_obj)
{
    ZoneScopedC(TRACY_COLOUR);

    // Store the master objective value.
    store_master_obj_history(master_obj);

    // Check if stalled.
    for (Size idx = 0; idx < STALLED_NUM_ROUNDS - 1; ++idx)
    {
        const auto change = master_obj_history_[idx + 1] - master_obj_history_[idx];
        debugln("    LP absolute change {}", change);

        // Stalled if change is positive (due to cuts) or less than a given amount. Handle NaN using double negation.
        if (!(change <= 0 && change > STALLED_ABSOLUTE_CHANGE))
        {
            return false;
        }
    }
    debugln("    Stalled");
    return true;
}

void Problem::update_ub(const Cost ub)
{
    ZoneScopedC(TRACY_COLOUR);

    debug_assert(is_integral(ub));
    ub_ = std::min(ub_, eps_round(ub));
    debugln("Updated UB to {}", ub_);
}

template<Size index>
Bool Problem::run_primal_heuristic()
{
    ZoneScopedC(TRACY_COLOUR);

    // Exit if timed out.
    stop_if_timed_out();

    // Run the primal heuristic.
    auto& heuristic = heuristics_.get<index>();
    auto [sol_cost, sol] = heuristic.run();
    debug_assert(is_ge(sol_cost, bbtree_.current_lb()));

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
        next_log_iter_ = iter_ +
            (bbtree_.current_id() == 0 ? PRINT_LOG_ROOT_ITERATIONS : PRINT_LOG_NODE_ITERATIONS);

        // Skip the current node if its lower bound is higher than the upper bound.
        if (is_ge(eps_ceil(bbtree_.current_lb()), ub()))
        {
            debugln("Discarding BB tree node {}, lb {}", bbtree_.current_id(), bbtree_.current_lb());
            return true;
        }
    }
    return false;
}

template<Size index>
Bool Problem::run_brancher()
{
    ZoneScopedC(TRACY_COLOUR);

    auto& brancher = branchers_.get<index>();
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
    println("{:-^156}", "");
#else
    println("{:-^143}", "");
#endif
}

void Problem::print_log_header()
{
    ZoneScopedC(TRACY_COLOUR);

    print_log_separator();
#ifdef DEBUG
    println("{:>8s} | {:>9s} | {:>8s} | {:>8s} | {:>8s} | {:>7s} | {:>7s} | {:>10s} | {:>10s} | {:>11s} | {:>11s} | {:>15s} | {:>8s}",
            "Time", "Iteration", "Closed", "Open", "Depth", "Columns", "Rows", "Master Obj", "Kappa", "Lower Bound", "Upper Bound", "Found By", "Gap");
#else
    println("{:>8s} | {:>9s} | {:>8s} | {:>8s} | {:>8s} | {:>7s} | {:>7s} | {:>10s} | {:>11s} | {:>11s} | {:>15s} | {:>8s}",
            "Time", "Iteration", "Closed", "Open", "Depth", "Columns", "Rows", "Master Obj", "Lower Bound", "Upper Bound", "Found By", "Gap");
#endif
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
    const auto master_obj_str = (std::isfinite(master_obj) ? fmt::format("{:10.2f}", master_obj) : "-");
    const auto ub_val = ub();
    const auto lb_val = lb();
    const auto gap = (ub_val - lb_val) / ub_val;
    const auto gap_str = std::isfinite(gap) ? fmt::format("{:7.2f}%", gap * 100.0) : "-";
#ifdef DEBUG
    println("{:>8.2f} | {:>9d} | {:>8d} | {:>8d} | {:>8d} | {:>7d} | {:>7d} | {:>10s} | {:>10.2f} | {:>11.2f} | {:>11.2f} | {:>15s} | {:>8s}",
#else
    println("{:>8.2f} | {:>9d} | {:>8d} | {:>8d} | {:>8d} | {:>7d} | {:>7d} | {:>10s} | {:>11.2f} | {:>11.2f} | {:>15s} | {:>8s}",
#endif
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
    const auto master_obj_str = (std::isfinite(master_obj) ? fmt::format("{:10.2f}", master_obj) : "-");
    const auto ub_val = ub();
    const auto bbtree_lb_val = bbtree_.lb();
    const auto lb_val = std::min(bbtree_lb_val, ub_val);
    const auto gap = (ub_val - lb_val) / ub_val;
    const auto gap_str = std::isfinite(gap) ? fmt::format("{:7.2f}%", gap * 100.0) : "-";
#ifdef DEBUG
    println("{:>8.2f} | {:>9d} | {:>8d} | {:>8d} | {:>8d} | {:>7d} | {:>7d} | {:>10s} | {:>10.2f} | {:>11.2f} | {:>11.2f} | {:>15s} | {:>8s}",
#else
    println("{:>8.2f} | {:>9d} | {:>8d} | {:>8d} | {:>8d} | {:>7d} | {:>7d} | {:>10s} | {:>11.2f} | {:>11.2f} | {:>15s} | {:>8s}",
#endif
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

    const auto solve_time = clock_.total_duration();
    const auto num_nodes = bbtree_.num_closed();
    const auto lb_val = eps_ceil(lb());
    const auto ub_val = ub();
    const auto status = (lb_val == INF    ? "Infeasible" :
                         ub_val == INF    ? "Unknown" :
                         lb_val == ub_val ? "Optimal" :
                                            "Feasible");
    const auto gap = (ub_val - lb_val) / ub_val;
    const auto gap_str = std::isfinite(gap) ? fmt::format("{:.2f}%", gap * 100.0) : "-";

    println("");
    println("{:<13} {:<.2f} seconds", "Solve time:", solve_time);
    println("{:<13} {}", "Status:", status);
    println("{:<13} {}", "Nodes solved:", num_nodes);
    println("{:<13} {:.0f}", "Upper bound:", ub_val);
    println("{:<13} {:.0f}", "Lower bound:", lb_val);
    println("{:<13} {}", "Gap:", gap_str);

    println("");
    println("{:-^57}", "");
    println("{:<28s} | {:>8s} | {:>15s}", "Pricer", "Time", "Columns Added");
    println("{:-^57}", "");
    println("{:<28s} | {:8.2f} | {:15d}", pricer_.name(), pricer_.run_time(), pricer_.num_added());
    println("{:-^57}", "");

    static_assert(decltype(initial_constraints_)::size() == 1, "Incorrect number of initial constraint families for printing");
    static_assert(decltype(lazy_constraints_)::size() == 8, "Incorrect number of lazy constraint families for printing");
    println("");
    println("{:-^57}", "");
    println("{:<28s} | {:>8s} | {:>15s}", "Separator", "Time", "Rows Added");
    println("{:-^57}", "");
    println("{:<28s} | {:8.2f} | {:15d}",
            initial_constraints_.get<0>().name(), initial_constraints_.get<0>().run_time(),
            initial_constraints_.get<0>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<0>().name(), lazy_constraints_.get<0>().run_time(),
            lazy_constraints_.get<0>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<1>().name(), lazy_constraints_.get<1>().run_time(),
            lazy_constraints_.get<1>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<2>().name(), lazy_constraints_.get<2>().run_time(),
            lazy_constraints_.get<2>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<3>().name(), lazy_constraints_.get<3>().run_time(),
            lazy_constraints_.get<3>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<4>().name(), lazy_constraints_.get<4>().run_time(),
            lazy_constraints_.get<4>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<5>().name(), lazy_constraints_.get<5>().run_time(),
            lazy_constraints_.get<5>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<6>().name(), lazy_constraints_.get<6>().run_time(),
            lazy_constraints_.get<6>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            lazy_constraints_.get<7>().name(), lazy_constraints_.get<7>().run_time(),
            lazy_constraints_.get<7>().num_added());
    // println("{:<28s} | {:8.2f} | {:15d}",
    //         lazy_constraints_.get<8>().name(), lazy_constraints_.get<8>().run_time(),
    //         lazy_constraints_.get<8>().num_added());
    // println("{:<28s} | {:8.2f} | {:15d}",
    //         lazy_constraints_.get<9>().name(), lazy_constraints_.get<9>().run_time(),
    //         lazy_constraints_.get<9>().num_added());
    println("{:-^57}", "");

    static_assert(decltype(branchers_)::size() == 2, "Incorrect number of branchers for printing");
    println("");
    println("{:-^57}", "");
    println("{:<28s} | {:>8s} | {:>15s}", "Brancher", "Time", "Nodes Added");
    println("{:-^57}", "");
    println("{:<28s} | {:8.2f} | {:15d}",
            branchers_.get<0>().name(), branchers_.get<0>().run_time(), branchers_.get<0>().num_added());
    println("{:<28s} | {:8.2f} | {:15d}",
            branchers_.get<1>().name(), branchers_.get<1>().run_time(), branchers_.get<1>().num_added());
    println("{:-^57}", "");

    static_assert(decltype(heuristics_)::size() == 2, "Incorrect number of primal heuristics for printing");
    println("");
    println("{:-^75}", "");
    println("{:<28s} | {:>8s} | {:>15s} | {:>15s}", "Solutions", "Time", "Improving Found", "Feasible Found");
    println("{:-^75}", "");
    println("{:<28s} | {:8.2f} | {:15d} | {:15d}",
            "LP relaxation", master_.run_time(), lp_num_improving_, lp_num_feasible_);
    println("{:<28s} | {:8.2f} | {:15d} | {:15d}",
            heuristics_.get<0>().name(), heuristics_.get<0>().run_time(),
            heuristics_.get<0>().num_improving(), heuristics_.get<0>().num_feasible());
    println("{:<28s} | {:8.2f} | {:15d} | {:15d}",
            heuristics_.get<1>().name(), heuristics_.get<1>().run_time(),
            heuristics_.get<1>().num_improving(), heuristics_.get<1>().num_feasible());
    println("{:-^75}", "");

    println("");
    println("{:-^39}", "");
    println("{:<28s} | {:>8s}", "Other Activities", "Time");
    println("{:-^39}", "");
    println("{:<28s} | {:8.2f}", "Projection", projection_.run_time());
    println("{:-^39}", "");

#ifndef DEBUG
    if (!sol_.empty())
    {
        println("");
        println("Solution:");
        print_solution(instance_.map, sol_);
    }
#endif
}

void Problem::print_paths()
{
    println("");
    println("Paths with cost {:.4f} in B&B node {}, lb {:.4f}, depth {}, condition number {}",
            master_.obj(),
            bbtree_.current_id(),
            bbtree_.current_lb(),
            bbtree_.current_depth(),
            master_.condition_number());
    master_.print_paths();
    println("");
}

#ifdef DEBUG

void Problem::check_solution(const Vector<Variable*>& solution, const Cost cost) const
{
    // Get the problem data.
    const auto& map = instance_.map;
    const Agent A = instance_.agents.size();

    // Check size.
    release_assert(solution.size() == A, "Solution has {} paths but there are {} agents", solution.size(), A);

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
        release_assert(path.front().n == instance_.agents[a].start,
                       "The path for agent {} starts at {} but its starting location is {}",
                       a,
                       format_node(path.front().n, map),
                       format_node(instance_.agents[0].start, map));
        release_assert(path.back().n == instance_.agents[a].target,
                       "The path for agent {} ends at {} but its target location is {}",
                       a,
                       format_node(path.back().n, map),
                       format_node(instance_.agents[0].target, map));

        // Check if the path is in conflict with paths already selected.
        {
            NodeTime nt{0, 0};
            for (; nt.t < path.size() - 1; ++nt.t)
            {
                nt.n = path[nt.t].n;
                release_assert(nodetime_in_use.find(nt) == nodetime_in_use.end(),
                               "Agent {} has a nodetime conflict at {}",
                               a, format_nodetime(nt, map));

                auto it = target_blocked.find(nt.n);
                release_assert(it == target_blocked.end() || it->second > nt.t,
                               "Agent {} is crossing {} at time {} but it is blocked from time {}",
                               a, format_node(nt.n, map), nt.t, it->second);

                EdgeTime et{map.get_undirected_edge(path[nt.t]), nt.t};
                release_assert(edgetime_in_use.find(et) == edgetime_in_use.end(),
                               "Agent {} has an edgetime conflict at {}",
                               a, format_edgetime(et, map));
            }
            {
                nt.n = path[nt.t].n;
                release_assert(target_crossed.at(nt.n) < nt.t,
                              "Agent {} wants to finish at time {} but another agent is crossing its target at time {}",
                              a, nt.t, target_crossed.at(nt.n));
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
                debug_assert(target_blocked_time == TIME_MAX);
                target_blocked_time = nt.t;
            }

            // Update the cost.
            check_cost += path.size() - 1;
        }
    }

    // Check the cost.
    release_assert(check_cost == cost,
                   "The solution cost is {} and does not match the expected cost {}",
                   check_cost, cost);
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
//         release_assert(path.front().n == instance_.agents[a].start);
//         release_assert(path.back().n == instance_.agents[a].target);
//
//         println("Agent {:3d}: {}", a, format_path_with_time_spaced(path, instance_.map));
//
//         const auto reduced_cost = master_.calculate_reduced_cost(a, path);
//         println("Reduced cost of path for agent {} = {}", a, reduced_cost);
//         if (is_lt(reduced_cost, 0.0))
//         {
//             master_.add_column(std::move(variable_ptr), reduced_cost);
//         }
//     }
//     exit(0);
// }
