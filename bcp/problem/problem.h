/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "bbtree/bbtree.h"
#include "bbtree/length_brancher.h"
#include "bbtree/negative_nodetime_brancher.h"
#include "bbtree/nodetime_brancher.h"
#include "constraints/agent.h"
#include "constraints/corridor_conflict.h"
#include "constraints/edgetime_conflict.h"
#include "constraints/exit_entry_conflict.h"
#include "constraints/multi_agent_exit_entry_conflict.h"
#include "constraints/multi_agent_two_edge_conflict.h"
#include "constraints/nodetime_conflict.h"
#include "constraints/pseudo_corridor_conflict.h"
#include "constraints/rectangle_clique_conflict.h"
#include "constraints/rectangle_knapsack_conflict.h"
#include "constraints/target_conflict.h"
#include "constraints/two_edge_conflict.h"
#include "heuristics/random_rounding.h"
#include "heuristics/simple_rounding.h"
#include "master/master.h"
#include "pricing/pricer.h"
#include "problem/projection.h"
#include "problem/subroutine_set.h"
#include "types/basic_types.h"
#include "types/clock.h"
#include <csignal>
#include <exception>

extern volatile std::sig_atomic_t raised_signal;

class TerminationException : public std::exception
{
  public:
    const char* what() const noexcept override
    {
        return "Termianted";
    }
};

class Problem
{
    // Data
    const Instance instance_;

    // Solver
    UInt64 iter_;
    BBTree bbtree_;
    MasterProblem master_;
    Vector<Cost> master_obj_history_;
    Pricer pricer_;
    SubroutineSet<AgentSeparator> initial_constraints_;
    SubroutineSet<NodeTimeConflictSeparator, EdgeTimeConflictSeparator, CorridorConflictSeparator,
                  ExitEntryConflictSeparator, PseudoCorridorConflictSeparator,
#ifdef USE_RECTANGLE_CLIQUE_CUTS
                  RectangleCliqueConflictSeparator,
#else
                  RectangleKnapsackConflictSeparator,
#endif
                  TargetConflictSeparator, TwoEdgeConflictSeparator
                  // MultiAgentExitEntryConflictSeparator,
                  // MultiAgentTwoEdgeConflictSeparator
                  >
        lazy_constraints_;
    SubroutineSet<LengthBrancher, NodeTimeBrancher
                  // NegativeNodeTimeBrancher
                  >
        branchers_;
    SubroutineSet<SimpleRounding, RandomRounding> heuristics_;

    // Solution
    Cost ub_;
    Vector<Variable*> sol_;
    Projection projection_;

    // Helper data
    Size64 lp_num_feasible_;
    Size64 lp_num_improving_;
    Clock clock_;
    UInt64 num_log_lines_;
    UInt64 next_log_iter_;

  public:
    // Constructors and destructor
    Problem() = delete;
    ~Problem() = default;
    Problem(const Problem&) = delete;
    Problem(Problem&&) = delete;
    Problem& operator=(const Problem&) = delete;
    Problem& operator=(Problem&&) = delete;
    Problem(const FilePath& scenario_path, const Agent agent_limit);

    // Getters
    const auto& instance() const
    {
        return instance_;
    }
    auto iter() const
    {
        return iter_;
    }
    auto& bbtree()
    {
        return bbtree_;
    }
    auto& master()
    {
        return master_;
    }
    auto& pricer()
    {
        return pricer_;
    }
    auto& projection()
    {
        return projection_;
    }
    Cost lb() const;
    Cost ub() const;
    Real64 gap() const;

    // Solve
    void solve(const Real64 time_limit = REAL_INF);
    inline void stop_if_timed_out() const
    {
        if (raised_signal || timed_out()) [[unlikely]]
            throw TerminationException();
    }
    inline auto time_remaining() const
    {
        return clock_.time_remaining();
    }
    // static inline void stop_if_terminated()
    // {
    //     if (raised_signal) [[unlikely]]
    //         throw TerminationException();
    // }

    // Debug
    void print_paths();
#ifdef DEBUG
    void check_solution(const Vector<Variable*>& solution, const Cost cost) const;
#endif
    // void add_debug_paths();

  protected:
    // Solve
    void store_master_obj_history(const Cost master_obj);
    Bool branch_early(const Cost master_obj);
    void update_ub(const Cost ub);
    template <class T>
    Bool run_primal_heuristic(T& heuristic);
    template <class T>
    Bool run_brancher(T& heuristic);
    inline Bool timed_out() const
    {
        return clock_.timed_out();
    }

    // Get solution
    void update_projection();

    // Printing
    void print_log_separator() const;
    void print_log_header();
    void print_node_log();
    void print_sol_log(const String& name);
    void print_results() const;
};
