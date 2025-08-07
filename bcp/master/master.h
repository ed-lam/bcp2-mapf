/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/constraint_pool.h"
#include "master/gurobi_lp.h"
#include "master/variable_pool.h"
#include "problem/instance.h"
#include "types/basic_types.h"
#include "types/clock.h"
#include "types/pointers.h"
#include "types/ranges.h"

class Problem;
class Constraint;
struct BBNode;
class Variable;
struct Instance;

enum class MasterProblemStatus
{
    Infeasible,
    Fractional,
    Integer,
    Unknown
};

class MasterProblem
{
    // Problem
    const Instance& instance_;
    Problem& problem_;

    // Master problem
    Clock clock_;
    GurobiLP lp_;
    MasterProblemStatus status_;
    Size vars_added_;
    Size constrs_added_;

    // Variables
    VariablePool variable_pool_;
    Vector<Variable*> all_variables_;
    Vector<Variable*>* agent_variables_;

    // Constraints
    ConstraintPool constraint_pool_;
    Vector<Constraint*> all_constraints_;
    Vector<Constraint*> universal_constraints_;
    Vector<Constraint*> subset_constraints_;
    Vector<Constraint*>* agent_constraints_;

    // Temporary buffers
    Vector<ColumnIndex> disable_cols_;

  public:
    // Constructors and destructor
    MasterProblem() = delete;
    MasterProblem(const Instance& instance, Problem& problem);
    ~MasterProblem();
    MasterProblem(const MasterProblem&) = delete;
    MasterProblem(MasterProblem&&) = delete;
    MasterProblem& operator=(const MasterProblem&) = delete;
    MasterProblem& operator=(MasterProblem&&) = delete;

    // Getters
    inline auto status() const { return status_; }
    inline auto obj() const { debug_assert(!(lp_.obj() <= 0.0)); return lp_.obj(); }
    inline auto num_cols() const { return lp_.num_cols(); }
    inline auto num_rows() const { return lp_.num_rows(); }
    inline auto condition_number() const { return lp_.condition_number(); }
    inline auto approx_condition_number() const { return lp_.approx_condition_number(); }
    inline auto vars_added() const { return vars_added_; }
    inline auto constrs_added() const { return constrs_added_; }
    inline auto changed() const { return static_cast<Bool>(vars_added_ + constrs_added_); }
    inline auto variable_primal_sol(const Variable& variable) const { return lp_.get_primal_sol(variable.col()); }
    inline auto constraint_dual_sol(const Constraint& constraint) const { return lp_.get_dual_sol(constraint.row()); }
    inline auto variable_basis(const Variable& variable) const { return lp_.get_col_basis(variable.col()); }
    inline auto constraint_basis(const Constraint& constraint) const { return lp_.get_row_basis(constraint.row()); }
    inline auto constraint_slack(const Constraint& constraint) const { return lp_.get_slack(constraint.row()); }

    // Statistics
    auto run_time() const { return clock_.total_duration(); }

    // Views of variables
    auto all_variables() const
    {
        return all_variables_ |
               Ranges::views::transform([](auto& ptr) -> const Variable& { return *ptr; });
    }
    auto agent_variables(const Agent a) const
    {
        return agent_variables_[a] |
               Ranges::views::transform([](const auto& ptr) -> const Variable& { return *ptr; });
    }
    const auto& agent_variable_ptrs(const Agent a) const
    {
        return agent_variables_[a];
    }

    // Views of constraints
    auto all_constraints() const
    {
        return all_constraints_ |
               Ranges::views::transform([](const auto& ptr) -> const Constraint& { return *ptr; });
    }
    auto universal_constraints() const
    {
        return universal_constraints_ |
               Ranges::views::transform([](const auto& ptr) -> const Constraint& { return *ptr; });
    }
    auto subset_constraints() const
    {
        return subset_constraints_ |
               Ranges::views::transform([](const auto& ptr) -> const Constraint& { return *ptr; });
    }
    auto agent_constraints(const Agent a) const
    {
        return agent_constraints_[a] |
               Ranges::views::transform([](const auto& ptr) -> const Constraint& { return *ptr; });
    }
    auto convexity_constraints() const
    {
        return all_constraints() |
               Ranges::views::take(instance_.agents.size());
    }

    // Modifications
    void add_column(UniquePtr<Variable, FreeDeleter>&& variable_ptr, const Cost reduced_cost);
    void add_permanent_row(UniquePtr<Constraint, FreeDeleter>&& constraint);
    void add_row(UniquePtr<Constraint, FreeDeleter>&& constraint);
    void disable_variable(const Variable& variable);

    // Solve
    void start_node(BBNode& bbnode);
    void solve();
    void store_basis(BBNode& bbnode);

    // Debug
    void print_paths() const;
    void print_dual_sol() const;
#ifdef DEBUG
    Float calculate_reduced_cost(const Agent a, const Path& path);
    void set_verbose(const Bool verbose = true) { lp_.set_verbose(verbose); }
#endif

  private:
    // Views of variables and constraints
    auto all_variables()
    {
        return all_variables_ |
               Ranges::views::transform([](auto& ptr) -> Variable& { return *ptr; });
    }
    auto all_constraints()
    {
        return all_constraints_ |
               Ranges::views::transform([](auto& ptr) -> Constraint& { return *ptr; });
    }
    auto secondary_variables_storage()
    {
        return Ranges::subrange(agent_variables_, agent_variables_ + instance_.agents.size());
    }
    auto secondary_constraints_storage()
    {
        return Ranges::views::concat(
            Ranges::subrange(&universal_constraints_, &universal_constraints_ + 1),
            Ranges::subrange(&subset_constraints_, &subset_constraints_ + 1),
            Ranges::subrange(agent_constraints_, agent_constraints_ + instance_.agents.size())
        );
    }

    // Solve
    void restore_column(Variable* variable_ptr);
    void restore_row(Constraint* constraint_ptr);
    MasterProblemStatus calculate_master_status();
    void accumulate_activity();
};
