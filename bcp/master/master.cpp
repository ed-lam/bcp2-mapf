/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "master/master.h"
#include "constraints/separator.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/debug.h"
#include "types/float_compare.h"
#include "types/pointers.h"
#include "types/tracy.h"
#include <fmt/color.h>

#define DELETE_COLUMN_MIN_ACTIVITY (0.8)
#define DELETE_ROW_MIN_ACTIVITY (0.8)
#define DELETE_COLUMN_DECAY (0.99)
#define DELETE_ROW_DECAY (0.99)

#define TRACY_COLOUR tracy::Color::ColorType::Goldenrod

MasterProblem::MasterProblem(const Instance& instance, Problem& problem) :
    instance_(instance),
    problem_(problem),

    clock_(),
    lp_(),
    status_(MasterProblemStatus::Unknown),
    vars_added_(0),
    constrs_added_(0),

    variable_storage_(),
    all_variables_(),
    agent_variables_(nullptr),

    constraint_storage_(),
    all_constraints_(),
    universal_constraints_(),
    subset_constraints_(),
    agent_constraints_(nullptr),

    temp_buffer_()
{
    ZoneScopedC(TRACY_COLOUR);

    const auto A = instance_.num_agents();
    agent_variables_ = new Vector<Variable*>[A];
    agent_constraints_ = new Vector<Constraint*>[A];
}

MasterProblem::~MasterProblem()
{
    ZoneScopedC(TRACY_COLOUR);

    delete[] agent_variables_;
    delete[] agent_constraints_;
}

void MasterProblem::add_column(UniquePtr<Variable>&& variable_ptr_input, const Cost reduced_cost)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the existing variable if it already exists.
    auto variable_ptr = variable_storage_.add_path(std::move(variable_ptr_input));
    auto& variable = *variable_ptr;

    // Get the path.
    const auto a = variable.a();
    const auto& path = variable.path();
    DEBUG_ASSERT(path.front().n == instance_.agents[a].start);
    DEBUG_ASSERT(path.back().n == instance_.agents[a].target);

    // Handle small floating point errors.
    if (reduced_cost >= -LENIENT_EPS)
    {
        // if (variable.col() >= 0)
        // {
        //     PRINTLN("WARNING: Skipping adding path {} for agent {} with reduced cost {} which "
        //             "already exists with ID {}, value {} and domain [{},{}]",
        //             format_path_with_time(path, instance_.map),
        //             a,
        //             reduced_cost,
        //             variable.id(),
        //             variable_primal_sol(variable),
        //             lp_.col_lb(variable.col()),
        //             lp_.col_ub(variable.col()));
        // }
        // else
        // {
        //     PRINTLN("WARNING: Skipping adding path {} for agent {} with reduced cost {} ",
        //             format_path_with_time(path, instance_.map),
        //             a,
        //             reduced_cost);
        // }
        return;
    }
    else if (variable.col() >= 0)
    {
        ERROR("Adding path {} for agent {} with negative reduced cost {} which already exists with "
              "ID {}, value {} and domain [{},{}]",
              format_path_with_time(path, instance_.map),
              a,
              reduced_cost,
              variable.id(),
              variable_primal_sol(variable),
              lp_.col_lb(variable.col()),
              lp_.col_ub(variable.col()));
    }

    // Calculate the cost of the column.
    const Real64 cost = path.size() - 1;

    // Create matrix coefficients.
    Vector<RowIndex> rows;
    Vector<Real64> coeffs;
#ifdef DEBUG
    Real64 check_reduced_cost = (status_ == MasterProblemStatus::Infeasible ? 0.0 : cost);
#endif
    for (const auto& constraint : agent_constraints(a))
    {
        const auto coeff = constraint.get_coeff(a, path);
        if (coeff)
        {
            rows.push_back(constraint.row());
            coeffs.push_back(coeff);
#ifdef DEBUG
            const auto dual = constraint_dual_sol(constraint);
            check_reduced_cost += coeff * -dual;
#endif
        }
    }
#ifdef DEBUG
    if (is_ne(check_reduced_cost, reduced_cost))
    {
        PRINTLN(
            "Reduced cost mismatch: calculated {}, solved {}", check_reduced_cost, reduced_cost);
        PRINTLN("Agent {}, path {}", a, format_path_with_time(path, instance_.map, ","));
        for (const auto& constraint : agent_constraints(a))
        {
            const auto coeff = constraint.get_coeff(a, path);
            if (coeff)
            {
                const auto dual = constraint_dual_sol(constraint);
                PRINTLN("    {} {} {}", constraint.name(), coeff, -dual);
            }
        }
        PRINTLN("");
    }
#endif
    DEBUG_ASSERT(is_eq(check_reduced_cost, reduced_cost));
    DEBUG_ASSERT(is_lt(check_reduced_cost, 0.0));

    // Check that the path doesn't already exist.
#ifdef DEBUG
    for (const auto& existing_var : agent_variables(a))
    {
        const auto& existing_path = existing_var.path();
        ASSERT(!std::equal(path.begin(), path.end(), existing_path.begin(), existing_path.end()),
               "Path {} for agent {} already exists with ID {}, value {} and domain [{},{}]",
               format_path_with_time(path, instance_.map),
               a,
               existing_var.id(),
               variable_primal_sol(existing_var),
               lp_.col_lb(existing_var.col()),
               lp_.col_ub(existing_var.col()));
    }
#endif

    // Create the name.
    // const auto name = fmt::format("path({},{})", a, format_path(path, instance_.map, ","));
    const auto name = fmt::format("x({})", variable.id());

    // Add the column to the LP.
    DEBUG_ASSERT(variable.col_ == -1);
    variable.col_ = lp_.add_column(rows, coeffs, cost, name);
    variable.activity_ = std::max(variable.activity_, 1.0);
    ++vars_added_;

    // Store the variable in the list of all variables.
    all_variables_.emplace_back(variable_ptr);

    // Store the variable in the list of variables partitioned by agent.
    agent_variables_[a].emplace_back(variable_ptr);
}

void MasterProblem::restore_column(Variable* variable_ptr)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the path.
    auto& variable = *variable_ptr;
    const auto a = variable.a();
    const auto& path = variable.path();
    DEBUG_ASSERT(variable.col() == -1);
    DEBUG_ASSERT(path.front().n == instance_.agents[a].start);
    DEBUG_ASSERT(path.back().n == instance_.agents[a].target);

    // Calculate the cost of the column.
    const Real64 cost = path.size() - 1;

    // Create matrix coefficients.
    Vector<RowIndex> rows;
    Vector<Real64> coeffs;
    for (const auto& constraint : agent_constraints(a))
    {
        const auto coeff = constraint.get_coeff(a, path);
        if (coeff)
        {
            rows.push_back(constraint.row());
            coeffs.push_back(coeff);
        }
    }

    // Check that the path doesn't already exist.
#ifdef DEBUG
    for (const auto& existing_var : agent_variables(a))
    {
        const auto& existing_path = existing_var.path();
        ASSERT(!std::equal(path.begin(), path.end(), existing_path.begin(), existing_path.end()),
               "Path {} for agent {} already exists with ID {}, value {} and domain [{},{}]",
               format_path_with_time(path, instance_.map),
               a,
               existing_var.id(),
               variable_primal_sol(existing_var),
               lp_.col_lb(existing_var.col()),
               lp_.col_ub(existing_var.col()));
    }
#endif

    // Create the name.
    // const auto name = fmt::format("path({},{})", a, format_path(path, instance_.map, ","));
    const auto name = fmt::format("x({})", variable.id());

    // Add the column to the LP.
    DEBUG_ASSERT(variable.col_ == -1);
    variable.col_ = lp_.add_column(rows, coeffs, cost, name);
    // Activity is incremented in the calling function.
    ++vars_added_;

    // Store the variable in the list of all variables.
    all_variables_.emplace_back(variable_ptr);

    // Store the variable in the list of variables partitioned by agent.
    agent_variables_[a].emplace_back(variable_ptr);
}

void MasterProblem::add_permanent_row(UniquePtr<Constraint>&& constraint_ptr)
{
    // Mark as non-removable.
    constraint_ptr->activity_ = COST_INF;

    // Create the row.
    add_row(std::move(constraint_ptr));
}

void MasterProblem::add_row(UniquePtr<Constraint>&& constraint_ptr_input)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the existing constraint if it already exists.
    auto constraint_ptr = constraint_storage_.add_constraint(std::move(constraint_ptr_input));
    auto& constraint = *constraint_ptr;

    // Get the constraint data.
    const auto& name = constraint.name();
    const auto sign = constraint.sign();
    const auto rhs = constraint.rhs();
    DEBUG_ASSERT(constraint.row() == -1);

    // Determine which agents that the constraint covers.
    const auto& agents = constraint.agents();
    const auto is_universal = agents.empty();
    const auto is_every_agent = (!is_universal && agents[0] == -1);

    // Create matrix coefficients.
    Vector<ColumnIndex> cols;
    Vector<Real64> coeffs;
#ifdef DEBUG
    Real64 lhs = 0.0;
#endif
    if (is_universal || is_every_agent)
    {
        const auto A = instance_.num_agents();
        for (Agent a = 0; a < A; ++a)
            for (const auto& variable : agent_variables(a))
            {
                const auto coeff = constraint.get_coeff(a, variable.path());
                if (coeff)
                {
                    cols.push_back(variable.col());
                    coeffs.push_back(coeff);

                // PRINTLN("Agent {} coeff {} value {} path {}",
                //         a,
                //         coeff,
                //         lp_.get_primal_sol(variable.col()),
                //         format_path_with_time(variable.path(), instance_.map));

#ifdef DEBUG
                    const auto val = lp_.get_primal_sol(variable.col());
                    lhs += coeff * val;
#endif
                }
            }
    }
    else
    {
        for (const auto a : agents)
            for (const auto& variable : agent_variables(a))
            {
                const auto coeff = constraint.get_coeff(a, variable.path());
                if (coeff)
                {
                    cols.push_back(variable.col());
                    coeffs.push_back(coeff);
#ifdef DEBUG
                    const auto val = lp_.get_primal_sol(variable.col());
                    lhs += coeff * val;
#endif
                }
            }
    }

    // Check violation.
    DEBUG_ASSERT((sign == '<' && is_gt(lhs, rhs)) || (sign == '>' && is_lt(lhs, rhs)) ||
                 (sign == '=' && (is_lt(lhs, rhs) || is_gt(lhs, rhs))));

    // Add the row to the LP.
    DEBUG_ASSERT(constraint.row_ == -1);
    constraint.row_ = lp_.add_row(cols, coeffs, sign, rhs, name);
    constraint.activity_ = std::max(constraint.activity_, 1.0);
    ++constrs_added_;

    // Store the constraint in the list of all constraints.
    all_constraints_.emplace_back(constraint_ptr);

    // Store the constraint elsewhere.
    if (is_universal || is_every_agent)
    {
        if (is_universal)
        {
            universal_constraints_.emplace_back(constraint_ptr);
        }
        else
        {
            subset_constraints_.emplace_back(constraint_ptr);
        }

        const auto A = instance_.num_agents();
        for (Agent a = 0; a < A; ++a)
        {
            agent_constraints_[a].emplace_back(constraint_ptr);
        }
    }
    else
    {
        subset_constraints_.emplace_back(constraint_ptr);

        for (const auto a : agents)
        {
            agent_constraints_[a].emplace_back(constraint_ptr);
        }
    }
}

void MasterProblem::restore_row(Constraint* constraint_ptr)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    auto& constraint = *constraint_ptr;
    const auto& name = constraint.name();
    const auto sign = constraint.sign();
    const auto rhs = constraint.rhs();
    DEBUG_ASSERT(constraint.row() == -1);

    // Determine which agents that the constraint covers.
    const auto& agents = constraint.agents();
    const auto is_universal = agents.empty();
    const auto is_every_agent = (!is_universal && agents[0] == -1);

    // Create matrix coefficients.
    Vector<ColumnIndex> cols;
    Vector<Real64> coeffs;
    if (is_universal || is_every_agent)
    {
        const auto A = instance_.num_agents();
        for (Agent a = 0; a < A; ++a)
            for (const auto& variable : agent_variables(a))
            {
                const auto coeff = constraint.get_coeff(a, variable.path());
                if (coeff)
                {
                    cols.push_back(variable.col());
                    coeffs.push_back(coeff);
                }
            }
    }
    else
    {
        for (const auto a : agents)
            for (const auto& variable : agent_variables(a))
            {
                const auto coeff = constraint.get_coeff(a, variable.path());
                if (coeff)
                {
                    cols.push_back(variable.col());
                    coeffs.push_back(coeff);
                }
            }
    }

    // Add the row to the LP.
    DEBUG_ASSERT(constraint.row_ == -1);
    constraint.row_ = lp_.add_row(cols, coeffs, sign, rhs, name);
    // Activity is incremented in the calling function.
    ++constrs_added_;

    // Store the constraint in the list of all constraints.
    all_constraints_.emplace_back(constraint_ptr);

    // Store the constraint elsewhere.
    if (is_universal || is_every_agent)
    {
        if (is_universal)
        {
            universal_constraints_.emplace_back(constraint_ptr);
        }
        else
        {
            subset_constraints_.emplace_back(constraint_ptr);
        }

        const auto A = instance_.num_agents();
        for (Agent a = 0; a < A; ++a)
        {
            agent_constraints_[a].emplace_back(constraint_ptr);
        }
    }
    else
    {
        subset_constraints_.emplace_back(constraint_ptr);

        for (const auto a : agents)
        {
            agent_constraints_[a].emplace_back(constraint_ptr);
        }
    }
}

void MasterProblem::solve()
{
    ZoneScopedC(TRACY_COLOUR);

    // Exit if timed out.
    problem_.stop_if_timed_out();

    // Start timer.
    auto timer = clock_.start_timer();

    // Write model to file.
    // lp_.write();

    // Choose a simplex algorithm.
    const auto algorithm =
        vars_added_ && !constrs_added_ ? LPAlgorithm::PrimalSimplex : LPAlgorithm::DualSimplex;

    // Solve the model.
    const auto lp_status = lp_.solve(algorithm, problem_.time_remaining());
    problem_.stop_if_timed_out();

    // Get the status and solution.
    switch (lp_status)
    {
        case LPStatus::Infeasible:
            status_ = MasterProblemStatus::Infeasible;
            break;
        case LPStatus::Optimal:
            status_ = calculate_master_status();
            accumulate_activity();
            break;
        default:
            PRINTLN("WARNING: LP timed out before time limit");
            throw TerminationException();
    }

#ifdef PRINT_DEBUG
    // Print dual solution.
    print_dual_sol();
#endif

    // Reset the counter of variables and constraints added since the last time that the master
    // problem is solved.
    vars_added_ = 0;
    constrs_added_ = 0;
}

void MasterProblem::accumulate_activity()
{
    ZoneScopedC(TRACY_COLOUR);

    // Update activity of variables.
    for (auto& variable : all_variables())
    {
        const auto col = variable.col();
        // const auto val = lp_.get_primal_sol(col);
        // const auto active = is_gt(val, 0.0);
        const auto basis = lp_.get_col_basis(col);
        const auto active = (basis != DEFAULT_COLUMN_BASIS);
        // const auto active = lp_.col_is_basic(col);
        variable.activity_ = variable.activity_ * DELETE_COLUMN_DECAY + active;
    }

    // Update activity of constraints.
    for (auto& constraint : all_constraints())
    {
        const auto row = constraint.row();
        // const auto slack = lp_.get_slack(row);
        // const auto active = is_eq(slack, 0.0);
        const auto basis = lp_.get_row_basis(row);
        const auto active = (basis != DEFAULT_ROW_BASIS);
        // const auto active = lp_.row_is_basic(row);
        constraint.activity_ = constraint.activity_ * DELETE_ROW_DECAY + active;
    }
}

void MasterProblem::start_node(BBNode& bbnode)
{
    ZoneScopedC(TRACY_COLOUR);

    // Reset status.
    status_ = MasterProblemStatus::Unknown;

    // Set domain upper bounds of all variables to infinity.
    lp_.enable_columns();

    // Add back variables and constraints that were previously deleted.
    for (auto& [variable_ptr, basis] : bbnode.variables)
    {
        auto& variable = *variable_ptr;
        const auto col = variable.col();
        if (col < 0)
        {
            restore_column(variable_ptr);
        }
        variable.activity_ = std::max(variable.activity_, 1.0);
        DEBUG_ASSERT(variable.activity() >= DELETE_COLUMN_MIN_ACTIVITY);
    }
    for (auto& [constraint_ptr, basis] : bbnode.constraints)
    {
        auto& constraint = *constraint_ptr;
        const auto row = constraint.row();
        if (row < 0)
        {
            restore_row(constraint_ptr);
        }
        constraint.activity_ = std::max(constraint.activity_, 1.0);
        DEBUG_ASSERT(constraint.activity() >= DELETE_ROW_MIN_ACTIVITY);
    }

    // Update the LP.
    lp_.update();

    // Delete variables with low activity.
#ifdef DEBUG
    Size64 num_cols_deleted = 0;
#endif
    {
        auto& delete_cols = temp_buffer_;
        DEBUG_ASSERT(delete_cols.empty());
        Size64 write = 0;
        for (Size64 read = 0; read < all_variables_.size(); ++read)
        {
            // Get the variable data.
            auto& variable = *all_variables_[read];
            const auto activity = variable.activity();
            DEBUG_ASSERT(fmt::format("x({})", variable.id()) == lp_.col_name(variable.col()));

            // Delete if the variable has been inactive for a while.
            if (activity < DELETE_COLUMN_MIN_ACTIVITY)
            {
                delete_cols.push_back(variable.col());
                variable.col_ = -1;
                continue;
            }

            // Move the kept variable to the write slot. Update its self-index.
            all_variables_[write] = all_variables_[read];
            all_variables_[write]->col_ = write;

            // Advance the write index.
            ++write;
        }
#ifdef DEBUG
        num_cols_deleted = delete_cols.size();
#endif
        all_variables_.erase(all_variables_.begin() + write, all_variables_.end());
        lp_.delete_columns(delete_cols);
        delete_cols.clear();
    }

    // Delete constraints with low activity.
    {
        auto& delete_rows = temp_buffer_;
        DEBUG_ASSERT(delete_rows.empty());
        Size64 write = 0;
        for (Size64 read = 0; read < all_constraints_.size(); ++read)
        {
            // Get the constraint data.
            auto& constraint = *all_constraints_[read];
            const auto activity = constraint.activity();
            DEBUG_ASSERT(constraint.name() == lp_.row_name(constraint.row()));

            // Delete if the constraint has been inactive for a while.
            if (activity < DELETE_ROW_MIN_ACTIVITY)
            {
                delete_rows.push_back(constraint.row());
                constraint.row_ = -1;
                continue;
            }

            // Move the kept constraint to the write slot. Update its self-index.
            all_constraints_[write] = all_constraints_[read];
            all_constraints_[write]->row_ = write;

            // Advance the write index.
            ++write;
        }
        all_constraints_.erase(all_constraints_.begin() + write, all_constraints_.end());
        lp_.delete_rows(delete_rows);
        delete_rows.clear();
    }

    // Update the LP.
    lp_.update();

    // Clean up.
    {
        // Clean up pointers to deleted variables.
#ifdef DEBUG
        Size64 check_num_cols_deleted = 0;
#endif
        for (auto& variables_set : secondary_variables_storage())
            for (Size64 index = 0; index < variables_set.size();)
            {
                auto& variable_ptr = variables_set[index];
                const auto& variable = *variable_ptr;
                if (variable.col() < 0)
                {
                    std::swap(variable_ptr, variables_set.back());
                    variables_set.pop_back();
#ifdef DEBUG
                    ++check_num_cols_deleted;
#endif
                }
                else
                {
                    ++index;
                }
            }
        DEBUG_ASSERT(check_num_cols_deleted == num_cols_deleted);

        // Clean up pointers to deleted constraints.
        for (auto& constraints_set : secondary_constraints_storage())
            for (Size64 index = 0; index < constraints_set.size();)
            {
                auto& constraint_ptr = constraints_set[index];
                const auto& constraint = *constraint_ptr;
                if (constraint.row() < 0)
                {
                    std::swap(constraint_ptr, constraints_set.back());
                    constraints_set.pop_back();
                }
                else
                {
                    ++index;
                }
            }
    }

    // Set up the basis.
    {
        // Input the basis to the LP.
        Vector<ColumnBasis> variable_basis(lp_.num_cols(), DEFAULT_COLUMN_BASIS);
        Vector<RowBasis> constraint_basis(lp_.num_rows(), DEFAULT_ROW_BASIS);
        for (const auto& [variable_ptr, basis] : bbnode.variables)
        {
            const auto& variable = *variable_ptr;
            const auto col = variable.col();
            DEBUG_ASSERT(0 <= col && col < variable_basis.size());
            variable_basis[col] = basis;
        }
        for (const auto& [constraint_ptr, basis] : bbnode.constraints)
        {
            const auto& constraint = *constraint_ptr;
            const auto row = constraint.row();
            DEBUG_ASSERT(0 <= row && row < constraint_basis.size());
            constraint_basis[row] = basis;
        }
        bbnode.variables.clear();
        bbnode.constraints.clear();
        lp_.set_col_basis(variable_basis);
        lp_.set_row_basis(constraint_basis);
    }

    // Disable variables incompatible with branching decisions.
    {
        // Get the variables.
        auto& disable_cols = temp_buffer_;
        DEBUG_ASSERT(disable_cols.empty());
        for (auto node = &bbnode; node && node->brancher; node = node->parent.get())
        {
            const auto brancher = node->brancher;
            const auto decision = node->decision.get();
            brancher->disable_vars(decision);
        }

        // Disable the variables.
        if (!disable_cols.empty())
        {
            // Remove duplicates.
            std::sort(disable_cols.begin(), disable_cols.end());
            auto last = std::unique(disable_cols.begin(), disable_cols.end());
            disable_cols.erase(last, disable_cols.end());

            // Disable the columns.
            lp_.disable_columns(disable_cols);

            // Clear.
            disable_cols.clear();
        }
    }
}

void MasterProblem::disable_variable(const Variable& variable)
{
    ZoneScopedC(TRACY_COLOUR);

    // Store the variable to disable.
    temp_buffer_.push_back(variable.col());
    ++constrs_added_;
}

void MasterProblem::store_basis(BBNode& node)
{
    ZoneScopedC(TRACY_COLOUR);

    // Copy variables, constraints and basis to the node.
    DEBUG_ASSERT(node.variables.empty());
    DEBUG_ASSERT(node.constraints.empty());
    for (auto& variable : all_variables())
    {
        // Get the variable data.
        const auto basis = variable_basis(variable);
        DEBUG_ASSERT(fmt::format("x({})", variable.id()) == lp_.col_name(variable.col()));
        DEBUG_ASSERT(basis == lp_.col_basis(variable.col()));

        // Store the variable in the node's basis.
        if (basis != DEFAULT_COLUMN_BASIS)
        {
            node.variables.emplace_back(&variable, basis);
        }
        else
        {
            DEBUG_ASSERT(is_eq(variable_primal_sol(variable), 0.0));
        }
    }
    for (auto& constraint : all_constraints())
    {
        // Get the constraint data.
        const auto basis = constraint_basis(constraint);
        DEBUG_ASSERT(constraint.name() == lp_.row_name(constraint.row()));
        DEBUG_ASSERT(basis == lp_.row_basis(constraint.row()));

        // Store the constraint in the node's basis.
        if (basis != DEFAULT_ROW_BASIS)
        {
            node.constraints.emplace_back(&constraint, basis);
        }
    }
}

MasterProblemStatus MasterProblem::calculate_master_status()
{
    ZoneScopedC(TRACY_COLOUR);

    // Check if any variable has fractional value.
    auto status = MasterProblemStatus::Integer;
    for (ColumnIndex col = 0; col < lp_.num_cols(); ++col)
        if (!is_integral(lp_.get_primal_sol(col)))
        {
            status = MasterProblemStatus::Fractional;
            break;
        }
    return status;
}

void MasterProblem::print_paths() const
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();
    const auto& map = instance_.map;

    // Find the makespan.
    Time makespan = 0;
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : agent_variables(a))
        {
            const auto val = variable_primal_sol(variable);
            if (is_gt(val, 0.0))
            {
                const auto& path = variable.path();
                makespan = std::max<Time>(makespan, path.size());
            }
        }

    // Get fractional edges and determine if the solution is fractional.
    // Bool is_fractional = false;
    HashMap<NodeTime, HashMap<Agent, Real64>> used_nodetimes;
    HashMap<EdgeTime, HashMap<Agent, Real64>> used_edgetimes;
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : agent_variables(a))
        {
            // Store fractional status.
            const auto val = variable_primal_sol(variable);
            // is_fractional |= !is_integral(val);

            // Store used arcs and edges.
            if (is_gt(val, 0.0))
            {
                const auto& path = variable.path();
                Time t = 0;
                for (; t < path.size() - 1; ++t)
                {
                    const NodeTime nt{path[t].n, t};
                    used_nodetimes[nt][a] += val;

                    const EdgeTime et{map.get_undirected_edge(path[t]), t};
                    used_edgetimes[et][a] += val;
                }
                const auto n = path[t].n;
                for (; t < makespan; ++t)
                {
                    const NodeTime nt{n, t};
                    used_nodetimes[nt][a] += val;
                }
            }
        }

    // Print time header line.
    fmt::print("                                          ");
    for (Time t = 0; t < makespan; ++t)
    {
        fmt::print("{:>10d}", t);
    }
    PRINTLN("");

    // Print paths.
    for (Agent a = 0; a < A; ++a)
        for (const auto& variable : agent_variables(a))
        {
            // if ((is_fractional && !is_integral(val)) || (!is_fractional && is_gt(val, 0.0)))
            const auto val = variable_primal_sol(variable);
            if (is_gt(val, 0.0))
            {
                // Print value.
                if (is_integral(val))
                {
                    fmt::print("    ");
                }
                else
                {
                    fmt::print("   *");
                }
                fmt::print(
                    "Agent {:3d}, ID {:6d}, Val {:6.4f}, Path", a, variable.id(), std::abs(val));

                // Print path.
                const auto& path = variable.path();
                Bool prev_edgetime_is_fractional = false;
                Time t = 0;
                for (; t < path.size() - 1; ++t)
                {
                    // Get the nodetime and edgetime.
                    const auto [x, y] = map.get_xy(path[t].n);
                    const NodeTime nt{path[t].n, t};
                    const EdgeTime et{map.get_undirected_edge(path[t]), t};

                    // Check if fractional.
                    auto nodetime_is_fractional = false;
                    if (auto it1 = used_nodetimes.find(nt); it1 != used_nodetimes.end())
                    {
                        const auto& agents = it1->second;
                        if (agents.size() > 1)
                        {
                            nodetime_is_fractional = true;
                        }
                        // else if (auto it2 = agents.find(a);
                        //          it2 != agents.end() && !is_integral(it2->second))
                        // {
                        //     nodetime_is_fractional = true;
                        // }
                    }
                    auto edgetime_is_fractional = false;
                    if (auto it1 = used_edgetimes.find(et); it1 != used_edgetimes.end())
                    {
                        const auto& agents = it1->second;
                        if (agents.size() > 1)
                        {
                            edgetime_is_fractional = true;
                        }
                        // else if (auto it2 = agents.find(a);
                        //          it2 != agents.end() && !is_integral(it2->second))
                        // {
                        //     edgetime_is_fractional = true;
                        // }
                    }

                    // Print.
                    if (nodetime_is_fractional)
                    {
                        fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::blue),
                                   "{:>10}",
                                   fmt::format("({},{})", x, y));
                    }
                    else if (edgetime_is_fractional || prev_edgetime_is_fractional)
                    {
                        fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::red),
                                   "{:>10}",
                                   fmt::format("({},{})", x, y));
                    }
                    else
                    {
                        fmt::print("{:>10}", fmt::format("({},{})", x, y));
                    }
                    prev_edgetime_is_fractional = edgetime_is_fractional;
                }
                {
                    // Get the nodetime.
                    const auto [x, y] = map.get_xy(path[t].n);
                    const NodeTime nt{path[t].n, t};

                    // Check if fractional.
                    auto nodetime_is_fractional = false;
                    if (auto it1 = used_nodetimes.find(nt); it1 != used_nodetimes.end())
                    {
                        const auto& agents = it1->second;
                        if (agents.size() > 1)
                        {
                            nodetime_is_fractional = true;
                        }
                        else if (auto it2 = agents.find(a);
                                 it2 != agents.end() && !is_integral(it2->second))
                        {
                            nodetime_is_fractional = true;
                        }
                    }

                    // Print.
                    if (nodetime_is_fractional)
                    {
                        fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::blue),
                                   "{:>10}",
                                   fmt::format("({},{})", x, y));
                    }
                    else if (prev_edgetime_is_fractional)
                    {
                        fmt::print(fmt::emphasis::bold | fg(fmt::terminal_color::red),
                                   "{:>10}",
                                   fmt::format("({},{})", x, y));
                    }
                    else
                    {
                        fmt::print("{:>10}", fmt::format("({},{})", x, y));
                    }
                }
                PRINTLN("");
            }
        }
}

void MasterProblem::print_dual_sol() const
{
    PRINTLN("Dual solution:");
    for (const auto& constraint : all_constraints())
    {
        const auto dual = constraint_dual_sol(constraint);
        if (is_ne(dual, 0.0))
        {
            const auto& name = constraint.name();
            PRINTLN("    {}: {:.6f}", name, dual);
        }
    }
}

#ifdef DEBUG

Real64 MasterProblem::calculate_reduced_cost(const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check the start and target.
    DEBUG_ASSERT(path.front().n == instance_.agents[a].start);
    DEBUG_ASSERT(path.back().n == instance_.agents[a].target);

    // Calculate the cost of the column.
    const Real64 cost = path.size() - 1;

    // Calculate the reduced cost.
    Real64 check_reduced_cost = (status_ == MasterProblemStatus::Infeasible ? 0.0 : cost);
    for (const auto& constraint : agent_constraints(a))
    {
        const auto coeff = constraint.get_coeff(a, path);
        if (coeff)
        {
            const auto dual = constraint_dual_sol(constraint);
            check_reduced_cost += coeff * -dual;
            // PRINTLN("    {} {} {}", constraint.name(), coeff, -dual);
        }
    }

    // Check that the path doesn't already exist.
    for (const auto& existing_var : agent_variables(a))
    {
        const auto& existing_path = existing_var.path();
        if (std::equal(path.begin(), path.end(), existing_path.begin(), existing_path.end()))
        {
            PRINTLN("Path {} for agent {} already exists with ID {}, value {} and domain [{},{}]",
                    format_path_with_time(path, instance_.map),
                    a,
                    existing_var.id(),
                    variable_primal_sol(existing_var),
                    lp_.col_lb(existing_var.col()),
                    lp_.col_ub(existing_var.col()));
        }
    }

    // Done.
    return check_reduced_cost;
}

#endif
