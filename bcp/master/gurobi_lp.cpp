/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "master/gurobi_lp.h"
#include "types/debug.h"
#include "types/float_compare.h"
#include "types/tracy.h"
#include <algorithm>

#define TRACY_COLOUR tracy::Color::ColorType::Gold

GurobiLP::GurobiLP() :
    env_(nullptr),
    model_(nullptr),
    num_cols_(0),
    num_rows_(0),

    obj_(COST_INF),
    primal_sol_(),
    dual_sol_(),
    row_slack_(),
    col_basis_(),
    row_basis_(),

    temp_infinity_(),
    temp_zero_()
{
    ZoneScopedC(TRACY_COLOUR);

    // Create environment.
    auto error = GRBloadenv(&env_, nullptr);
    ASSERT(!error, "Gurobi failed to create environment (error {})", error);

    // Turn off logging.
    error = GRBsetintparam(env_, GRB_INT_PAR_OUTPUTFLAG, 0);
    ASSERT(!error, "Gurobi failed to turn off logging (error {})", error);

    // Set number of threads.
    error = GRBsetintparam(env_, GRB_INT_PAR_THREADS, 1);
    ASSERT(!error, "Gurobi failed to set single thread (error {})", error);

    // Set to compute the Farkas certificate of infeasibility.
    error = GRBsetintparam(env_, GRB_INT_PAR_INFUNBDINFO, 1);
    ASSERT(!error, "Gurobi failed to turn on infeasibility proof (error {})", error);

    // Enable presolve when warm-starting from LP basis.
    // error = GRBsetintparam(env_, GRB_INT_PAR_LPWARMSTART, 2);
    // ASSERT(!error, "Gurobi failed to turn on warm-start presolving (error {})", error);

    // Create an empty model.
    error = GRBnewmodel(env_, &model_, "master", 0, nullptr, nullptr, nullptr, nullptr, nullptr);
    ASSERT(!error, "Gurobi failed to create model (error {})", error);
};

GurobiLP::~GurobiLP()
{
    ZoneScopedC(TRACY_COLOUR);

    // Free model.
    const auto error = GRBfreemodel(model_);
    ASSERT(!error, "Gurobi failed to free model (error {})", error);

    // Free environment.
    GRBfreeenv(env_);
}

Value GurobiLP::col_lb(const ColumnIndex col) const
{
    ZoneScopedC(TRACY_COLOUR);

    Value val;
    const auto error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_LB, col, &val);
    ASSERT(!error, "Gurobi failed to get variable domain lower bound (error {})", error);
    return val;
}

Value GurobiLP::col_ub(const ColumnIndex col) const
{
    ZoneScopedC(TRACY_COLOUR);

    Value val;
    const auto error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_UB, col, &val);
    ASSERT(!error, "Gurobi failed to get variable domain upper bound (error {})", error);
    return val;
}

Real64 GurobiLP::condition_number() const
{
    ZoneScopedC(TRACY_COLOUR);

    Real64 kappa;
    const auto error = GRBgetdblattr(model_, GRB_DBL_ATTR_KAPPA_EXACT, &kappa);
    ASSERT(!error, "Gurobi failed to get condition number (error {})", error);
    return kappa;
}

Real64 GurobiLP::approx_condition_number() const
{
    ZoneScopedC(TRACY_COLOUR);

    Real64 kappa;
    const auto error = GRBgetdblattr(model_, GRB_DBL_ATTR_KAPPA, &kappa);
    ASSERT(!error, "Gurobi failed to get approximate condition number (error {})", error);
    return kappa;
}

#ifdef DEBUG
String GurobiLP::col_name(const ColumnIndex col) const
{
    ZoneScopedC(TRACY_COLOUR);

    char* name;
    const auto error = GRBgetstrattrelement(model_, GRB_STR_ATTR_VARNAME, col, &name);
    ASSERT(!error, "Gurobi failed to get variable name (error {})", error);

    String val{name};
    return val;
}

String GurobiLP::row_name(const RowIndex row) const
{
    ZoneScopedC(TRACY_COLOUR);

    char* name;
    const auto error = GRBgetstrattrelement(model_, GRB_STR_ATTR_CONSTRNAME, row, &name);
    ASSERT(!error, "Gurobi failed to get constraint name (error {})", error);

    String val{name};
    return val;
}

ColumnBasis GurobiLP::col_basis(const ColumnIndex col) const
{
    ZoneScopedC(TRACY_COLOUR);

    ColumnBasis basis;
    const auto error = GRBgetintattrelement(model_, GRB_INT_ATTR_VBASIS, col, &basis);
    ASSERT(!error, "Gurobi failed to get variable basis (error {})", error);
    return basis;
}

RowBasis GurobiLP::row_basis(const RowIndex row) const
{
    ZoneScopedC(TRACY_COLOUR);

    RowBasis basis;
    const auto error = GRBgetintattrelement(model_, GRB_INT_ATTR_CBASIS, row, &basis);
    ASSERT(!error, "Gurobi failed to get constraint basis (error {})", error);
    return basis;
}
#endif

ColumnIndex GurobiLP::add_column(const Span<RowIndex>& rows, const Span<Value>& coeffs,
                                 const Cost obj, const String& name)
{
    ZoneScopedC(TRACY_COLOUR);

    // Add the column to the model.
    const auto error = GRBaddvar(model_,
                                 rows.size(),
                                 const_cast<RowIndex*>(rows.data()),
                                 const_cast<Value*>(coeffs.data()),
                                 obj,
                                 0.0,
                                 GRB_INFINITY,
                                 GRB_CONTINUOUS,
                                 name.c_str());
    ASSERT(!error, "Gurobi failed to add variable {} (error {})", name, error);

    // Resize data structures.
    primal_sol_.push_back(0.0);
    col_basis_.push_back(DEFAULT_COLUMN_BASIS);

    // Return column number of new variable.
    return num_cols_++;
}

RowIndex GurobiLP::add_row(const Span<ColumnIndex>& cols, const Span<Value>& coeffs,
                           const Sign sign, const Real64 rhs, const String& name)
{
    ZoneScopedC(TRACY_COLOUR);

    // Add the row to the model.
    const auto error = GRBaddconstr(model_,
                                    cols.size(),
                                    const_cast<ColumnIndex*>(cols.data()),
                                    const_cast<Value*>(coeffs.data()),
                                    sign,
                                    rhs,
                                    name.c_str());
    ASSERT(!error, "Gurobi failed to add constraint {} (error {})", name, error);

    // Resize data structures.
    dual_sol_.push_back(0.0);
    row_slack_.push_back(0.0);
    row_basis_.push_back(DEFAULT_ROW_BASIS);

    // Return row number of new constraint.
    return num_rows_++;
}

void GurobiLP::enable_columns()
{
    ZoneScopedC(TRACY_COLOUR);

    temp_infinity_.resize(std::max<Size64>(temp_infinity_.size(), num_cols_), GRB_INFINITY);
    const auto error =
        GRBsetdblattrarray(model_, GRB_DBL_ATTR_UB, 0, num_cols_, temp_infinity_.data());
    ASSERT(!error, "Gurobi failed to reset variable domains (error {})", error);
}

void GurobiLP::disable_columns(const Span<ColumnIndex>& cols)
{
    ZoneScopedC(TRACY_COLOUR);

    temp_zero_.resize(std::max<Size64>(temp_zero_.size(), cols.size()), 0.0);
    const auto error =
        GRBsetdblattrlist(model_, GRB_DBL_ATTR_UB, cols.size(), cols.data(), temp_zero_.data());
    ASSERT(!error, "Gurobi failed to fix variable values to 0 (error {})", error);
}

void GurobiLP::delete_columns(const Span<ColumnIndex>& cols)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check the number of variables.
#ifdef DEBUG
    {
        ColumnIndex num_cols = 0;
        const auto error = GRBgetintattr(model_, GRB_INT_ATTR_NUMVARS, &num_cols);
        ASSERT(
            !error, "Gurobi failed to check number of variables before deletion (error {})", error);
        DEBUG_ASSERT(num_cols == num_cols_);
    }
#endif

    // Nothing to do if no columns are to be erased.
    if (cols.empty())
    {
        return;
    }

    // Delete the columns from the model.
    const auto error = GRBdelvars(model_, cols.size(), const_cast<ColumnIndex*>(cols.data()));
    ASSERT(!error, "Gurobi failed to delete variables (error {})", error);

    // Delete solution data.
    DEBUG_ASSERT(primal_sol_.size() == num_cols_);
    DEBUG_ASSERT(col_basis_.size() == num_cols_);
    num_cols_ -= cols.size();
    for (const auto col : cols)
    {
        primal_sol_[col] = std::numeric_limits<Value>::max();
        col_basis_[col] = std::numeric_limits<ColumnBasis>::max();
    }
    std::erase(primal_sol_, std::numeric_limits<Value>::max());
    std::erase(col_basis_, std::numeric_limits<ColumnBasis>::max());
    DEBUG_ASSERT(primal_sol_.size() == num_cols_);
    DEBUG_ASSERT(col_basis_.size() == num_cols_);
}

void GurobiLP::delete_rows(const Span<RowIndex>& rows)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check the number of constraints.
#ifdef DEBUG
    {
        RowIndex num_rows = 0;
        const auto error = GRBgetintattr(model_, GRB_INT_ATTR_NUMCONSTRS, &num_rows);
        ASSERT(!error,
               "Gurobi failed to check number of constraints before deletion (error {})",
               error);
        DEBUG_ASSERT(num_rows == num_rows_);
    }
#endif

    // Nothing to do if no rows are to be erased.
    if (rows.empty())
    {
        return;
    }

    // Delete the rows from the model.
    const auto error = GRBdelconstrs(model_, rows.size(), const_cast<RowIndex*>(rows.data()));
    ASSERT(!error, "Gurobi failed to delete constraints (error {})", error);

    // Delete solution data.
    DEBUG_ASSERT(dual_sol_.size() == num_rows_);
    DEBUG_ASSERT(row_slack_.size() == num_rows_);
    DEBUG_ASSERT(row_basis_.size() == num_rows_);
    num_rows_ -= rows.size();
    for (const auto row : rows)
    {
        dual_sol_[row] = std::numeric_limits<Value>::max();
        row_slack_[row] = std::numeric_limits<Value>::max();
        row_basis_[row] = std::numeric_limits<RowBasis>::max();
    }
    std::erase(dual_sol_, std::numeric_limits<Value>::max());
    std::erase(row_slack_, std::numeric_limits<Value>::max());
    std::erase(row_basis_, std::numeric_limits<RowBasis>::max());
    DEBUG_ASSERT(dual_sol_.size() == num_rows_);
    DEBUG_ASSERT(row_slack_.size() == num_rows_);
    DEBUG_ASSERT(row_basis_.size() == num_rows_);
}

void GurobiLP::update()
{
    ZoneScopedC(TRACY_COLOUR);

    const auto error = GRBupdatemodel(model_);
    ASSERT(!error, "Gurobi failed to update model (error {})", error);
}

void GurobiLP::set_col_basis(const Span<ColumnBasis>& basis)
{
    ZoneScopedC(TRACY_COLOUR);

    const auto error = GRBsetintattrarray(
        model_, GRB_INT_ATTR_VBASIS, 0, basis.size(), const_cast<ColumnBasis*>(basis.data()));
    ASSERT(!error, "Gurobi failed to set column basis (error {})", error);
}

void GurobiLP::set_row_basis(const Span<RowBasis>& basis)
{
    ZoneScopedC(TRACY_COLOUR);

    const auto error = GRBsetintattrarray(
        model_, GRB_INT_ATTR_CBASIS, 0, basis.size(), const_cast<RowBasis*>(basis.data()));
    ASSERT(!error, "Gurobi failed to set row basis (error {})", error);
}

LPStatus GurobiLP::solve(const LPAlgorithm algorithm, const Real64 time_limit)
{
    ZoneScopedC(TRACY_COLOUR);

    // Set method/algorithm.
    auto error = GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_METHOD, static_cast<int>(algorithm));
    ASSERT(!error, "Gurobi failed to set algorithm (error {})", error);

    // Set time limit.
    error = GRBsetdblparam(GRBgetenv(model_), GRB_DBL_PAR_TIMELIMIT, time_limit + 0.5);
    ASSERT(!error, "Gurobi failed to set time limit (error {})", error);

    // Solve.
    error = GRBoptimize(model_);
    ASSERT(!error, "Gurobi failed to solve (error {})", error);

    // Get the status.
    int lp_status;
    error = GRBgetintattr(model_, GRB_INT_ATTR_STATUS, &lp_status);
    ASSERT(!error, "Gurobi failed to get solving status (error {})", error);

    // Check.
#ifdef DEBUG
    {
        // Get number of variables.
        ColumnIndex num_cols = 0;
        auto error = GRBgetintattr(model_, GRB_INT_ATTR_NUMVARS, &num_cols);
        ASSERT(!error, "Gurobi failed to get number of variables after solving (error {})", error);
        DEBUG_ASSERT(num_cols == num_cols_);

        // Get number of constraints.
        RowIndex num_rows = 0;
        error = GRBgetintattr(model_, GRB_INT_ATTR_NUMCONSTRS, &num_rows);
        ASSERT(
            !error, "Gurobi failed to get number of constraints after solving (error {})", error);
        DEBUG_ASSERT(num_rows == num_rows_);
    }
#endif

    // Get the solution.
    DEBUG_ASSERT(primal_sol_.size() == num_cols_);
    DEBUG_ASSERT(dual_sol_.size() == num_rows_);
    DEBUG_ASSERT(row_slack_.size() == num_rows_);
    DEBUG_ASSERT(col_basis_.size() == num_cols_);
    DEBUG_ASSERT(row_basis_.size() == num_rows_);
    if (lp_status == GRB_OPTIMAL)
    {
        // Store objective value.
        error = GRBgetdblattr(model_, GRB_DBL_ATTR_OBJVAL, &obj_);
        ASSERT(!error, "Gurobi failed to get objective value (error {})", error);

        // Store primal solution and zero-out near zero values.
        error = GRBgetdblattrarray(model_, GRB_DBL_ATTR_X, 0, num_cols_, primal_sol_.data());
        ASSERT(!error, "Gurobi failed to get primal solution (error {})", error);
        for (auto& x : primal_sol_)
        {
            x *= is_ne(x, 0.0);
        }

        // Store dual solution and zero-out near zero values.
        error = GRBgetdblattrarray(model_, GRB_DBL_ATTR_PI, 0, num_rows_, dual_sol_.data());
        ASSERT(!error, "Gurobi failed to get dual solution (error {})", error);
        for (auto& x : dual_sol_)
        {
            x *= is_ne(x, 0.0);
        }

        // Store the row slack values and zero-out near zero values.
        error = GRBgetdblattrarray(model_, GRB_DBL_ATTR_SLACK, 0, num_rows_, row_slack_.data());
        ASSERT(!error, "Gurobi failed to get constraint slack values (error {})", error);
        for (auto& x : row_slack_)
        {
            x *= is_ne(x, 0.0);
        }

        // Store the column basis.
        error = GRBgetintattrarray(model_, GRB_INT_ATTR_VBASIS, 0, num_cols_, col_basis_.data());
        ASSERT(!error, "Gurobi failed to get column basis (error {})", error);
#ifdef DEBUG
        for (ColumnIndex col = 0; col < num_cols_; ++col)
        {
            const auto basis = col_basis_[col];
            const auto val = primal_sol_[col];
            DEBUG_ASSERT(basis >= -2);
            DEBUG_ASSERT(basis == 0 || is_eq(val, 0.0));
        }
#endif

        // Store the row basis.
        error = GRBgetintattrarray(model_, GRB_INT_ATTR_CBASIS, 0, num_rows_, row_basis_.data());
        ASSERT(!error, "Gurobi failed to get row basis (error {})", error);
#ifdef DEBUG
        for (RowIndex row = 0; row < num_rows_; ++row)
        {
            const auto basis = row_basis_[row];
            const auto slack = row_slack_[row];
            DEBUG_ASSERT(basis == 0 || is_eq(slack, 0.0));
        }
#endif
    }
    else if (lp_status == GRB_INFEASIBLE)
    {
        // Store objective value.
        obj_ = std::numeric_limits<Value>::quiet_NaN();

        // Store primal solution.
        std::fill(primal_sol_.begin(), primal_sol_.end(), std::numeric_limits<Value>::quiet_NaN());

        // Store dual solution and zero-out near zero values. Also negate because Gurobi outputs the
        // opposite sign.
        error = GRBgetdblattrarray(model_, GRB_DBL_ATTR_FARKASDUAL, 0, num_rows_, dual_sol_.data());
        ASSERT(!error, "Gurobi failed to get Farkas certificate (error {})", error);
        for (auto& x : dual_sol_)
        {
            x *= -static_cast<Value>(is_ne(x, 0.0));
        }

        // Store the row slack values.
        std::fill(row_slack_.begin(), row_slack_.end(), 0.0);
    }

    // Return status.
    switch (lp_status)
    {
        case GRB_OPTIMAL:
            return LPStatus::Optimal;
        case GRB_UNBOUNDED:
            return LPStatus::Unbounded;
        case GRB_INFEASIBLE:
            return LPStatus::Infeasible;
        default:
            return LPStatus::Unknown;
    };
}

#ifdef DEBUG

void GurobiLP::reset()
{
    ZoneScopedC(TRACY_COLOUR);

    const auto error = GRBreset(model_, 1);
    ASSERT(!error, "Gurobi failed to reset model (error {})", error);
}

void GurobiLP::print()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the status.
    int lp_status;
    auto error = GRBgetintattr(model_, GRB_INT_ATTR_STATUS, &lp_status);
    ASSERT(!error, "Gurobi failed to get solving status for printing (error {})", error);

    // Print.
    if (lp_status == GRB_OPTIMAL)
    {
        // Get number of variables.
        ColumnIndex num_cols = 0;
        auto error = GRBgetintattr(model_, GRB_INT_ATTR_NUMVARS, &num_cols);
        ASSERT(!error, "Gurobi failed to get number of variables for printing (error {})", error);

        // Get number of constraints.
        RowIndex num_rows = 0;
        error = GRBgetintattr(model_, GRB_INT_ATTR_NUMCONSTRS, &num_rows);
        ASSERT(!error, "Gurobi failed to get number of constraints for printing (error {})", error);

        // Print columns.
        for (ColumnIndex col = 0; col < num_cols; ++col)
        {
            Value value = std::numeric_limits<Value>::quiet_NaN();
            error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_X, col, &value);
            ASSERT(!error, "Gurobi failed to get variable values for printing (error {})", error);

            Value lb = std::numeric_limits<Value>::quiet_NaN();
            error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_LB, col, &lb);
            ASSERT(!error,
                   "Gurobi failed to get variable domain lower bounds for printing (error {})",
                   error);

            Value ub = std::numeric_limits<Value>::quiet_NaN();
            error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_UB, col, &ub);
            ASSERT(!error,
                   "Gurobi failed to get variable domain upper bounds for printing (error {})",
                   error);

            char* name = nullptr;
            error = GRBgetstrattrelement(model_, GRB_STR_ATTR_VARNAME, col, &name);
            ASSERT(!error, "Gurobi failed to get variable names for printing (error {})", error);

            PRINTLN("Variable {}, value {}, lb {}, ub {}", name, value, lb, ub);
        }

        // Print rows.
        for (RowIndex row = 0; row < num_rows; ++row)
        {
            Value dual = std::numeric_limits<Value>::quiet_NaN();
            error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_PI, row, &dual);
            ASSERT(!error,
                   "Gurobi failed to get constraint dual value for printing (error {})",
                   error);

            char* name = nullptr;
            error = GRBgetstrattrelement(model_, GRB_STR_ATTR_CONSTRNAME, row, &name);
            ASSERT(!error, "Gurobi failed to get constraint name for printing (error {})", error);

            PRINTLN("Constraint {}, dual {}", name, dual);
        }
    }
    else if (lp_status == GRB_INFEASIBLE)
    {
        // Get number of constraints.
        RowIndex num_rows = 0;
        error = GRBgetintattr(model_, GRB_INT_ATTR_NUMCONSTRS, &num_rows);
        ASSERT(!error, "Gurobi failed to get number of constraints for printing (error {})", error);

        // Print rows.
        for (RowIndex row = 0; row < num_rows; ++row)
        {
            Value dual = std::numeric_limits<Value>::quiet_NaN();
            error = GRBgetdblattrelement(model_, GRB_DBL_ATTR_FARKASDUAL, row, &dual);
            ASSERT(!error,
                   "Gurobi failed to get constraint Farkas certificate for printing (error {})",
                   error);
            dual *= -1;

            char* name = nullptr;
            error = GRBgetstrattrelement(model_, GRB_STR_ATTR_CONSTRNAME, row, &name);
            ASSERT(!error, "Gurobi failed to get constraint name for printing (error {})", error);

            PRINTLN("Constraint {}, Farkas dual {}", name, dual);
        }
    }
}

void GurobiLP::write()
{
    ZoneScopedC(TRACY_COLOUR);

    static int num = 0;
    const auto filename = fmt::format("gurobi_{}.lp", num++);
    const auto error = GRBwrite(model_, filename.c_str());
    ASSERT(!error, "Gurobi failed to save model (error {})", error);
}

void GurobiLP::set_verbose(const Bool verbose)
{
    const auto error = GRBsetintparam(GRBgetenv(model_), GRB_INT_PAR_OUTPUTFLAG, verbose);
    ASSERT(!error, "Gurobi failed to set logging (error {})", error);
}
#endif
