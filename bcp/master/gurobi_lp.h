/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/span.h"
#include "types/string.h"
#include "types/vector.h"
#include <gurobi_c.h>

using Sign = char;
using Value = double;
using ColumnIndex = int;
using RowIndex = int;
using ColumnBasis = int;
using RowBasis = int;

#define DEFAULT_COLUMN_BASIS (-1)
#define DEFAULT_ROW_BASIS (0)
#define COLUMN_IS_BASIC (0)
#define ROW_IS_BASIC (0)

enum class LPAlgorithm : int
{
    Automatic = -1,
    PrimalSimplex = 0,
    DualSimplex = 1
};

enum class LPStatus
{
    Infeasible,
    Unbounded,
    Optimal,
    Unknown
};

class GurobiLP
{
    // Model
    GRBenv* env_;
    GRBmodel* model_;
    ColumnIndex num_cols_;
    RowIndex num_rows_;

    // Solution
    Value obj_;
    Vector<Value> primal_sol_;
    Vector<Value> dual_sol_;
    Vector<Value> row_slack_;
    Vector<ColumnBasis> col_basis_;
    Vector<RowBasis> row_basis_;

    // Temporary buffers
    Vector<Value> temp_infinity_;
    Vector<Value> temp_zero_;

  public:
    // Constructors and destructor
    GurobiLP();
    ~GurobiLP();
    GurobiLP(const GurobiLP&) noexcept = default;
    GurobiLP(GurobiLP&&) noexcept = default;
    GurobiLP& operator=(const GurobiLP&) noexcept = default;
    GurobiLP& operator=(GurobiLP&&) noexcept = default;

    // Getters
    auto num_cols() const { return num_cols_; }
    auto num_rows() const { return num_rows_; }
    Value col_lb(const ColumnIndex col) const;
    Value col_ub(const ColumnIndex col) const;
    Float condition_number() const;
    Float approx_condition_number() const;
#ifdef DEBUG
    String col_name(const ColumnIndex col) const;
    String row_name(const RowIndex row) const;
    ColumnBasis col_basis(const ColumnIndex col) const;
    RowBasis row_basis(const RowIndex row) const;
#endif

    // Modifications
    ColumnIndex add_column(const Span<RowIndex>& rows, const Span<Value>& coeffs, const Cost obj, const String& name);
    RowIndex add_row(const Span<ColumnIndex>& cols,
                     const Span<Value>& coeffs,
                     const Sign sign,
                     const Float rhs,
                     const String& name);
    void enable_columns();
    void disable_columns(const Span<ColumnIndex>& cols);
    void delete_columns(const Span<ColumnIndex>& cols);
    void delete_rows(const Span<RowIndex>& rows);
    void update();

    // Set up
    void set_col_basis(const Span<ColumnBasis>& basis);
    void set_row_basis(const Span<RowBasis>& basis);

    // Solve
    LPStatus solve(const LPAlgorithm algorithm = LPAlgorithm::DualSimplex,
                   const Float time_limit = std::numeric_limits<Float>::infinity());
    inline auto obj() const { return obj_; };
    inline auto get_primal_sol(const ColumnIndex col) const { return primal_sol_[col]; }
    inline auto get_dual_sol(const RowIndex row) const { return dual_sol_[row]; }
    inline auto get_slack(const RowIndex row) const { return row_slack_[row]; }
    inline auto get_col_basis(const ColumnIndex col) const { return col_basis_[col]; }
    inline auto get_row_basis(const RowIndex row) const { return row_basis_[row]; }
    inline auto col_is_basic(const ColumnIndex col) const { return get_col_basis(col) == COLUMN_IS_BASIC; }
    inline auto row_is_basic(const RowIndex row) const { return get_row_basis(row) == ROW_IS_BASIC; }

    // Debug
#ifdef DEBUG
    void reset();
    void print();
    void write();
    void set_verbose(const Bool verbose = true);
#endif
};
