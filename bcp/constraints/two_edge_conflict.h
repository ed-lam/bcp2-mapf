/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/array.h"
#include "types/matrix.h"
#include "types/vector.h"

class TwoEdgeConflictSeparator : public Separator
{
    struct TwoEdgeConstraint : public Constraint
    {
        Agent a1;
        Agent a2;
        Time t;
        Array<Edge, 3> a1_es;
        Array<Edge, 3> a2_es;
    };
    struct TwoEdgeConstraintCandidate
    {
        Float lhs;
        Agent a1;
        Agent a2;
        Time t;
        Array<Edge, 3> a1_es;
        Array<Edge, 3> a2_es;
    };

    // Helper data
    Vector<TwoEdgeConstraintCandidate> candidates_;
    Matrix<UInt8> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    TwoEdgeConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Two-edge"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Float calculate_coeff(const Array<Edge, 3>& es, const Time t, const Path& path);
    void create_row(const TwoEdgeConstraintCandidate& candidate);
};
