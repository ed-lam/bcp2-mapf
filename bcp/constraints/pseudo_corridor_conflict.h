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

class PseudoCorridorConflictSeparator : public Separator
{
    struct PseudoCorridorConstraint : public Constraint
    {
        Agent a1;
        Agent a2;
        Array<EdgeTime, 4> a1_ets;
        Array<EdgeTime, 2> a2_ets;
    };
    struct PseudoCorridorConstraintCandidate
    {
        Float lhs;
        Agent a1;
        Agent a2;
        Array<EdgeTime, 4> a1_ets;
        Array<EdgeTime, 2> a2_ets;
    };

    // Helper data
    Vector<PseudoCorridorConstraintCandidate> candidates_;
    Matrix<UInt8> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    PseudoCorridorConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Pseudo-corridor"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Bool calculate_a1_coeff(const Array<EdgeTime, 4>& ets, const Path& path);
    static Bool calculate_a2_coeff(const Array<EdgeTime, 2>& ets, const Path& path);
    void create_row(const PseudoCorridorConstraintCandidate& candidate);
};
