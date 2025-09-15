/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/matrix.h"
#include "types/vector.h"

class ExitEntryConflictSeparator : public Separator
{
    struct ConstraintData
    {
        Agent a1;
        Agent a2;
        Time t;
        Edge a1_e;
        Size32 num_a2_es;
        Edge a2_es[];
    };
    struct ExitEntryConstraintCandidate
    {
        Real64 lhs;
        Agent a1;
        Agent a2;
        Time t;
        Edge a1_e;
        Edge a2_opposite_e;
        Node origin;
        Node destination;
    };

    // Helper data
    Vector<ExitEntryConstraintCandidate> candidates_;
    Matrix<UInt16> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    ExitEntryConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name()
    {
        return "Exit-entry";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Bool calculate_a1_coeff(const Edge e, const Time t, const Path& path);
    static Bool calculate_a2_coeff(const Size32 num_es, const Edge* es, const Time t,
                                   const Path& path);
    void create_row(const ExitEntryConstraintCandidate& candidate);
};
