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

class TargetConflictSeparator : public Separator
{
    struct ConstraintData
    {
        Agent finishing_agent;
        Agent crossing_agent;
        NodeTime nt;
    };
    struct TargetConstraintCandidate
    {
        Real64 lhs;
        Agent finishing_agent;
        Agent crossing_agent;
        NodeTime nt;
    };

    // Helper data
    Vector<TargetConstraintCandidate> candidates_;
    Matrix<UInt16> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    TargetConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name()
    {
        return "Target";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Real64 calculate_finishing_agent_coeff(const NodeTime nt, const Path& path);
    static Real64 calculate_crossing_agent_coeff(const NodeTime nt, const Path& path);
    void create_row(const TargetConstraintCandidate& candidate);
};
