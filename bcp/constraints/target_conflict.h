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
    struct TargetConstraint : public Constraint
    {
        Agent finishing_agent;
        Agent crossing_agent;
        NodeTime nt;
    };
    struct TargetConstraintCandidate
    {
        Float lhs;
        Agent finishing_agent;
        Agent crossing_agent;
        NodeTime nt;
    };

    // Helper data
    Vector<TargetConstraintCandidate> candidates_;
    Matrix<UInt8> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    TargetConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Target"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Float calculate_finishing_agent_coeff(const NodeTime nt, const Path& path);
    static Float calculate_crossing_agent_coeff(const NodeTime nt, const Path& path);
    void create_row(const TargetConstraintCandidate& candidate);
};
