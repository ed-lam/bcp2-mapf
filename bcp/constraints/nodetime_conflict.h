/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/random.h"
#include "types/vector.h"

class NodeTimeConflictSeparator : public Separator
{
    struct NodeTimeConstraint : public Constraint
    {
        NodeTime nt;
    };
    struct NodeTimeConstraintCandidate
    {
        Float lhs;
        Float random;
        NodeTime nt;
    };

    // Helper data
    RandomNumberGenerator rng_;
    UniformDistribution uniform_;
    Vector<NodeTimeConstraintCandidate> candidates_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    NodeTimeConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Node-time"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Bool calculate_coeff(const NodeTime nt, const Path& path);
    void create_row(const NodeTime nt);
};
