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

class EdgeTimeConflictSeparator : public Separator
{
    struct ConstraintData
    {
        Time t;
        Edge e1;
        Edge e2;
        Edge e3;
    };
    struct EdgeTimeConstraintCandidate
    {
        Real64 lhs;
        Real64 random;
        EdgeTime et1;
    };

    // Helper data
    RandomNumberGenerator rng_;
    UniformDistribution uniform_;
    Vector<EdgeTimeConstraintCandidate> candidates_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    EdgeTimeConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name()
    {
        return "Edge-time";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Bool calculate_coeff(const Time t, const Edge e1, const Edge e2, const Edge e3,
                                const Path& path);
    void create_row(const Time t, const Edge e1, const Edge e2, const Edge e3);
};
