/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"

class AgentSeparator : public Separator
{
    struct AgentConstraint : public Constraint
    {
        Agent a;
    };

  public:
    // Constructors and destructor
    using Separator::Separator;

    // Separator type
    constexpr static auto name() { return "Agent"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    void create_row(const Agent a);
};
