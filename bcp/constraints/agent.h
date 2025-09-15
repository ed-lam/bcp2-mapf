/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "pricing/pricer.h"
#include "types/basic_types.h"
#include "types/path.h"

class Constraint;

class AgentSeparator : public Separator
{
    struct ConstraintData
    {
        Agent a;
    };

  public:
    // Constructors and destructor
    using Separator::Separator;

    // Separator type
    constexpr static auto name()
    {
        return "Agent";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    void create_row(const Agent a);
};
