/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/span.h"
#include "types/vector.h"

class RectangleKnapsackConflictSeparator : public Separator
{
  public:
    struct ConstraintData
    {
        Agent a1;
        Agent a2;
        NodeTime a1_start;
        NodeTime a2_start;
        NodeTime a1_end;
        NodeTime a2_end;
        Size32 num_a1_ets;
        Size32 num_ets;
        EdgeTime ets[];

        Span<const EdgeTime> a1_ets() const
        {
            return {&ets[0], &ets[num_a1_ets]};
        }
        Span<const EdgeTime> a2_ets() const
        {
            return {&ets[num_a1_ets], &ets[num_ets]};
        }
    };

  public:
    // Constructors and destructor
    using Separator::Separator;

    // Separator type
    constexpr static auto name()
    {
        return "Rectangle knapsack";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Real64 calculate_coeff(const Span<const EdgeTime>& ets, const Path& path);
    void create_row(const Agent a1, const Agent a2, const NodeTime a1_start,
                    const NodeTime a2_start, const NodeTime a1_end, const NodeTime a2_end,
                    const Size32 num_a1_ets, const Size32 num_a2_ets,
                    const Vector<EdgeTime>& rectangle);
};
