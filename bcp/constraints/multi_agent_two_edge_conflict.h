/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/array.h"
#include "types/hash_map.h"
#include "types/vector.h"

class MultiAgentTwoEdgeConflictSeparator : public Separator
{
    struct ConstraintData
    {
        Agent every_agent;
        Agent canonical;
        Time t;
        Array<Edge, 3> canonical_es;
        Array<Edge, 3> other_es;
    };
    struct MultiAgentTwoEdgeConstraintCandidate
    {
        Real64 lhs;
        Size64 num_other;
        Agent canonical;
        Time t;
        Array<Edge, 3> canonical_es;
        Array<Edge, 3> other_es;
    };

    // Helper data
    Vector<MultiAgentTwoEdgeConstraintCandidate> candidates_;
    HashMap<AgentTime, UInt16> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    MultiAgentTwoEdgeConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name()
    {
        return "Multi-agent two-edge";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Real64 calculate_coeff(const Array<Edge, 3>& es, const Time t, const Path& path);
    void create_row(const MultiAgentTwoEdgeConstraintCandidate& candidate);
};
