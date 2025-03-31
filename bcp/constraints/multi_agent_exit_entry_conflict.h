/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/hash_map.h"
#include "types/vector.h"

class MultiAgentExitEntryConflictSeparator : public Separator
{
    struct MultiAgentExitEntryConstraint : public Constraint
    {
        Agent every_agent;
        Agent canonical;
        Time t;
        Edge canonical_e;
        Size32 num_other_es;
        Edge other_es[];
    };
    struct MultiAgentExitEntryConstraintCandidate
    {
        Float lhs;
        Size num_other;
        Agent canonical;
        Time t;
        Edge canonical_e;
        Edge opposite_e;
        Node origin;
        Node destination;
    };

    // Helper data
    Vector<MultiAgentExitEntryConstraintCandidate> candidates_;
    HashMap<AgentTime, UInt8> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    MultiAgentExitEntryConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Multi-agent exit-entry"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Float calculate_canonical_agent_coeff(const Edge e, const Time t, const Path& path);
    static Float calculate_other_agent_coeff(const Size32 num_es, const Edge* es, const Time t, const Path& path);
    void create_row(const MultiAgentExitEntryConstraintCandidate& candidate);
};
