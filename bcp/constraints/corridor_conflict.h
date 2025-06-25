/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "master/constraint.h"
#include "types/matrix.h"
#include "types/tuple.h"
#include "types/vector.h"

class DistanceHeuristic;
class Map;

class CorridorConflictSeparator : public Separator
{
    struct CorridorConstraint : public Constraint
    {
        Agent a1;
        Agent a2;
        NodeTime a1_earliest_arrival;
        NodeTime a2_earliest_arrival;
    };
    struct CorridorConstraintCandidate
    {
        Float lhs;
        Agent a1;
        Agent a2;
        NodeTime a1_earliest_arrival;
        NodeTime a2_earliest_arrival;
    };

    // Helper data
    DistanceHeuristic& distance_heuristic_;
    Vector<CorridorConstraintCandidate> candidates_;
    Matrix<UInt8> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    CorridorConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name() { return "Corridor"; }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    void add_pricing_costs(const Constraint& constraint, const Float dual);

    // Add coefficient to a column
    Float get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Tuple<NodeTime, NodeTime, NodeTime, NodeTime> find_endpoint(const EdgeTime a1_et,
                                                                       const EdgeTime a2_et,
                                                                       const Node a1_target,
                                                                       const Node a2_target,
                                                                       const Map& map);
    void create_row(const CorridorConstraintCandidate& candidate);
};
