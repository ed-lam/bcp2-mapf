/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "constraints/separator.h"
#include "pricing/pricer.h"
#include "types/matrix.h"
#include "types/tuple.h"
#include "types/vector.h"

class DistanceHeuristic;
class Map;

class CorridorConflictSeparator : public Separator
{
    struct ConstraintData
    {
        Agent a1;
        Agent a2;
        NodeTime a1_earliest_arrival;
        NodeTime a2_earliest_arrival;
    };
    struct CorridorConstraintCandidate
    {
        Real64 lhs;
        Agent a1;
        Agent a2;
        NodeTime a1_earliest_arrival;
        NodeTime a2_earliest_arrival;
    };

    // Helper data
    DistanceHeuristic& distance_heuristic_;
    Vector<CorridorConstraintCandidate> candidates_;
    Matrix<UInt16> num_separated_;

  public:
    // Constructors and destructor
    using Separator::Separator;
    CorridorConflictSeparator(const Instance& instance, Problem& problem);

    // Separator type
    constexpr static auto name()
    {
        return "Corridor";
    }

    // Separate
    void separate();

    // Add dual solution to pricing costs
    static void apply_in_pricer(const Constraint& constraint, const Real64 dual, Pricer& pricer);

    // Add coefficient to a column
    static Real64 get_coeff(const Constraint& constraint, const Agent a, const Path& path);

  private:
    static Tuple<NodeTime, NodeTime, NodeTime, NodeTime> find_endpoint(const EdgeTime a1_et,
                                                                       const EdgeTime a2_et,
                                                                       const Node a1_target,
                                                                       const Node a2_target,
                                                                       const Map& map);
    void create_row(const CorridorConstraintCandidate& candidate);
};
