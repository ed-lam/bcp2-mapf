/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "heuristics/heuristic.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/random.h"
#include "types/vector.h"

class SimpleRounding : public PrimalHeuristic
{
    struct CandidatePath
    {
        Float random;
        UInt32 rounded_val;
        Agent a;
        Variable* variable_ptr;
    };

    RandomNumberGenerator rng_;
    UniformDistribution uniform_;
    Vector<CandidatePath> candidates_;
    HashSet<NodeTime> nodetime_in_use_;
    HashSet<EdgeTime> edgetime_in_use_;
    HashMap<Node, Time> target_crossed_;
    HashMap<Node, Time> target_blocked_;

  public:
    // Constructors and destructor
    SimpleRounding() = delete;
    SimpleRounding(const Instance& instance, Problem& problem);
    ~SimpleRounding() = default;
    SimpleRounding(const SimpleRounding&) = default;
    SimpleRounding(SimpleRounding&&) = default;
    SimpleRounding& operator=(const SimpleRounding&) = delete;
    SimpleRounding& operator=(SimpleRounding&&) = delete;

    // Primal heuristic type
    constexpr static auto name() { return "Simple rounding"; }

    // Run
    Pair<Cost, Vector<Variable*>> execute();
};
