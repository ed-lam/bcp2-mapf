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

class RandomRounding : public PrimalHeuristic
{
    RandomNumberGenerator rng_;
    Vector<Agent> agents_;
    HashSet<NodeTime> nodetime_in_use_;
    HashSet<EdgeTime> edgetime_in_use_;
    HashMap<Node, Time> target_crossed_;
    HashMap<Node, Time> target_blocked_;

  public:
    // Constructors and destructor
    RandomRounding() = delete;
    RandomRounding(const Instance& instance, Problem& problem);
    ~RandomRounding() = default;
    RandomRounding(const RandomRounding&) = default;
    RandomRounding(RandomRounding&&) = default;
    RandomRounding& operator=(const RandomRounding&) = delete;
    RandomRounding& operator=(RandomRounding&&) = delete;

    // Primal heuristic type
    constexpr static auto name() { return "Random rounding"; }

    // Run
    Pair<Cost, Vector<Variable*>> execute();
};
