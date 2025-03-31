/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "problem/map.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/priority_queue.h"
#include "types/vector.h"

// Item in the priority queue
struct HeuristicPriorityQueueItem
{
    Time g;
    Node n;
};

inline Bool operator>(const HeuristicPriorityQueueItem& lhs, const HeuristicPriorityQueueItem& rhs)
{
    return lhs.g > rhs.g;
}

class DistanceHeuristic
{
    // Priority queue data structure
    using PriorityQueue = ImmutablePriorityQueue<HeuristicPriorityQueueItem>;

    // Instance
    const Map& map_;

    // Lower bounds
    HashMap<Node, Vector<Time>> h_;

    // Solver state
    mutable PriorityQueue open_;

  public:
    // Constructors and destructor
    DistanceHeuristic() = delete;
    DistanceHeuristic(const Map& map);
    DistanceHeuristic(const DistanceHeuristic&) noexcept = default;
    DistanceHeuristic(DistanceHeuristic&&) noexcept = default;
    DistanceHeuristic& operator=(const DistanceHeuristic&) noexcept = delete;
    DistanceHeuristic& operator=(DistanceHeuristic&&) noexcept = delete;
    ~DistanceHeuristic() = default;

    // Get the lower bound from every node to a target node
    const Time* get_h(const Node target);
    Vector<Time> get_h_using_map(const Node target, const Map& map) const;

  private:
    // Generate labels
    void generate_start(const Node n, Time* h) const;
    void generate(const Node current, const Node next, Time* h) const;
    void generate_neighbours(const Node current, Time* h, const Map& map) const;

    // Compute lower bound from every node to a target node
    void search(const Node target, Time* h, const Map& map) const;
};
