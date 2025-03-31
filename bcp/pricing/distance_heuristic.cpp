/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "pricing/distance_heuristic.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::CornflowerBlue

DistanceHeuristic::DistanceHeuristic(const Map& map) :
    map_(map),
    h_(),
    open_()
{
    ZoneScopedC(TRACY_COLOUR);

    h_.reserve(1000);
}

void DistanceHeuristic::generate_start(const Node n, Time* h) const
{
    ZoneScopedC(TRACY_COLOUR);

    const Time g = 0;
    h[n] = g;
    open_.push(HeuristicPriorityQueueItem{g, n});
}

void DistanceHeuristic::generate(const Node current, const Node next, Time* h) const
{
    ZoneScopedC(TRACY_COLOUR);

    const auto g = h[current] + 1;
    if (g < h[next])
    {
        h[next] = g;
        open_.push(HeuristicPriorityQueueItem{g, next});
    }
}

void DistanceHeuristic::generate_neighbours(const Node current, Time* h, const Map& map) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Expand in four directions.
    if (const auto next = map.get_north(current); map[next])
    {
        generate(current, next, h);
    }
    if (const auto next = map.get_south(current); map[next])
    {
        generate(current, next, h);
    }
    if (const auto next = map.get_west(current); map[next])
    {
        generate(current, next, h);
    }
    if (const auto next = map.get_east(current); map[next])
    {
        generate(current, next, h);
    }
}

void DistanceHeuristic::search(const Node target, Time* h, const Map& map) const
{
    ZoneScopedC(TRACY_COLOUR);

    // Reset.
    open_.clear();

    // Fill with infeasible time.
    std::fill(h, h + map.size(), TIME_MAX);

    // Solve.
    generate_start(target, h);
    while (!open_.empty())
    {
        // Get a label from priority queue.
        const auto [_, current] = open_.top();
        open_.pop();

        // Generate neighbours.
        generate_neighbours(current, h, map);
    }
}

const Time* DistanceHeuristic::get_h(const Node target)
{
    ZoneScopedC(TRACY_COLOUR);

    auto& h = h_[target];
    if (h.empty())
    {
        // Allocate memory for N+1 nodes, ranging from -1 to N-1 , where n = -1 represents the end.
        h.resize(map_.size() + 1);
        h[0] = 0;

        // Compute the h values for this target.
        search(target, h.data() + 1, map_);
    }
    return h.data() + 1; // INFO: Returning pointer to data in vector so h_ can change.
}

Vector<Time> DistanceHeuristic::get_h_using_map(const Node target, const Map& map) const
{
    ZoneScopedC(TRACY_COLOUR);

    Vector<Time> h(map.size());
    search(target, h.data(), map);
    return h;
}
