/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/hash_map.h"
#include "types/memory_pool.h"
#include "types/ranges.h"
#include "types/tuple.h"

class MasterProblem;
class Problem;
struct Instance;

struct ProjectionValues
{
    Float total;
    Float agent[];

    // Constructors and destructor
    ProjectionValues() = default;
    ~ProjectionValues() = default;
    ProjectionValues(const ProjectionValues&) = delete;
    ProjectionValues(ProjectionValues&&) = delete;
    ProjectionValues& operator=(const ProjectionValues&) = delete;
    ProjectionValues& operator=(ProjectionValues&&) = delete;

    // Getters
    inline auto operator[](const Agent a) const { return agent[a]; }
};

class Projection
{
    // Problem
    const Instance& instance_;
    MasterProblem& master_;
    Clock clock_;

    // Solution in the original space, partitioned by agent
    Vector<HashMap<NodeTime, Float>> agent_nodetimes_;
    Vector<HashMap<EdgeTime, Float>> agent_edgetimes_;

    // Solution in the original space, summed over all agents
    HashMap<NodeTime, Float> summed_nodetimes_;
    HashMap<EdgeTime, Float> summed_undirected_edgetimes_;

    // Solution for reaching the target at particular times
    HashMap<AgentNode, Vector<Float>> agent_node_finish_;
    HashMap<Node, Pair<Agent, Vector<Float>*>> node_finish_;

    // Solution in the original space, listed by agent
    MemoryPool memory_pool_;
    ProjectionValues* zeros_;
    HashMap<NodeTime, ProjectionValues*> list_fractional_nodetimes_;
    HashMap<EdgeTime, ProjectionValues*> list_fractional_edgetimes_;
    HashMap<Node, Vector<ProjectionValues*>> list_node_finish_;

  public:
    // Constructors and destructor
    Projection() = delete;
    ~Projection() = default;
    Projection(const Projection&) = delete;
    Projection(Projection&&) = delete;
    Projection& operator=(const Projection&) = delete;
    Projection& operator=(Projection&&) = delete;
    Projection(const Instance& instance, Problem& problem);

    // Statistics
    auto run_time() const { return clock_.total_duration(); }

    // Get summed values
    const auto& summed_nodetimes() const { return summed_nodetimes_; }
    const auto& summed_undirected_edgetimes() const { return summed_undirected_edgetimes_; }
    Float find_summed_nodetime(const NodeTime et) const;
    // Float find_summed_wait_edgetime(const EdgeTime et) const;

    // Get agent values
    auto agent_move_edgetimes(const Agent a) const
    {
        return agent_edgetimes_[a] |
            Ranges::views::filter([](const auto& it) { return it.first.d != Direction::WAIT; });
    }
    const auto& agent_node_finish() const { return agent_node_finish_; }
    Float find_agent_nodetime(const Agent a, const NodeTime nt) const;
    Float find_agent_move_edgetime(const Agent a, const EdgeTime et) const;
    Float find_agent_wait_edgetime(const Agent a, const EdgeTime et) const;

    // Get list values
    const ProjectionValues& find_list_fractional_nodetime(const NodeTime nt) const;
    const ProjectionValues& find_list_fractional_move_edgetime(const EdgeTime et) const;
    const ProjectionValues& find_list_fractional_wait_edgetime(const EdgeTime et) const;

    // Update projection
    void update();

  private:
    // Update projection
    void clear();
    void compute();
};
