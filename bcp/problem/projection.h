/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/arena.h"
#include "types/basic_types.h"
#include "types/clock.h"
#include "types/hash_map.h"
#include "types/map_types.h"
#include "types/ranges.h"
#include "types/tuple.h"
#include "types/vector.h"

class MasterProblem;
class Problem;
struct Instance;

struct ProjectionValues
{
    Real64 total;
    Real64 agent[];

    // Constructors and destructor
    ProjectionValues() = default;
    ~ProjectionValues() = default;
    ProjectionValues(const ProjectionValues&) = delete;
    ProjectionValues(ProjectionValues&&) = delete;
    ProjectionValues& operator=(const ProjectionValues&) = delete;
    ProjectionValues& operator=(ProjectionValues&&) = delete;

    // Getters
    inline auto operator[](const Agent a) const
    {
        return agent[a];
    }
};

class Projection
{
    // Problem
    const Instance& instance_;
    MasterProblem& master_;
    Clock clock_;

    // Solution in the original space, partitioned by agent
    Vector<HashMap<NodeTime, Real64>> agent_nodetimes_;
    Vector<HashMap<EdgeTime, Real64>> agent_edgetimes_;

    // Solution in the original space, summed over all agents
    HashMap<NodeTime, Real64> summed_nodetimes_;
    HashMap<EdgeTime, Real64> summed_undirected_edgetimes_;

    // Solution for reaching the target at particular times
    HashMap<AgentAgent, Vector<Real64>> pair_target_crossing_vals_;
    Vector<Time> latest_target_crossing_times_;
    HashMap<Node, Pair<Agent, Vector<Real64>*>> target_finishing_;
    HashMap<AgentNode, Vector<Real64>> agent_target_finishing_;
    HashMap<AgentNodeTime, Real64> agent_target_crossing_;

    // Solution in the original space, listed by agent
    Arena arena_;
    ProjectionValues* zeros_;
    HashMap<NodeTime, ProjectionValues*> list_fractional_nodetimes_;
    HashMap<EdgeTime, ProjectionValues*> list_fractional_edgetimes_;
    HashMap<Node, Vector<ProjectionValues*>> list_target_finish_;

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
    auto run_time() const
    {
        return clock_.total_duration();
    }

    // Get summed values
    const auto& summed_nodetimes() const
    {
        return summed_nodetimes_;
    }
    const auto& summed_undirected_edgetimes() const
    {
        return summed_undirected_edgetimes_;
    }
    Real64 find_summed_nodetime(const NodeTime et) const;

    // Get agent values
    auto agent_move_edgetimes(const Agent a) const
    {
        return agent_edgetimes_[a] |
               Ranges::views::filter([](const auto& it) { return it.first.d != Direction::WAIT; });
    }
    const auto& target_finishing() const
    {
        return target_finishing_;
    }
    const auto& agent_target_crossing() const
    {
        return agent_target_crossing_;
    }
    Real64 find_agent_nodetime(const Agent a, const NodeTime nt) const;
    Real64 find_agent_move_edgetime(const Agent a, const EdgeTime et) const;
    Real64 find_agent_wait_edgetime(const Agent a, const EdgeTime et) const;

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
