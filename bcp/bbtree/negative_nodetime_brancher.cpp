/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "bbtree/negative_nodetime_brancher.h"
#include "master/master.h"
#include "problem/debug.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::PaleGreen

struct AgentCandidate
{
    Agent a;
    Float val;

    AgentCandidate(const Agent a, const Float val) : a(a), val(val) {}
    auto operator<=>(const AgentCandidate& rhs) const { return a <=> rhs.a; }
};

struct NodeCandidate
{
    Node n;
    Float val;

    NodeCandidate(const Node n, const Float val) : n(n), val(val) {}
    auto operator<=>(const NodeCandidate& rhs) const { return n <=> rhs.n; }
};

Decisions NegativeNodeTimeBrancher::branch()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();
    auto& master = problem_.master();

    // Calculate the number of times each nodetime is used.
    HashMap<NodeTime, Vector<AgentCandidate>> different_agent_candidates; // TODO: hash map for both
    Vector<Vector<Vector<NodeCandidate>>> same_agent_candidates(A); // TODO: hash map for both
    Vector<Size> num_paths(A);
    for (Agent a = 0; a < A; ++a)
    {
        auto& same_agent_candidates_a = same_agent_candidates[a];

        for (const auto& variable : master.agent_variables(a))
        {
            const auto val = master.variable_primal_sol(variable);
            if (!is_integral(val))
            {
                // Increment the number of paths used by the agent.
                ++num_paths[a];

                // Resize to store the whole path.
                const auto& path = variable.path();
                if (same_agent_candidates_a.size() < path.size())
                {
                    same_agent_candidates_a.resize(path.size());
                }

                // Store the candidates.
                for (Time t = 1; t < path.size(); ++t)
                {
                    const NodeTime nt{path[t].n, t};
                    different_agent_candidates[nt].emplace_back(a, val);
                    same_agent_candidates_a[t].emplace_back(nt.n, val);
                }
            }
        }

        for (auto& same_agent_candidates_a_t : same_agent_candidates_a)
        {
            std::sort(same_agent_candidates_a_t.begin(), same_agent_candidates_a_t.end());
            for (Size idx = same_agent_candidates_a_t.size() - 2; idx >= 0; --idx)
            {
                const auto n1 = same_agent_candidates_a_t[idx].n;
                const auto n2 = same_agent_candidates_a_t[idx + 1].n;
                if (n1 == n2)
                {
                    same_agent_candidates_a_t[idx].val += same_agent_candidates_a_t[idx + 1].val;
                    same_agent_candidates_a_t.erase(same_agent_candidates_a_t.begin() + idx + 1);
                }
            }
        }
    }
        for (auto& [nt, different_agent_candidates_nt] : different_agent_candidates)
        {
            std::sort(different_agent_candidates_nt.begin(), different_agent_candidates_nt.end());
            for (Size idx = different_agent_candidates_nt.size() - 2; idx >= 0; --idx)
            {
                const auto a1 = different_agent_candidates_nt[idx].a;
                const auto a2 = different_agent_candidates_nt[idx + 1].a;
                if (a1 == a2)
                {
                    different_agent_candidates_nt[idx].val += different_agent_candidates_nt[idx + 1].val;
                    different_agent_candidates_nt.erase(different_agent_candidates_nt.begin() + idx + 1);
                }
            }
        }

    // Choose two agent-nodetimes for branching.
    Decisions output = {std::make_unique<NegativeNodeTimeBrancherData>(),
                        std::make_unique<NegativeNodeTimeBrancherData>()};
    auto best_ant1 = static_cast<NegativeNodeTimeBrancherData*>(output.first.get());
    auto best_ant2 = static_cast<NegativeNodeTimeBrancherData*>(output.second.get());
    best_ant1->a = -1;
    Float best_val = 0.0;
    Size best_num_paths = 0;
    for (const auto& [nt, agent_candidates] : different_agent_candidates)
        for (auto it1 = agent_candidates.begin(); it1 != agent_candidates.end(); ++it1)
            for (auto it2 = it1 + 1; it2 != agent_candidates.end(); ++it2)
            {
                const auto& [a1, val1] = *it1;
                const auto& [a2, val2] = *it2;

                const auto val = val1 + val2;
                if ((val > best_val) || (val == best_val && num_paths[a1] + num_paths[a2] > best_num_paths))
                {
                    best_val =  val;
                    best_num_paths = num_paths[a1] + num_paths[a2];
                    best_ant1->a = a1;
                    best_ant1->nt = nt;
                    best_ant2->a = a2;
                    best_ant2->nt = nt;
                }
            }
    for (Agent a = 0; a < A; ++a)
        for (Time t = 0; t < same_agent_candidates[a].size(); ++t)
        {
            const auto& same_agent_candidates_a_t = same_agent_candidates[a][t];
            for (auto it1 = same_agent_candidates_a_t.begin(); it1 != same_agent_candidates_a_t.end(); ++it1)
                for (auto it2 = it1 + 1; it2 != same_agent_candidates_a_t.end(); ++it2)
                {
                    const auto& [n1, val1] = *it1;
                    const auto& [n2, val2] = *it2;

                    const auto val = val1 + val2;
                    if ((val > best_val) || (val == best_val && num_paths[a] > best_num_paths))
                    {
                        best_val =  val;
                        best_num_paths = num_paths[a];
                        best_ant1->a = a;
                        best_ant1->nt = NodeTime{n1, t};
                        best_ant2->a = a;
                        best_ant2->nt = NodeTime{n2, t};
                    }
                }
        }
    release_assert(best_ant1->a >= 0, "Negative nodetime branching failed to make a decision");
    release_assert((best_ant1->a != best_ant2->a && best_ant1->nt == best_ant2->nt) ||
                   (best_ant1->a == best_ant2->a && best_ant1->nt.t == best_ant2->nt.t && best_ant1->nt.n != best_ant2->nt.n),
                   "Negative nodetime branching decided on an invalid branching - "
                   "agent {} node {} time {}, agent {} node {} time {}",
                   best_ant1->a, best_ant1->nt.n, best_ant1->nt.t,
                   best_ant2->a, best_ant2->nt.n, best_ant2->nt.t);
    debugln("Negative nodetime brancher branching on agent {} and nodetime {} in B&B node {}",
            best_a,
            format_nodetime(best_nt, instance_.map),
            problem_.bbtree().current_id());
    num_added_ += 2;
    return output;
}

void NegativeNodeTimeBrancher::add_pricing_costs(const BrancherData* const data)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [a, nt] = *static_cast<const NegativeNodeTimeBrancherData*>(data);

    // Add the penalty.
    auto& pricer = problem_.pricer();
    pricer.add_nodetime_penalty_one_agent(a, nt, INF);
}

void NegativeNodeTimeBrancher::disable_vars(const BrancherData* const data)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Get the constraint data.
    const auto& [a, nt] = *static_cast<const NegativeNodeTimeBrancherData*>(data);

    // Disable paths that use the nodetime.
    debugln("Negative nodetime brancher disabling paths incompatible with decision on agent {} forbidding {} in B&B "
            "node {}:",
            a,
            format_nodetime(nt, instance_.map),
            problem_.bbtree().current_id());
    for (const auto& variable : master.agent_variables(a))
    {
        const auto& path = variable.path();
        const auto t = std::min<Time>(nt.t, path.size() - 1);
        const auto path_visits_nt = (path[t].n == nt.n);
        if (path_visits_nt)
        {
            debugln("Agent {}, path {}", a, format_path_with_time_spaced(path, instance_.map));
            master.disable_variable(variable);
        }
    }
    debugln("");
}

void NegativeNodeTimeBrancher::print(const BrancherData* const data) const
{
    // Get the constraint data.
    const auto& [a, nt] = *static_cast<const NegativeNodeTimeBrancherData*>(data);

    // Print.
    println("    Agent {} forbidding {}",
            a,
            format_nodetime(nt, instance_.map));
}
