/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/nodetime_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::Crimson

#define MAX_CUTS_PER_RUN (1000)

NodeTimeConflictSeparator::NodeTimeConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    rng_(RANDOM_SEED),
    uniform_(0.0, 1.0),
    candidates_()
{
}

void NodeTimeConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for nodetime conflicts");

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    for (const auto& [nt, val] : projection.summed_nodetimes())
        if (is_gt(val, 1.0))
        {
            candidates_.push_back({val, uniform_(rng_), nt});
        }

    // Create the most violated cuts.
    std::sort(candidates_.begin(), candidates_.end(),
              [](const auto& a, const auto& b) { return std::tie(a.lhs, a.random) > std::tie(b.lhs, b.random); });
    for (Size index = 0; index < std::min<Size>(candidates_.size(), MAX_CUTS_PER_RUN); ++index)
    {
        // Get the candidate.
        const auto& [lhs, random, nt] = candidates_[index];

        // Print.
        debugln("    Creating nodetime constraint at {} with LHS {}", format_nodetime(nt, instance_.map), lhs);

        // Create the row.
        create_row(nt);
        ++num_added_;
    }
}

void NodeTimeConflictSeparator::create_row(const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Create the row.
    auto name = fmt::format("nodetime{}", format_nodetime(nt, instance_.map));
    constexpr auto object_size = sizeof(NodeTimeConstraint);
    constexpr auto hash_size = sizeof(NodeTime);
    auto constraint = Constraint::construct<NodeTimeConstraint>(object_size,
                                                                hash_size,
                                                                ConstraintFamily::NodeTime,
                                                                this,
                                                                std::move(name),
                                                                0,
                                                                '<',
                                                                1.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->nt) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    constraint->nt = nt;
    master.add_row(std::move(constraint));
}

void NodeTimeConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& nodetime_constraint = *static_cast<const NodeTimeConstraint*>(&constraint);
    const auto nt = nodetime_constraint.nt;

    // Add the dual solution to the reduced cost function.
    auto& pricer = problem_.pricer();
    pricer.add_nodetime_penalty_all_agents(nt, -dual);
}

Float NodeTimeConflictSeparator::get_coeff(const Constraint& constraint, const Agent, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& nodetime_constraint = *static_cast<const NodeTimeConstraint*>(&constraint);
    const auto nt = nodetime_constraint.nt;

    // Calculate coefficient.
    return calculate_nodetime_coeff(nt, path);
}
