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
    DEBUGLN("Starting separator for nodetime conflicts");

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
    std::sort(candidates_.begin(),
              candidates_.end(),
              [](const auto& a, const auto& b)
              { return std::tie(a.lhs, a.random) > std::tie(b.lhs, b.random); });
    for (Size64 index = 0; index < std::min<Size64>(candidates_.size(), MAX_CUTS_PER_RUN); ++index)
    {
        // Get the candidate.
        const auto& [lhs, random, nt] = candidates_[index];

        // Print.
        DEBUGLN("    Creating nodetime constraint at {} with LHS {}",
                format_nodetime(nt, instance_.map),
                lhs);

        // Create the row.
        create_row(nt);
    }
}

void NodeTimeConflictSeparator::create_row(const NodeTime nt)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Create the row.
    auto name = fmt::format("nodetime{}", format_nodetime(nt, instance_.map));
    constexpr auto data_size = sizeof(ConstraintData);
    constexpr auto hash_size = data_size;
    auto constraint = Constraint::construct(
        '<', 1.0, 0, data_size, hash_size, &apply_in_pricer, &get_coeff, name);
    auto data = new (constraint->data()) ConstraintData;
    data->nt = nt;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void NodeTimeConflictSeparator::apply_in_pricer(const Constraint& constraint, const Real64 dual,
                                                Pricer& pricer)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [nt] = *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Add the dual solution to the reduced cost function.
    pricer.add_nodetime_penalty_all_agents(nt, -dual);
}

Real64 NodeTimeConflictSeparator::get_coeff(const Constraint& constraint, const Agent,
                                            const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [nt] = *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Calculate coefficient.
    return calculate_nodetime_coeff(nt, path);
}
