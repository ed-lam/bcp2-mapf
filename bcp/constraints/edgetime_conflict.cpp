/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/edgetime_conflict.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::HotPink

#define MAX_CUTS_PER_RUN (1000)

EdgeTimeConflictSeparator::EdgeTimeConflictSeparator(const Instance& instance, Problem& problem) :
    Separator(instance, problem),
    rng_(RANDOM_SEED),
    uniform_(0.0, 1.0),
    candidates_()
{
}

void EdgeTimeConflictSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Print.
    debugln("Starting separator for edgetime conflicts");

    // Get the problem data.
    const auto& map = instance_.map;

    // Get the solution.
    const auto& projection = problem_.projection();

    // Find cuts.
    candidates_.clear();
    for (const auto& [et1, val12] : projection.summed_undirected_edgetimes())
    {
        debug_assert(et1.d == Direction::NORTH || et1.d == Direction::WEST);
        if (is_gt(val12, 1.0))
        {
            candidates_.push_back({val12, uniform_(rng_), et1});
        }
    }

    // Create the most violated cuts.
    std::sort(candidates_.begin(), candidates_.end(),
              [](const auto& a, const auto& b) { return std::tie(a.lhs, a.random) > std::tie(b.lhs, b.random); });
    for (Size index = 0; index < std::min<Size>(candidates_.size(), MAX_CUTS_PER_RUN); ++index)
    {
        // Get the candidate.
        const auto& [lhs, random, et1] = candidates_[index];

        // Get the opposite edgetime.
        const EdgeTime et2{map.get_opposite_edge(et1.et.e), et1.t};

        // Choose a wait edgetime.
        auto et3 = (uniform_(rng_) <= 0.5 ? et1 : et2);
        et3.d = Direction::WAIT;

        // Print.
        debugln("    Creating edgetime constraint at {} and {} with LHS {}",
                format_edgetime(et1, map), format_edgetime(et3, map), lhs);

        // Create the row.
        create_row(et1.t, et1.et.e, et2.et.e, et3.et.e);
    }
}

void EdgeTimeConflictSeparator::create_row(const Time t, const Edge e1, const Edge e2, const Edge e3)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto& map = instance_.map;
    auto& master = problem_.master();

    // Create the row.
    debug_assert(e3.d == Direction::WAIT);
    auto name = fmt::format("edgetime{}", format_edgetime(e3.n == e1.n ? EdgeTime{e1, t} : EdgeTime{e2, t}, map));
    constexpr auto object_size = sizeof(EdgeTimeConstraint);
    constexpr auto hash_size = sizeof(Time) + sizeof(Edge) * 3;
    auto constraint = Constraint::construct<EdgeTimeConstraint>(object_size,
                                                                hash_size,
                                                                ConstraintFamily::EdgeTime,
                                                                this,
                                                                std::move(name),
                                                                0,
                                                                '<',
                                                                1.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->t) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    constraint->t = t;
    constraint->e1 = e1;
    constraint->e2 = e2;
    constraint->e3 = e3;
    master.add_row(std::move(constraint));
    ++num_added_;
}

void EdgeTimeConflictSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& edgetime_constraint = *static_cast<const EdgeTimeConstraint*>(&constraint);
    const auto t = edgetime_constraint.t;
    const auto et1 = EdgeTime{edgetime_constraint.e1, t};
    const auto et2 = EdgeTime{edgetime_constraint.e2, t};
    const auto et3 = EdgeTime{edgetime_constraint.e3, t};
    debug_assert(et3.d == Direction::WAIT);

    // Add the dual solution to the reduced cost function.
    auto& pricer = problem_.pricer();
    pricer.add_edgetime_penalty_all_agents(et1, -dual);
    pricer.add_edgetime_penalty_all_agents(et2, -dual);
    pricer.add_edgetime_penalty_all_agents(et3, -dual);
}

Float EdgeTimeConflictSeparator::get_coeff(const Constraint& constraint, const Agent, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& edgetime_constraint = *static_cast<const EdgeTimeConstraint*>(&constraint);
    const auto t = edgetime_constraint.t;
    const auto e1 = edgetime_constraint.e1;
    const auto e2 = edgetime_constraint.e2;
    const auto e3 = edgetime_constraint.e3;
    debug_assert(e3.d == Direction::WAIT);

    // Calculate coefficient.
    return calculate_coeff(t, e1, e2, e3, path);
}

Bool EdgeTimeConflictSeparator::calculate_coeff(const Time t, const Edge e1, const Edge e2, const Edge e3, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    return (t < path.size() - 1 && (path[t] == e1 || path[t] == e2 || path[t] == e3)) ||
           (t >= path.size() - 1 && path.back().n == e3.n);
}
