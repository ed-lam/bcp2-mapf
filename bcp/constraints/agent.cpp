/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "constraints/agent.h"
#include "master/master.h"
#include "output/formatting.h"
#include "problem/problem.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::LightPink

void AgentSeparator::separate()
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    const auto A = instance_.num_agents();

    // Create the constraint for every agent.
    for (Agent a = 0; a < A; ++a)
    {
        create_row(a);
    }
}

void AgentSeparator::create_row(const Agent a)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Print.
    DEBUGLN("    Creating agent constraint for agent {}", a);

    // Create the row.
    auto name = fmt::format("agent({})", a);
    constexpr auto data_size = sizeof(ConstraintData);
    constexpr auto hash_size = data_size;
    auto constraint = Constraint::construct(
        '>', 1.0, 1, data_size, hash_size, &apply_in_pricer, &get_coeff, name);
    auto data = new (constraint->data()) ConstraintData;
    data->a = a;
    master.add_permanent_row(std::move(constraint));
    ++num_added_;
}

void AgentSeparator::apply_in_pricer(const Constraint& constraint, const Real64 dual,
                                     Pricer& pricer)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& [a] = *reinterpret_cast<const ConstraintData*>(constraint.data());

    // Add the dual solution to the reduced cost function.
    pricer.set_constant(a, -dual);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
Real64 AgentSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path& path)
{
    ZoneScopedC(TRACY_COLOUR);

    // Check the constraint data.
#ifdef DEBUG
    const auto& data = *reinterpret_cast<const ConstraintData*>(constraint.data());
    DEBUG_ASSERT(data.a == a);
#endif

    // Calculate coefficient.
    return 1.0;
}
#pragma GCC diagnostic pop
