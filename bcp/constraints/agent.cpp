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
    const Agent A = instance_.agents.size();

    // Create the constraint for every agent.
    for (Agent a = 0; a < A; ++a)
    {
        create_row(a);
        ++num_added_;
    }
}

void AgentSeparator::create_row(const Agent a)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the problem data.
    auto& master = problem_.master();

    // Print.
    debugln("    Creating agent constraint for agent {}", a);

    // Create the row.
    auto name = fmt::format("agent({})", a);
    constexpr auto object_size = sizeof(AgentConstraint);
    constexpr auto hash_size = sizeof(Agent);
    auto constraint = Constraint::construct<AgentConstraint>(object_size,
                                                             hash_size,
                                                             ConstraintFamily::Agent,
                                                             this,
                                                             std::move(name),
                                                             1,
                                                             '>',
                                                             1.0);
    debug_assert(reinterpret_cast<std::uintptr_t>(&constraint->a) ==
                 reinterpret_cast<std::uintptr_t>(constraint->data()));
    constraint->a = a;
    master.add_convexity_row(std::move(constraint));
}

void AgentSeparator::add_pricing_costs(const Constraint& constraint, const Float dual)
{
    ZoneScopedC(TRACY_COLOUR);

    // Get the constraint data.
    const auto& agent_constraint = *static_cast<const AgentConstraint*>(&constraint);
    const auto a = agent_constraint.a;

    // Add the dual solution to the reduced cost function.
    auto& pricer = problem_.pricer();
    pricer.set_constant(a, -dual);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

Float AgentSeparator::get_coeff(const Constraint& constraint, const Agent a, const Path&)
{
    ZoneScopedC(TRACY_COLOUR);

    // Calculate coefficient.
    debug_assert(static_cast<const AgentConstraint*>(&constraint)->a == a);
    return 1.0;
}

#pragma GCC diagnostic pop
