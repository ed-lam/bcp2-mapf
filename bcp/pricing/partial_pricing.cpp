/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "pricing/partial_pricing.h"
#include "problem/problem.h"
#include "types/float_compare.h"
#include "types/tracy.h"

// #define USE_PARTIAL_PRICING

#define TRACY_COLOUR tracy::Color::ColorType::DeepSkyBlue

PartialPricing::PartialPricing(const Instance& instance, Problem& problem) :
    master_(problem.master()),
    pricing_tasks_(instance.num_agents()),
    pricing_priority_(instance.num_agents(), 1.0)
{
    ZoneScopedC(TRACY_COLOUR);

#ifndef USE_PARTIAL_PRICING
    // If partial pricing is not used, always price all agents.
    const Agent A = pricing_tasks_.size();
    for (Agent a = 0; a < A; ++a)
    {
        pricing_tasks_[a] = {a, true, &pricing_priority_[a]};
    }
#endif
}

Vector<PricingTask>& PartialPricing::agents_order()
{
    ZoneScopedC(TRACY_COLOUR);

#ifdef USE_PARTIAL_PRICING
    // Get the problem data.
    const Agent A = pricing_tasks_.size();

    // Determine an order for pricing the agents.
    const auto status = master_.status();
    if (status == MasterProblemStatus::Integer)
    {
        // Price all agents if the master problem solution is integral.
        for (Agent a = 0; a < A; ++a)
        {
            pricing_tasks_[a] = {a, true, &pricing_priority_[a]};
        }
    }
    else if (status == MasterProblemStatus::Infeasible)
    {
        // Price all agents whose convexity constraint has positive dual solution.
        Agent a = 0;
        for (const auto& constraint : master_.convexity_constraints())
        {
            DEBUG_ASSERT(a < A && constraint.name() == fmt::format("agent({})", a));
            const auto dual = master_.constraint_dual_sol(constraint);
            pricing_tasks_[a] = {a, is_gt(dual, 0.0), &pricing_priority_[a]};
            ++a;
        }
    }
    else if (status == MasterProblemStatus::Fractional)
    {
        // Price all agents whose convexity constraint has positive dual solution and the agent has
        // fractional value.
        Agent a = 0;
        for (const auto& constraint : master_.convexity_constraints())
        {
            DEBUG_ASSERT(a < A && constraint.name() == fmt::format("agent({})", a));
            pricing_tasks_[a] = {a, false, &pricing_priority_[a]};
            const auto dual = master_.constraint_dual_sol(constraint);
            if (is_gt(dual, 0.0))
            {
                for (const auto& variable : master_.agent_variables(a))
                {
                    const auto val = master_.variable_primal_sol(variable);
                    if (!is_integral(val))
                    {
                        pricing_tasks_[a].must_price = true;
                        break;
                    }
                }
            }
            ++a;
        }
    }
    else
    {
        ERROR("Invalid master problem status {}", static_cast<Int32>(status));
    }

    // Sort.
    std::sort(pricing_tasks_.begin(),
              pricing_tasks_.end(),
              [](const PricingTask& task1, const PricingTask& task2)
              {
                  return std::tie(task1.must_price, *task1.priority) >
                         std::tie(task2.must_price, *task2.priority);
              });

    // Decay pricing priority.
    for (Agent a = 0; a < A; ++a)
    {
        pricing_priority_[a] /= PRICE_PRIORITY_DECAY_FACTOR;
    }
#endif

    // Done.
    return pricing_tasks_;
}
