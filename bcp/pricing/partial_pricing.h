/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/vector.h"

#define PRICE_PRIORITY_DECAY_FACTOR 1.3

class MasterProblem;
class Problem;
struct Instance;

struct PricingTask
{
    Agent a;
    Bool must_price;
    Float* priority;
};

class PartialPricing
{
    MasterProblem& master_;
    Vector<PricingTask> pricing_tasks_;
    Vector<Float> pricing_priority_;

  public:
    // Constructors and destructor
    PartialPricing() = delete;
    PartialPricing(const Instance& instance, Problem& problem);
    ~PartialPricing() = default;
    PartialPricing(const PartialPricing&) noexcept = delete;
    PartialPricing(PartialPricing&&) noexcept = delete;
    PartialPricing& operator=(const PartialPricing&) noexcept = delete;
    PartialPricing& operator=(PartialPricing&&) noexcept = delete;

    // Compute ordering of agents to price
    Vector<PricingTask>& agents_order();
};
