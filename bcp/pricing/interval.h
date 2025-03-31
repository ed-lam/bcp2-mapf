/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"

struct Interval
{
    Time start;
    Time end;
    Cost cost;
    Interval* next;

    // Constructors and destructor
    Interval() = default;
    ~Interval() = default;
    Interval(const Interval&) noexcept = default;
    Interval(Interval&&) noexcept = default;
    Interval& operator=(const Interval&) noexcept = default;
    Interval& operator=(Interval&&) noexcept = default;
    Interval(const Time start, const Time end, const Cost cost) : start(start), end(end), cost(cost), next(nullptr) {}
};
