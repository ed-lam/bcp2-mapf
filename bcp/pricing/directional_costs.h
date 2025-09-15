/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/debug.h"
#include "types/map_types.h"

// Data structure for storing the costs of traversing an edge
struct DirectionalCosts
{
    union
    {
        Cost costs[5];
        struct
        {
            Cost north;
            Cost south;
            Cost west;
            Cost east;
            Cost wait;
        };
    };

    // Constructors and destructor
    DirectionalCosts(const Cost x) :
        north(x),
        south(x),
        west(x),
        east(x),
        wait(x)
    {
    }
    DirectionalCosts() :
        DirectionalCosts(0)
    {
    }
    DirectionalCosts(const DirectionalCosts&) noexcept = default;
    DirectionalCosts(DirectionalCosts&&) noexcept = default;
    DirectionalCosts& operator=(const DirectionalCosts&) noexcept = delete;
    DirectionalCosts& operator=(DirectionalCosts&&) noexcept = delete;
    ~DirectionalCosts() = default;

    // Getters
    inline auto operator[](const Size64 d) const
    {
        DEBUG_ASSERT(0 <= d && d < 5);
        return costs[d];
    }
    inline auto& operator[](const Size64 d)
    {
        DEBUG_ASSERT(0 <= d && d < 5);
        return costs[d];
    }

    // Reset all values to 0
    void set_zero()
    {
        costs[0] = 0;
        costs[1] = 0;
        costs[2] = 0;
        costs[3] = 0;
        costs[4] = 0;
    }
};
static_assert(std::is_trivially_copyable_v<DirectionalCosts>);
static_assert(sizeof(DirectionalCosts) == 5 * 8);
