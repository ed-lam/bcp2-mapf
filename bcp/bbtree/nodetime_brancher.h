/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "bbtree/brancher.h"
#include "types/map_types.h"

class NodeTimeBrancher : public Brancher
{
    struct NodeTimeBrancherData : public BrancherData
    {
        Agent a;
        NodeTime nt;
        BranchDirection dir;
    };

  public:
    // Constructors and destructor
    using Brancher::Brancher;

    // Bracner type
    constexpr static auto name()
    {
        return "Node-time";
    }

    // Branch
    Decisions branch();

    // Prevent future incompatible edges in the pricing problem
    void add_pricing_costs(const BrancherData* const data);

    // Disable existing incompatible paths in the master problem
    void disable_vars(const BrancherData* const data);

    // Print
    void print(const BrancherData* const data) const;
};
