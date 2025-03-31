/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/gurobi_lp.h"
#include "types/basic_types.h"
#include "types/pointers.h"
#include "types/tuple.h"

class Brancher;
struct BrancherData;
class Constraint;
class Variable;

// Branch-and-bound tree node
struct BBNode
{
    UInt32 id;
    UInt32 depth;
    Cost lb;
    SharedPtr<BBNode> parent;

    Brancher* brancher;
    UniquePtr<BrancherData> decision;

    Vector<Pair<Variable*, ColumnBasis>> variables;
    Vector<Pair<Constraint*, RowBasis>> constraints;
};
