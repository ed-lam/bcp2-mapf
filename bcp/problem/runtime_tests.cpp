/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "problem/debug.h"
#include "problem/runtime_tests.h"
#include "types/basic_types.h"
#include "types/pointers.h"

struct FlexibleArrayTestStruct
{
    Agent a1;
    Agent a2;
    UInt32 data[0];
};

struct FlexibleArrayTestStructSubclass : public FlexibleArrayTestStruct
{
    Agent a3;
    Agent a4;
    UInt32 array[];
};

static_assert(sizeof(FlexibleArrayTestStruct) == sizeof(Agent) * 2,
              "Your compiler or platform does not support flexible array members and/or arrays of size 0.");
static_assert(sizeof(FlexibleArrayTestStructSubclass) == sizeof(Agent) * 4,
              "Your compiler or platform does not support flexible array members and/or arrays of size 0.");

void test_flexible_array_pointers()
{
    auto ptr = std::make_unique<FlexibleArrayTestStructSubclass>();
    release_assert(reinterpret_cast<std::uintptr_t>(&ptr->a3) == reinterpret_cast<std::uintptr_t>(&ptr->data),
                   "Your compiler or platform does not support flexible array members and/or arrays of size 0.");
    release_assert(reinterpret_cast<std::uintptr_t>(&ptr->array) ==
                   reinterpret_cast<std::uintptr_t>(&ptr->data) + sizeof(Agent) * 2,
                   "Your compiler or platform does not support flexible array members and/or arrays of size 0.");
}
