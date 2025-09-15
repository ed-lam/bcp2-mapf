/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/variable.h"
#include "types/basic_types.h"
#include "types/hash_map.h"

class VariableStorage
{
    HashSet<UniquePtr<Variable>> storage_;
    Size64 next_variable_id_;

  public:
    // Constructors and destructor
    VariableStorage();
    ~VariableStorage() = default;
    VariableStorage(const VariableStorage&) = default;
    VariableStorage(VariableStorage&&) = default;
    VariableStorage& operator=(const VariableStorage&) = default;
    VariableStorage& operator=(VariableStorage&&) = default;

    // Add path
    Variable* add_path(UniquePtr<Variable>&& variable_ptr);
};
