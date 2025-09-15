/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/hash_map.h"
#include "types/pointers.h"

class Constraint;

class ConstraintStorage
{
    HashSet<UniquePtr<Constraint>> storage_;

  public:
    // Constructors and destructor
    ConstraintStorage();
    ~ConstraintStorage() = default;
    ConstraintStorage(const ConstraintStorage&) = default;
    ConstraintStorage(ConstraintStorage&&) = default;
    ConstraintStorage& operator=(const ConstraintStorage&) = default;
    ConstraintStorage& operator=(ConstraintStorage&&) = default;

    // Add constraint
    Constraint* add_constraint(UniquePtr<Constraint>&& constraint_ptr);
};
