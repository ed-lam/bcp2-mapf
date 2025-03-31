/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/constraint.h"
#include "types/basic_types.h"
#include "types/hash_map.h"

inline std::size_t hash_value(const UniquePtr<Constraint, FreeDeleter>& constraint_ptr) noexcept
{
    // Hash the memory that contains a unique representation of the constraint.
    const auto& hash_data = constraint_ptr->hash_data();
    return boost::hash_range(hash_data.begin(), hash_data.end());
}
inline Bool operator==(const UniquePtr<Constraint, FreeDeleter>& lhs, const UniquePtr<Constraint, FreeDeleter>& rhs)
{
    return lhs->hash_data() == rhs->hash_data();
}

class ConstraintPool
{
    HashSet<UniquePtr<Constraint, FreeDeleter>> constraints_;

  public:
    // Constructors and destructor
    ConstraintPool() : constraints_() {}
    ~ConstraintPool() = default;
    ConstraintPool(const ConstraintPool&) = default;
    ConstraintPool(ConstraintPool&&) = default;
    ConstraintPool& operator=(const ConstraintPool&) = default;
    ConstraintPool& operator=(ConstraintPool&&) = default;

    // Add constraint
    Constraint* add_constraint(UniquePtr<Constraint, FreeDeleter>&& constraint_ptr)
    {
        auto [it, created] = constraints_.emplace(std::move(constraint_ptr));
        auto constraint = it->get();
        return constraint;
    }
};
