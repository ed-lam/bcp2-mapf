/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "master/constraint_storage.h"
#include "master/constraint.h"

inline Hash hash_value(const UniquePtr<Constraint>& constraint_ptr) noexcept
{
    DEBUG_ASSERT(constraint_ptr->hash() != 0);
    return constraint_ptr->hash();
}

inline Bool operator==(const UniquePtr<Constraint>& lhs, const UniquePtr<Constraint>& rhs)
{
    const auto& lhs_data = lhs->hash_range();
    const auto& rhs_data = rhs->hash_range();
    return std::equal(lhs_data.begin(), lhs_data.end(), rhs_data.begin(), rhs_data.end());
}

ConstraintStorage::ConstraintStorage() :
    storage_()
{
}

Constraint* ConstraintStorage::add_constraint(UniquePtr<Constraint>&& constraint_ptr)
{
    // Compute the hash.
    DEBUG_ASSERT(constraint_ptr->hash() == 0);
    constraint_ptr->update_hash();
    DEBUG_ASSERT(constraint_ptr->hash() != 0);

    // Store.
    auto [it, created] = storage_.emplace(std::move(constraint_ptr));
    auto constraint = it->get();
    return constraint;
}
