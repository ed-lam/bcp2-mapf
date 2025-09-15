/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "master/variable_storage.h"
#include "types/path.h"

inline Hash hash_value(const UniquePtr<Variable>& variable_ptr) noexcept
{
    DEBUG_ASSERT(variable_ptr->hash() != 0);
    return variable_ptr->hash();
}
inline Bool operator==(const UniquePtr<Variable>& lhs, const UniquePtr<Variable>& rhs)
{
    return lhs->path() == rhs->path();
}

VariableStorage::VariableStorage() :
    storage_(),
    next_variable_id_(0)
{
}

Variable* VariableStorage::add_path(UniquePtr<Variable>&& variable_ptr)
{
    // Compute the hash.
    DEBUG_ASSERT(variable_ptr->hash() == 0);
    variable_ptr->update_hash();
    DEBUG_ASSERT(variable_ptr->hash() != 0);

    // Store.
    auto [it, created] = storage_.emplace(std::move(variable_ptr));
    auto variable = it->get();
    if (created)
    {
        variable->id_ = next_variable_id_++;
    }
    return variable;
}
