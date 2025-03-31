/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/variable.h"
#include "types/basic_types.h"
#include "types/hash_map.h"

inline std::size_t hash_value(const UniquePtr<Variable, FreeDeleter>& variable_ptr) noexcept
{
    // Agents have unique start and target nodes so no need to hash the agent number as well.
    const auto& hash_data = variable_ptr->hash_data();
    return boost::hash_range(hash_data.begin(), hash_data.end());
}
inline Bool operator==(const UniquePtr<Variable, FreeDeleter>& lhs, const UniquePtr<Variable, FreeDeleter>& rhs)
{
    return lhs->hash_data() == rhs->hash_data();
}

class VariablePool
{
    HashSet<UniquePtr<Variable, FreeDeleter>> paths_;
    Size next_variable_id_;

  public:
    // Constructors and destructor
    VariablePool() : paths_(), next_variable_id_(0) {}
    ~VariablePool() = default;
    VariablePool(const VariablePool&) = default;
    VariablePool(VariablePool&&) = default;
    VariablePool& operator=(const VariablePool&) = default;
    VariablePool& operator=(VariablePool&&) = default;

    // Add path
    Variable* add_path(UniquePtr<Variable, FreeDeleter>&& variable_ptr)
    {
        auto [it, created] = paths_.emplace(std::move(variable_ptr));
        auto variable = it->get();
        if (created)
        {
            variable->id_ = next_variable_id_++;
        }
        return variable;
    }
};
