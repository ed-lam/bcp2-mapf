/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/gurobi_lp.h"
#include "types/basic_types.h"
#include "types/debug.h"
#include "types/path.h"
#include "types/pointers.h"
#include <boost/container_hash/hash.hpp>

class Variable
{
    friend class MasterProblem;
    friend class VariableStorage;

    Hash hash_;
    Real64 activity_;
    UInt32 id_;
    ColumnIndex col_;
    Agent a_;
    Time path_length_;
    Edge path_[];

  private:
    // Constructors and destructor
    Variable(const Agent a, const Time path_length) noexcept :
        hash_(0),
        activity_(1.0),
        id_(-1),
        col_(-1),
        a_(a),
        path_length_(path_length)
    {
    }

  public:
    // Constructors and destructor
    Variable() = delete;
    ~Variable() noexcept = default;
    Variable(const Variable&) noexcept = delete;
    Variable(Variable&&) noexcept = delete;
    Variable& operator=(const Variable&) noexcept = delete;
    Variable& operator=(Variable&&) noexcept = delete;
    static auto construct(const Agent a, const Time path_length)
    {
        DEBUG_ASSERT(path_length >= 1);

        // Calculate object size and round up to alignment.
        auto object_size = sizeof(Variable) + sizeof(Edge) * path_length;
        const auto alignment = alignof(Variable);
        object_size = (object_size + alignment - 1) & (~(alignment - 1));

        // Allocate and construct.
        auto ptr = new (::operator new(object_size)) Variable(a, path_length);
        new (ptr->path_) Edge[path_length];
        return UniquePtr<Variable>(ptr);
    }

    // Getters
    inline auto hash() const
    {
        return hash_;
    }
    inline auto activity() const
    {
        return activity_;
    }
    inline auto id() const
    {
        return id_;
    }
    inline auto col() const
    {
        return col_;
    }
    inline auto a() const
    {
        return a_;
    }
    inline auto path() const
    {
        return Span<const Edge>(path_, path_length_);
    }
    inline auto path()
    {
        return Span<Edge>(path_, path_length_);
    }

  protected:
    // Hashing
    void update_hash()
    {
        // Check that hash is not yet computed.
        DEBUG_ASSERT(hash_ == 0);

        // Hash the path. Agents have unique start and target nodes so no need to hash the agent
        // number as well.
        auto ptr = reinterpret_cast<Byte*>(&path_);
        auto n = sizeof(Edge) * path_length_;
        while (n >= sizeof(std::size_t))
        {
            DEBUG_ASSERT(reinterpret_cast<std::uintptr_t>(ptr) % alignof(std::size_t) == 0);
            std::size_t data;
            std::memcpy(&data, ptr, sizeof(std::size_t));
            boost::hash_combine(hash_, data);
            ptr += sizeof(std::size_t);
            n -= sizeof(std::size_t);
        }
        if (n)
        {
            std::size_t data = 0;
            std::memcpy(&data, ptr, n);
            boost::hash_combine(hash_, data);
        }
    }
};
static_assert(sizeof(Variable) == 8 * 2 + 4 * 4);
static_assert(std::is_trivial_v<Variable>);
