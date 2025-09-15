/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/gurobi_lp.h"
#include "pricing/pricer.h"
#include "types/basic_types.h"
#include "types/debug.h"
#include "types/path.h"
#include "types/pointers.h"
#include <boost/container_hash/hash.hpp>

class Constraint;
class Separator;

using ApplyConstraintInPricerFunction = void (*)(const Constraint& constraint, const Real64 dual,
                                                 Pricer& pricer);
using GetCoeffFunction = Real64 (*)(const Constraint& constraint, const Agent a, const Path& path);

class Constraint
{
    friend class MasterProblem;
    friend class ConstraintStorage;

  protected:
    Hash hash_;
    Real64 activity_;
    Real64 rhs_;
    RowIndex row_;
    Sign sign_ : 8;
    Agent num_agents_ : 24;
    ApplyConstraintInPricerFunction apply_in_pricer_;
    Size32 data_size_;
    Size32 hash_size_;
    GetCoeffFunction get_coeff_;
    Byte data_[];

  private:
    Constraint(const Sign sign, const Real64 rhs, const Agent num_agents, const Size64 data_size,
               const Size64 hash_size, ApplyConstraintInPricerFunction apply_in_pricer,
               GetCoeffFunction get_coeff, const String& name) :
        hash_(0),
        activity_(1.0),
        rhs_(rhs),
        row_(-1),
        sign_(sign),
        num_agents_(num_agents),
        apply_in_pricer_(apply_in_pricer),
        data_size_(data_size),
        hash_size_(sizeof(GetCoeffFunction) + hash_size),
        get_coeff_(get_coeff)
    {
        // Copy the name.
        const auto name_size = name.size();
        auto name_ptr = new (data_ + data_size_) char[name_size + 1];
        const auto name_c_str = name.c_str();
        std::copy(name_c_str, name_c_str + name_size + 1, name_ptr);
        DEBUG_ASSERT(name_c_str[name_size] == 0);
    }

  public:
    // Constructors and destructor
    Constraint() = delete;
    ~Constraint() = default;
    Constraint(const Constraint&) noexcept = delete;
    Constraint(Constraint&&) noexcept = delete;
    Constraint& operator=(const Constraint&) noexcept = delete;
    Constraint& operator=(Constraint&&) noexcept = delete;

    static auto construct(const Sign sign, const Real64 rhs, const Agent num_agents,
                          const Size64 data_size, const Size64 hash_size,
                          ApplyConstraintInPricerFunction apply_in_pricer,
                          GetCoeffFunction get_coeff, const String& name)
    {
        // Calculate object size and round up to alignment.
        auto object_size = sizeof(Constraint) + data_size + (name.size() + 1);
        const auto alignment = alignof(Constraint);
        object_size = (object_size + alignment - 1) & (~(alignment - 1));

        // Allocate and construct.
        auto ptr = new (::operator new(object_size)) Constraint(
            sign, rhs, num_agents, data_size, hash_size, apply_in_pricer, get_coeff, name);
        return UniquePtr<Constraint>(ptr);
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
    inline Real64 rhs() const
    {
        return rhs_;
    }
    inline auto row() const
    {
        return row_;
    }
    inline auto sign() const
    {
        return sign_;
    }
    inline auto data() const
    {
        return data_;
    }
    inline auto data()
    {
        return data_;
    }
    // inline auto data_range() const
    // {
    //     return Span<const Byte>(data(), data_size_);
    // }
    inline auto hash_range() const
    {
        return Span<const Byte>(data(), hash_size_);
    }
    inline auto num_agents() const
    {
        return num_agents_;
    }
    inline auto agents() const
    {
        return Span<const Agent>(reinterpret_cast<const Agent*>(data()), num_agents());
    }
    inline auto name() const
    {
        return reinterpret_cast<const char*>(data_ + data_size_);
    }
    inline auto apply_in_pricer(const Real64 dual, Pricer& pricer) const
    {
        return (*apply_in_pricer_)(*this, dual, pricer);
    }
    inline auto get_coeff(const Agent a, const Path& path) const
    {
        return (*get_coeff_)(*this, a, path);
    }

  protected:
    // Hashing
    void update_hash()
    {
        // Check that hash is not yet computed.
        DEBUG_ASSERT(hash_ == 0);

        // (1) Hash the number of bytes for a unique representation of the object to prevent ending
        // zeros and smaller data to hash to the same value. Stop 0100 and 01 hashing to the same
        // value.
        hash_ = boost::hash<Size32>()(hash_size_);

        // (2) Hash the pointer to the separator function as a proxy for the constraint family.
        // (3) Hash the data.
        auto ptr = reinterpret_cast<Byte*>(&get_coeff_);
        auto n = hash_size_;
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
static_assert(sizeof(Constraint) == 8 * 3 + 4 * 2 + 8 * 1 + 4 * 2 + 8 * 1);
static_assert(std::is_trivial_v<Constraint>);
