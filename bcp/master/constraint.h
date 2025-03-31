/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/gurobi_lp.h"
#include "problem/debug.h"
#include "types/basic_types.h"
#include "types/pointers.h"
#include "types/string.h"

class Separator;

enum class ConstraintFamily : UInt32
{
    Agent,
    Corridor,
    EdgeTime,
    ExitEntry,
    MultiAgentExitEntry,
    MultiAgentTwoEdge,
    NodeTime,
    PseudoCorridor,
    RectangleClique,
    RectangleKnapsack,
    Target,
    TwoEdge
};

class Constraint
{
    friend class MasterProblem;
    friend class ConstraintPool;

  protected:
    String name_;
    Separator* separator_;
    Float activity_;
    Float rhs_;
    RowIndex row_;
    Size32 hash_size_;
    Size32 num_agents_ : 24;
    Sign sign_ : 8;
    ConstraintFamily family_;
    char32_t data_[0];
    // WARNING: data_[0] to data_[num_agents_] must contain the agents involved in this row. This flexible array can be
    // empty if the constraint is universal (same edge costs/coefficients for every agent) or the array can contain -1
    // if the constraint spans all agents but has different edge costs/coefficients for different agents.
    // WARNING: data_[0] to data_[hash_size_] must contain a unique representation of the constraint for a hash table.

  private:
    Constraint(const Size32 hash_size,
               const ConstraintFamily family,
               Separator* separator,
               String&& name,
               const Size32 num_agents,
               const Sign sign,
               const Float rhs) :
        name_(std::move(name)),
        separator_(separator),
        activity_(1.0),
        rhs_(rhs),
        row_(-1),
        hash_size_(1 + hash_size / sizeof(char32_t)),
        num_agents_(num_agents),
        sign_(sign),
        family_(family)
    {
        debug_assert(hash_size % sizeof(char32_t) == 0);
    }

  public:
    // Constructors and destructor
    Constraint() = delete;
    ~Constraint() = default;
    Constraint(const Constraint&) noexcept = default;
    Constraint(Constraint&&) noexcept = default;
    Constraint& operator=(const Constraint&) noexcept = default;
    Constraint& operator=(Constraint&&) noexcept = default;
    template<class T>
    static auto construct(const Size32 object_size,
                          const Size32 hash_size,
                          const ConstraintFamily family,
                          Separator* separator,
                          String&& name,
                          const Size32 num_agents,
                          const Sign sign,
                          const Float rhs)
    {
        auto ptr = UniquePtr<T, FreeDeleter>{static_cast<T*>(std::malloc(object_size))};
        new (ptr.get()) Constraint(hash_size, family, separator, std::move(name), num_agents, sign, rhs);
        return ptr;
    }

    // Getters
    inline auto separator() const { return separator_; }
    inline const auto& name() const { return name_; }
    inline auto activity() const { return activity_; }
    inline auto row() const { return row_; }
    inline auto sign() const { return sign_; }
    inline auto rhs() const { return rhs_; }
    inline auto data() const { return data_; }
    inline auto hash_data() const
    {
        static_assert(sizeof(ConstraintFamily) == sizeof(char32_t));
        debug_assert(reinterpret_cast<std::uintptr_t>(data_) ==
                     reinterpret_cast<std::uintptr_t>(&family_) + sizeof(ConstraintFamily));

        return std::u32string_view(reinterpret_cast<const char32_t*>(&family_), hash_size_);
    }
    inline auto agents() const
    {
        const auto begin = reinterpret_cast<const Agent*>(data_);
        const auto end = reinterpret_cast<const Agent*>(data_) + num_agents_;
        return Span<const Agent>{begin, end};
    }
};
static_assert(sizeof(Constraint) == sizeof(String) + 8*3 + 4*4);
