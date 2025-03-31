/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "master/gurobi_lp.h"
#include "problem/debug.h"
#include "types/basic_types.h"
#include "types/path.h"
#include "types/pointers.h"
#include "types/string.h"

class Variable
{
    friend class MasterProblem;
    friend class VariablePool;

    Float activity_;
    UInt32 id_;
    ColumnIndex col_;
    Agent a_;
    Time path_length_;
    Edge path_[];

  private:
    // Constructors and destructor
    Variable(const Agent a, const Time path_length) noexcept :
        activity_(1.0),
        id_(-1),
        col_(-1),
        a_(a),
        path_length_(path_length)
    {}

  public:
    // Constructors and destructor
    Variable() = delete;
    ~Variable() noexcept = default;
    Variable(const Variable&) noexcept = default;
    Variable(Variable&&) noexcept = default;
    Variable& operator=(const Variable&) noexcept = default;
    Variable& operator=(Variable&&) noexcept = default;
    static auto construct(const Agent a, const Time path_length)
    {
        debug_assert(path_length >= 1);
        auto ptr = static_cast<Variable*>(std::malloc(sizeof(Variable) + sizeof(Edge) * path_length));
        new (ptr) Variable(a, path_length);
        return UniquePtr<Variable, FreeDeleter>(ptr);
    }

    // Getters
    inline auto id() const { return id_; }
    inline auto a() const { return a_; }
    inline auto path() const { return Span<const Edge>(path_, path_length_); }
    inline auto path() { return Span<Edge>(path_, path_length_); }
    inline auto hash_data() const
    {
        static_assert(sizeof(Edge) == sizeof(char32_t));

        return std::u32string_view(reinterpret_cast<const char32_t*>(path_), path_length_);
    }
    inline auto col() const { return col_; }
    inline auto activity() const { return activity_; }
};
static_assert(sizeof(Variable) == 8*1 + 4*4);
static_assert(std::is_trivial<Variable>::value);
