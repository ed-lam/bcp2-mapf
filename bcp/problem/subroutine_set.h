/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/tuple.h"

class Problem;
struct Instance;

// Container storing subroutines using static polymorphism
template <class... SubroutineTypes>
struct SubroutineSet : public Tuple<SubroutineTypes...>
{
    using Base = Tuple<SubroutineTypes...>;

    SubroutineSet(const Instance& instance, Problem& problem) :
        Base(SubroutineTypes(instance, problem)...)
    {
    }

    constexpr static auto size()
    {
        return std::tuple_size<Base>();
    }

    template <class T>
    auto apply(T&& f)
    {
        return std::apply(f, *static_cast<Base*>(this));
    }

    template <class T>
    auto apply(T&& f) const
    {
        return std::apply(f, *static_cast<const Base*>(this));
    }
};

template <class... SubroutineTypes>
struct std::tuple_size<SubroutineSet<SubroutineTypes...>>
    : integral_constant<size_t, sizeof...(SubroutineTypes)>
{
};

template <size_t I, class... SubroutineTypes>
struct std::tuple_element<I, SubroutineSet<SubroutineTypes...>>
{
    using type = tuple_element_t<I, tuple<SubroutineTypes...>>;
};