/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/tuple.h"

// Container storing subroutines using static polymorphism
template<class... SubroutineTypes>
struct SubroutineSet : public Tuple<SubroutineTypes...>
{
    SubroutineSet(const Instance& instance, Problem& problem) :
        Tuple<SubroutineTypes...>(SubroutineTypes{instance, problem}...) {}

    constexpr static auto size() { return std::tuple_size<Tuple<SubroutineTypes...>>(); }

    // template<Size index>
    // const auto& get() const { return std::get<index>(*this); }
    // template<Size index>
    // auto& get() { return std::get<index>(*this); }
};

template<class... SubroutineTypes>
struct std::tuple_size<SubroutineSet<SubroutineTypes...>> :
    std::integral_constant<std::size_t, sizeof...(SubroutineTypes)>
{
};
