/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <boost/heap/d_ary_heap.hpp>
#pragma GCC diagnostic pop

template<class T, class Comparison = std::greater<T>>
using ImmutablePriorityQueue = boost::heap::d_ary_heap<
    T,
    boost::heap::arity<4>,
    boost::heap::compare<Comparison>,
    boost::heap::mutable_<false>
>;
template<class T, class Comparison = std::greater<T>>
using MutablePriorityQueue = boost::heap::d_ary_heap<
    T,
    boost::heap::arity<4>,
    boost::heap::compare<Comparison>,
    boost::heap::mutable_<true>
>;
