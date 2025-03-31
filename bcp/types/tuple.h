/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include <utility>

template<class T1, class T2>
using Pair = std::pair<T1, T2>;

template<class... T>
using Tuple = std::tuple<T...>;
