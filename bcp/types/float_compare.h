/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include <cmath>

#define EPS (1e-6)
#define LENIENT_EPS (1e-4)

template <class T, T eps = EPS>
inline Bool is_eq(T x, T y)
{
    return (x == y) || (std::abs(x - y) <= eps);
}

template <class T, T eps = EPS>
inline Bool is_ne(T x, T y)
{
    return !is_eq<T, eps>(x, y);
}

template <class T, T eps = EPS>
inline Bool is_lt(T x, T y)
{
    return x - y < -eps;
}

template <class T, T eps = EPS>
inline Bool is_le(T x, T y)
{
    return x - y <= eps;
}

template <class T, T eps = EPS>
inline Bool is_gt(T x, T y)
{
    return x - y > eps;
}

template <class T, T eps = EPS>
inline Bool is_ge(T x, T y)
{
    return x - y >= -eps;
}

template <class T, T eps = EPS>
inline T eps_floor(T x)
{
    return std::floor(x + eps);
}

template <class T, T eps = EPS>
inline T eps_ceil(T x)
{
    return std::ceil(x - eps);
}

template <class T, T eps = EPS>
inline T eps_round(T x)
{
    return std::ceil(x - 0.5 + eps);
}

template <class T, T eps = EPS>
inline T eps_frac(T x)
{
    return x - eps_floor<T, eps>(x);
}

template <class T, T eps = EPS>
inline Bool is_integral(T x)
{
    return eps_frac<T, eps>(x) <= eps;
}
