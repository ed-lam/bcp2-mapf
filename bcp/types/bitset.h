/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "problem/debug.h"
#include "types/basic_types.h"
#include <climits>

static inline Bool get_bitset(const void* bitset, const Size i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    return (reinterpret_cast<const Byte*>(bitset)[index] & mask) != 0;
}

static inline void set_bitset(void* const bitset, const Size i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    reinterpret_cast<Byte*>(bitset)[index] |= mask;
}

// static inline void clear_bitset(void* const bitset, const Size i)
// {
//     const auto index = i / CHAR_BIT;
//     const auto mask = 0b1 << (i % CHAR_BIT);
//     reinterpret_cast<Byte*>(bitset)[index] &= ~mask;
// }

static inline Bool flip_bitset(void* const bitset, const Size i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    reinterpret_cast<Byte*>(bitset)[index] ^= mask;
    return (reinterpret_cast<const Byte*>(bitset)[index] & mask) != 0;
}

// static inline void and_bitset(void* const bitset1, const void* const bitset2, const Size size)
// {
//     for (Size index = 0; index < size; ++index)
//     {
//         reinterpret_cast<Byte*>(bitset1)[index] &= reinterpret_cast<const Byte*>(bitset2)[index];
//     }
// }

// static inline void or_bitset(void* const bitset1, const void* const bitset2, const Size size)
// {
//     for (Size index = 0; index < size; ++index)
//     {
//         reinterpret_cast<Byte*>(bitset1)[index] |= reinterpret_cast<const Byte*>(bitset2)[index];
//     }
// }

// static inline UInt8 get_2_bitset(const void* bitset, const Size i)
// {
//     const auto index = i * 2 / CHAR_BIT;
//     const auto mask = 0b11 << (i * 2 % CHAR_BIT);
//     return (reinterpret_cast<const Byte*>(bitset)[index] & mask) >> (i * 2 % CHAR_BIT);
// }

// static inline void set_2_bitset(void* const bitset, const Size i, const UInt8 val)
// {
//     debug_assert(val < 4);
//     const auto index = i * 2 / CHAR_BIT;
//     const auto mask = val << (i * 2 % CHAR_BIT);
//     reinterpret_cast<Byte*>(bitset)[index] |= mask;
// }
