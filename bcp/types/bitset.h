/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include <climits>

inline Bool get_bitset(const Byte* bitset, const Size64 i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    return (bitset[index] & mask) != 0;
}

inline void set_bitset(Byte* const bitset, const Size64 i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    bitset[index] |= mask;
}

inline void clear_bitset(Byte* const bitset, const Size64 i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    bitset[index] &= ~mask;
}

inline Bool flip_bitset(Byte* const bitset, const Size64 i)
{
    const auto index = i / CHAR_BIT;
    const auto mask = 0b1 << (i % CHAR_BIT);
    bitset[index] ^= mask;
    return (bitset[index] & mask) != 0;
}
