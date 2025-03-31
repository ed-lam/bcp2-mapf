/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/map_types.h"
#include "types/span.h"

using Path = Span<const Edge>;

inline Bool operator==(const Path& lhs, const Path& rhs)
{
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}
inline Bool operator!=(const Path& lhs, const Path& rhs)
{
    return !(lhs == rhs);
}
