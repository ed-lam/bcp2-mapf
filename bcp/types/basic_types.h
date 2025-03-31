/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

using Byte = unsigned char;
using Bool = bool;
using Int16 = std::int16_t;
using Int32 = std::int32_t;
using Int64 = std::int64_t;
using UInt8 = std::uint8_t;
using UInt16 = std::uint16_t;
using UInt32 = std::uint32_t;
using UInt64 = std::uint64_t;
using Float = double;
using Size = std::ptrdiff_t;
using Size32 = Int32;

using Agent = Int32;
using Time = Int32;
using Cost = Float;

constexpr auto INF = std::numeric_limits<Cost>::infinity();
constexpr auto TIME_MAX = std::numeric_limits<Time>::max();
