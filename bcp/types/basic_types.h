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
using Real32 = float;
using Real64 = double;
using Size64 = std::ptrdiff_t;
using Size32 = Int32;
using Hash = std::size_t;

using Agent = Int32;
using Time = Int32;
using Cost = Real64;

constexpr auto REAL_INF = std::numeric_limits<Real64>::infinity();
constexpr auto COST_INF = std::numeric_limits<Cost>::infinity();
constexpr auto COST_NAN = std::numeric_limits<Cost>::quiet_NaN();
constexpr auto TIME_MAX = std::numeric_limits<Time>::max();
