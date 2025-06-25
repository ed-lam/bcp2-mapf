/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include <type_traits>

using Position = Int16;

struct XY
{
    // struct
    // {
        Position x;
        Position y;
    // };
    // UInt32 id;

    constexpr XY() noexcept = default;
    // explicit constexpr XY(const UInt32 id) noexcept : id(id) {}
    constexpr XY(const Position x, const Position y) noexcept : x(x), y(y) {}
};
static_assert(sizeof(Position) == 2);
static_assert(sizeof(XY) == 4);
static_assert(std::has_unique_object_representations_v<XY>);
static_assert(std::is_trivial<XY>::value);
// inline Bool operator==(const XY lhs, const XY rhs) { return lhs.id == rhs.id; }
// inline Bool operator!=(const XY lhs, const XY rhs) { return !(lhs == rhs); }

union XYT
{
    struct
    {
        Position x;
        Position y;
        Time t;
    };
    UInt64 id;

    constexpr XYT() noexcept = default;
    // explicit constexpr XYT(const UInt64 id) noexcept : id(id) {}
    constexpr XYT(const Position x, const Position y, const Time t) noexcept : x(x), y(y), t(t) {}
};
static_assert(sizeof(Position) == 2);
static_assert(sizeof(Time) == 4);
static_assert(sizeof(XYT) == 8);
static_assert(std::has_unique_object_representations_v<XYT>);
static_assert(std::is_trivial<XYT>::value);
inline Bool operator==(const XYT lhs, const XYT rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const XYT lhs, const XYT rhs) { return !(lhs == rhs); }

using Node = Int32;

union NodeTime
{
    struct
    {
        Node n;
        Time t;
    };
    UInt64 id;

    constexpr NodeTime() noexcept = default;
    // explicit constexpr NodeTime(const UInt64 nt) noexcept : nt(nt) {}
    constexpr NodeTime(const Node n, const Time t) noexcept : n(n), t(t) {}
};
static_assert(sizeof(Node) == 4);
static_assert(sizeof(Time) == 4);
static_assert(sizeof(NodeTime) == 8);
static_assert(std::has_unique_object_representations_v<NodeTime>);
static_assert(std::is_trivial<NodeTime>::value);
inline Bool operator==(const NodeTime lhs, const NodeTime rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const NodeTime lhs, const NodeTime rhs) { return !(lhs == rhs); }

enum Direction : UInt8
{
    NORTH = 0,  // 0000
    SOUTH = 1,  // 0001
    WEST = 2,   // 0010
    EAST = 3,   // 0011
    WAIT = 4,   // 0100
    INVALID = 5 // 0101
};
// inline Direction& operator++(Direction& d)
// {
//     d = static_cast<Direction>(static_cast<std::underlying_type<Direction>::type>(d) + 1);
//     return d;
// }

union Edge
{
    struct
    {
        Node n : 29;
        Direction d : 3;
    };
    UInt32 id;

    constexpr Edge() noexcept = default;
    constexpr Edge(const Node n, const Direction d) noexcept : n(n), d(d) {}
};
static_assert(sizeof(Node) == 4);
static_assert(sizeof(Edge) == 4);
static_assert(std::has_unique_object_representations_v<Edge>);
static_assert(std::is_trivial<Edge>::value);
inline Bool operator==(const Edge lhs, const Edge rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const Edge lhs, const Edge rhs) { return !(lhs == rhs); }

union EdgeTime
{
    struct
    {
        Edge e;
        Time t;
    } et;
    struct
    {
        Node n : 29;
        Direction d : 3;
        Time t;
    };
    UInt64 id;

    constexpr EdgeTime() noexcept = default;
    constexpr EdgeTime(const Edge e, const Time t) noexcept : et(e, t) {}
    constexpr EdgeTime(const NodeTime nt, const Direction d) noexcept : n(nt.n), d(d), t(nt.t) {}
    constexpr EdgeTime(const Node n, const Direction d, const Time t) noexcept : n(n), d(d), t(t) {}

    inline NodeTime nt() const noexcept { return NodeTime{n, t}; }
};
static_assert(sizeof(EdgeTime) == 8);
static_assert(std::has_unique_object_representations_v<EdgeTime>);
static_assert(std::is_trivial<EdgeTime>::value);
inline Bool operator==(const EdgeTime lhs, const EdgeTime rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const EdgeTime lhs, const EdgeTime rhs) { return !(lhs == rhs); }

union AgentNode
{
    struct
    {
        Agent a;
        Node n;
    };
    UInt64 id;

    constexpr AgentNode() noexcept = default;
    constexpr AgentNode(const Agent a, const Node n) noexcept : a(a), n(n) {}
};
static_assert(sizeof(AgentNode) == 8);
static_assert(std::has_unique_object_representations_v<AgentNode>);
static_assert(std::is_trivial<AgentNode>::value);
inline Bool operator==(const AgentNode lhs, const AgentNode rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const AgentNode lhs, const AgentNode rhs) { return !(lhs == rhs); }

union AgentTime
{
    struct
    {
        Agent a;
        Time t;
    };
    UInt64 id;

    constexpr AgentTime() noexcept = default;
    constexpr AgentTime(const Agent a, const Time t) noexcept : a(a), t(t) {}
};
static_assert(sizeof(AgentTime) == 8);
static_assert(std::has_unique_object_representations_v<AgentTime>);
static_assert(std::is_trivial<AgentTime>::value);
inline Bool operator==(const AgentTime a, const AgentTime b) { return a.id == b.id; }
inline Bool operator!=(const AgentTime a, const AgentTime b) { return !(a == b); }

union AgentNodeTime
{
    struct
    {
        UInt16 a;
        UInt16 t;
        Node n;
    };
    UInt64 id;

    constexpr AgentNodeTime() noexcept = default;
    constexpr AgentNodeTime(const Agent a, const Node n, const Time t) noexcept : a(a), t(t), n(n) {}
};
static_assert(sizeof(AgentNodeTime) == 8);
static_assert(std::has_unique_object_representations_v<AgentNodeTime>);
static_assert(std::is_trivial<AgentNodeTime>::value);
inline Bool operator==(const AgentNodeTime lhs, const AgentNodeTime rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const AgentNodeTime lhs, const AgentNodeTime rhs) { return !(lhs == rhs); }

union AgentAgent
{
    struct
    {
        Agent a1;
        Agent a2;
    };
    UInt64 id;

    constexpr AgentAgent() noexcept = default;
    constexpr AgentAgent(const Agent a1, const Agent a2) noexcept : a1(a1), a2(a2) {}
};
static_assert(sizeof(AgentAgent) == 8);
static_assert(std::has_unique_object_representations_v<AgentAgent>);
static_assert(std::is_trivial<AgentAgent>::value);
inline Bool operator==(const AgentAgent lhs, const AgentAgent rhs) { return lhs.id == rhs.id; }
inline Bool operator!=(const AgentAgent lhs, const AgentAgent rhs) { return !(lhs == rhs); }
