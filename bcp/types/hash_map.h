/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/map_types.h"
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

template<class Key,
         class T,
         class Hash = boost::hash<Key>,
         class KeyEqual = std::equal_to<Key>,
         class Allocator = std::allocator<std::pair<const Key, T>>>
using HashMap = boost::unordered_flat_map<Key, T, Hash, KeyEqual, Allocator>;

template<class Key,
         class Hash = boost::hash<Key>,
         class KeyEqual = std::equal_to<Key>,
         class Allocator = std::allocator<Key>>
using HashSet = boost::unordered_flat_set<Key, Hash, KeyEqual, Allocator>;

inline std::size_t hash_value(const XYT xyt) noexcept
{
    static_assert(sizeof(XYT) == sizeof(UInt64));
    return boost::hash<decltype(xyt.id)>{}(xyt.id);
}

inline std::size_t hash_value(const NodeTime nt) noexcept
{
    static_assert(sizeof(NodeTime) == sizeof(UInt64));
    return boost::hash<decltype(nt.id)>{}(nt.id);
}

inline std::size_t hash_value(const Edge e) noexcept
{
    static_assert(sizeof(Edge) == sizeof(UInt32));
    return boost::hash<decltype(e.id)>{}(e.id);
}

inline std::size_t hash_value(const EdgeTime et) noexcept
{
    static_assert(sizeof(EdgeTime) == sizeof(UInt64));
    return boost::hash<decltype(et.id)>{}(et.id);
}

inline std::size_t hash_value(const AgentNode an) noexcept
{
    static_assert(sizeof(AgentNode) == sizeof(UInt64));
    return boost::hash<decltype(an.id)>{}(an.id);
}

inline std::size_t hash_value(const AgentTime at) noexcept
{
    static_assert(sizeof(AgentTime) == sizeof(UInt64));
    return boost::hash<decltype(at.id)>{}(at.id);
}
