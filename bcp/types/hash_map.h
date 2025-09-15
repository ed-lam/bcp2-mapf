/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/map_types.h"
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

template <class Key, class T, class Hash = boost::hash<Key>, class KeyEqual = std::equal_to<Key>,
          class Allocator = std::allocator<std::pair<const Key, T>>>
using HashMap = boost::unordered_flat_map<Key, T, Hash, KeyEqual, Allocator>;

template <class Key, class Hash = boost::hash<Key>, class KeyEqual = std::equal_to<Key>,
          class Allocator = std::allocator<Key>>
using HashSet = boost::unordered_flat_set<Key, Hash, KeyEqual, Allocator>;

inline Hash hash_value(const XYT xyt) noexcept
{
    return boost::hash<UInt64>()(xyt.id());
}

inline Hash hash_value(const NodeTime nt) noexcept
{
    return boost::hash<UInt64>()(nt.id());
}

inline Hash hash_value(const Edge e) noexcept
{
    return boost::hash<UInt32>()(e.id());
}

inline Hash hash_value(const EdgeTime et) noexcept
{
    return boost::hash<UInt64>()(et.id());
}

inline Hash hash_value(const AgentNode an) noexcept
{
    return boost::hash<UInt64>()(an.id());
}

inline Hash hash_value(const AgentTime at) noexcept
{
    return boost::hash<UInt64>()(at.id());
}

inline Hash hash_value(const AgentNodeTime ant) noexcept
{
    return boost::hash<UInt64>()(ant.id());
}

inline Hash hash_value(const AgentAgent aa) noexcept
{
    return boost::hash<UInt64>()(aa.id());
}
