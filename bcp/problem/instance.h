/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "problem/map.h"
#include "types/basic_types.h"
#include "types/file_system.h"
#include "types/string.h"
#include "types/vector.h"

struct AgentData
{
    Node start;
    Node target;
};

struct Instance
{
    // Instance
    String name;
    FilePath scenario_path;
    FilePath map_path;

    // Map
    Map map;

    // Agents
    Agent num_agents;
    Vector<AgentData> agents;
    Time time_horizon;

  public:
    // Constructors and destructor
    Instance(const FilePath& scenario_path, const Agent agent_limit);
    Instance() = delete;
    Instance(const Instance&) = delete;
    Instance(Instance&&) = delete;
    Instance& operator=(const Instance&) = delete;
    Instance& operator=(Instance&&) = delete;
    ~Instance() = default;
};
