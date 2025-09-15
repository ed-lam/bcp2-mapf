/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

// #define PRINT_DEBUG

#include "problem/instance.h"
#include "types/debug.h"
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

Instance::Instance(const FilePath& scenario_path_, const Agent agent_limit_) :
    name(scenario_path_.filename().stem().string()),
    scenario_path(scenario_path_),
    map_path(),
    map(),
    agents()
{
    // Open scenario file.
    std::ifstream scenario_file;
    scenario_file.open(scenario_path, std::ios::in);
    ASSERT(scenario_file.good(), "Cannot find scenario file \"{}\"", scenario_path.string());

    // Read file.
    char buffer[1024];
    scenario_file.getline(buffer, 1024);
    ASSERT(strstr(buffer, "version 1"), "Expecting \"version 1\" scenario file format");

    // Read agents data.
    const auto agent_limit = agent_limit_ < 0 ? std::numeric_limits<Agent>::max() : agent_limit_;
    String agent_map_path;
    Position agent_map_width;
    Position agent_map_height;
    Position start_x;
    Position start_y;
    Position target_x;
    Position target_y;
    Real64 tmp;
    String first_agent_map_path;
    while (num_agents() < agent_limit && scenario_file >> tmp >> agent_map_path >>
                                             agent_map_width >> agent_map_height >> start_x >>
                                             start_y >> target_x >> target_y >> tmp)
    {
        // Add padding.
        agent_map_width += 2;
        agent_map_height += 2;
        start_x++;
        start_y++;
        target_x++;
        target_y++;

        // Read map.
        if (map.empty())
        {
            // Prepend the directory of the scenario file.
            map_path = scenario_path.parent_path().append(agent_map_path);

            // Store map data for checking.
            first_agent_map_path = agent_map_path;

            // Read map.
            map = Map(map_path);
        }

        // Store the agent.
        const auto a = num_agents();
        auto& [start, target] = agents.emplace_back();
        start = map.get_n(start_x, start_y);
        target = map.get_n(target_x, target_y);
        ASSERT(agent_map_path == first_agent_map_path, "Agent {} uses a different map", a);
        ASSERT(agent_map_width == map.width(),
               "Map width of agent {} does not match actual map size",
               a);
        ASSERT(agent_map_height == map.height(),
               "Map height of agent {} does not match actual map size",
               a);
        ASSERT(start < map.size() && map[start], "Agent {} starts at an obstacle", a);
        ASSERT(target < map.size() && map[target], "Agent {} ends at an obstacle", a);
    }
    ASSERT(agent_limit_ == -1 || num_agents() == agent_limit_,
           "Cannot read {} agents from a scenario with only {} agents",
           agent_limit_,
           num_agents());

    // Close file.
    scenario_file.close();
}
