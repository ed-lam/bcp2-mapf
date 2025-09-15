/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "problem/problem.h"
#include "types/debug.h"
#include <cxxopts.hpp>

#define PROGRAM_DESCRIPTION                                                                        \
    "BCP2-MAPF - branch-and-cut-and-price for multi-agent path finding, version 2"

int main(int argc, char** argv)
{
    // Parse program options.
    FilePath instance_path;
    Agent agent_limit = -1;
    Real64 time_limit = REAL_INF;
    try
    {
        // Create program options.
        cxxopts::Options options(argv[0], PROGRAM_DESCRIPTION);
        options.positional_help("instance").show_positional_help();
        options.add_options()("help", "Print help")(
            "i,instance", "Path to instance file", cxxopts::value<String>())(
            "a,agent-limit", "Read the first several agents", cxxopts::value<Agent>())(
            "t,time-limit", "Time limit in seconds", cxxopts::value<Real64>());
        options.parse_positional({"instance"});

        // Parse options.
        auto result = options.parse(argc, argv);

        // Print help.
        if (result.count("help") || !result.count("instance"))
        {
            PRINTLN("{}", options.help());
            exit(0);
        }

        // Get the path to instance.
        if (result.count("instance"))
        {
            instance_path = result["instance"].as<String>();
        }

        // Get the agent limit.
        if (result.count("agent-limit"))
        {
            agent_limit = result["agent-limit"].as<Agent>();
        }

        // Get the time limit.
        if (result.count("time-limit"))
        {
            time_limit = result["time-limit"].as<Real64>();
        }
    }
    catch (const cxxopts::exceptions::exception& e)
    {
        ERROR("{}", e.what());
    }

    // Print.
    PRINTLN(PROGRAM_DESCRIPTION);
    PRINTLN("Edward Lam <ed@ed-lam.com>");
#ifdef DEBUG
    PRINTLN("Compiled in debug mode");
#endif
    PRINTLN("");

    // Create problem and solve.
    Problem problem(instance_path, agent_limit);
    problem.solve(time_limit);

    // Done.
    return 0;
}
