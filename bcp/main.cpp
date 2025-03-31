/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "problem/debug.h"
#include "problem/instance.h"
#include "problem/problem.h"
#include <cxxopts.hpp>

int main(int argc, char** argv)
{
    // Parse program options.
    FilePath instance_path;
    Agent agent_limit = -1;
    Float time_limit = std::numeric_limits<Float>::infinity();
    // UInt64 node_limit = 0;
    // Float gap_limit = 0;
    try
    {
        // Create program options.
        cxxopts::Options options(argv[0],
                                 "BCP2-MAPF - branch-and-cut-and-price for multi-agent path finding, version 2");
        options.positional_help("instance").show_positional_help();
        options.add_options()
            ("help", "Print help")
            ("i,instance", "Path to instance file", cxxopts::value<String>())
            ("a,agent-limit", "Read the first several agents", cxxopts::value<Agent>())
            ("t,time-limit", "Time limit in seconds", cxxopts::value<Float>())
            // ("n,node-limit", "Maximum number of branch-and-bound nodes", cxxopts::value<UInt64>())
            // ("g,gap-limit", "Solve to an optimality gap", cxxopts::value<Float>())
        ;
        options.parse_positional({"instance"});

        // Parse options.
        auto result = options.parse(argc, argv);

        // Print help.
        if (result.count("help") || !result.count("instance"))
        {
            println("{}", options.help());
            exit(0);
        }

        // Get path to instance.
        if (result.count("instance"))
        {
            instance_path = result["instance"].as<String>();
        }

        // Get agent limit.
        if (result.count("agent-limit"))
        {
            agent_limit = result["agent-limit"].as<Agent>();
        }

        // Get time limit.
        if (result.count("time-limit"))
        {
            time_limit = result["time-limit"].as<Float>();
        }

        // Get node limit.
        // if (result.count("node-limit"))
        // {
        //     node_limit = result["node-limit"].as<UInt64>();
        // }

        // Get optimality gap limit.
        // if (result.count("gap-limit"))
        // {
        //     gap_limit = result["gap-limit"].as<Float>();
        // }
    }
    catch (const cxxopts::exceptions::exception& e)
    {
        err("{}", e.what());
    }

    // Print.
    println("BCP2-MAPF - branch-and-cut-and-price for multi-agent path finding, version 2");
    println("Edward Lam <ed@ed-lam.com>");
#ifdef DEBUG
    println("Compiled in debug mode");
#endif
    println("");

    // Create problem and solve.
    Problem problem{instance_path, agent_limit};
    problem.solve(time_limit);

    // Done.
    return 0;
}
