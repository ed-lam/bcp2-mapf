cmake_minimum_required(VERSION 3.21)
project(bcp2-mapf)

# Verbose.
# set(CMAKE_VERBOSE_MAKEFILE on)

# Enable downloading dependencies.
include(FetchContent)

# Add custom configurations
set(_extra_cfgs Profile ASan UBSan)
if(CMAKE_CONFIGURATION_TYPES)  # multi-config generators
    set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES};${_extra_cfgs}" CACHE STRING "" FORCE)
else()                         # single-config generators
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel ${_extra_cfgs})
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
    endif()
endif()

# Set C++ version.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

# Set release build.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

# Set source files.
set(SOURCE_FILES
    bcp/bbtree/bbtree.cpp
    bcp/bbtree/length_brancher.cpp
    bcp/bbtree/negative_nodetime_brancher.cpp
    bcp/bbtree/node_selectors.cpp
    bcp/bbtree/nodetime_brancher.cpp
    bcp/constraints/agent.cpp
    bcp/constraints/corridor_conflict.cpp
    bcp/constraints/edgetime_conflict.cpp
    bcp/constraints/exit_entry_conflict.cpp
    bcp/constraints/multi_agent_exit_entry_conflict.cpp
    bcp/constraints/multi_agent_two_edge_conflict.cpp
    bcp/constraints/nodetime_conflict.cpp
    bcp/constraints/pseudo_corridor_conflict.cpp
    bcp/constraints/rectangle_clique_conflict.cpp
    bcp/constraints/rectangle_knapsack_conflict.cpp
    bcp/constraints/target_conflict.cpp
    bcp/constraints/two_edge_conflict.cpp
    bcp/heuristics/random_rounding.cpp
    bcp/heuristics/simple_rounding.cpp
    bcp/main.cpp
    bcp/master/constraint_storage.cpp
    bcp/master/gurobi_lp.cpp
    bcp/master/master.cpp
    bcp/master/variable_storage.cpp
    bcp/output/formatting.cpp
    bcp/pricing/distance_heuristic.cpp
    bcp/pricing/independent_intervals.cpp
    bcp/pricing/independent_time_expanded_astar_pricer.cpp
    bcp/pricing/independent_time_expanded_astar.cpp
    bcp/pricing/independent_time_interval_astar_pricer.cpp
    bcp/pricing/independent_time_interval_astar.cpp
    bcp/pricing/partial_pricing.cpp
    bcp/pricing/shared_intervals.cpp
    bcp/pricing/shared_time_expanded_astar_pricer.cpp
    bcp/pricing/shared_time_expanded_astar.cpp
    bcp/pricing/shared_time_interval_astar_pricer.cpp
    bcp/pricing/shared_time_interval_astar.cpp
    bcp/problem/instance.cpp
    bcp/problem/map.cpp
    bcp/problem/problem.cpp
    bcp/problem/projection.cpp
    bcp/problem/runtime_tests.cpp
    bcp/types/arena.cpp
    bcp/types/tracy.cpp
    )

# Declare executables.
add_executable(bcp2-mapf ${SOURCE_FILES})
target_include_directories(bcp2-mapf PRIVATE ./ bcp/)

# Include Gurobi.
find_path(GUROBI_INCLUDE_DIR
    NAMES gurobi_c.h
    HINTS "${GUROBI_DIR}/include" "${GUROBI_DIR}/*/include"
          "$ENV{GUROBI_DIR}/include" "$ENV{GUROBI_DIR}/*/include"
          "$ENV{GUROBI_HOME}/include"
    REQUIRED)
find_library(GUROBI_LIBRARY
    NAMES gurobi120
    HINTS "${GUROBI_DIR}/lib" "${GUROBI_DIR}/*/lib"
          "$ENV{GUROBI_DIR}/lib" "$ENV{GUROBI_DIR}/*/lib"
          "$ENV{GUROBI_HOME}/lib"
    REQUIRED)
message("Found Gurobi at ${GUROBI_LIBRARY} and ${GUROBI_INCLUDE_DIR}")
target_include_directories(bcp2-mapf PRIVATE ${GUROBI_INCLUDE_DIR})
target_link_libraries(bcp2-mapf PRIVATE ${GUROBI_LIBRARY})

# Download boost.
set(BOOST_VERSION 1.88.0)
FetchContent_Declare(boost
    URL "https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-b2-nodocs.tar.gz"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(boost)
find_path(BOOST_INCLUDE_DIR
    NAMES boost/heap/d_ary_heap.hpp
    HINTS "${boost_SOURCE_DIR}"
    REQUIRED)
target_include_directories(bcp2-mapf PRIVATE ${BOOST_INCLUDE_DIR})

# Download fmt.
FetchContent_Declare(fmt
    SOURCE_DIR        fmt
    GIT_REPOSITORY    https://github.com/fmtlib/fmt.git
    GIT_TAG           e424e3f
    GIT_SHALLOW       TRUE
    GIT_PROGRESS      TRUE
    GIT_SUBMODULES    ""
)
FetchContent_MakeAvailable(fmt)
find_path(FMT_INCLUDE_DIR
    NAMES fmt/core.h
    HINTS "${fmt_SOURCE_DIR}/include/"
    REQUIRED)
target_include_directories(bcp2-mapf PRIVATE ${FMT_INCLUDE_DIR})
target_link_libraries(bcp2-mapf PRIVATE fmt::fmt)

# Download ranges.
FetchContent_Declare(ranges
    SOURCE_DIR        ranges
    GIT_REPOSITORY    https://github.com/ericniebler/range-v3.git
    GIT_TAG           ca1388f
    GIT_SHALLOW       TRUE
    GIT_PROGRESS      TRUE
    GIT_SUBMODULES    ""
)
FetchContent_MakeAvailable(ranges)
find_path(RANGES_INCLUDE_DIR
    NAMES range/v3/all.hpp
    HINTS "${ranges_SOURCE_DIR}/include/"
    REQUIRED)
target_include_directories(bcp2-mapf PRIVATE ${RANGES_INCLUDE_DIR})

# Download cxxopts.
FetchContent_Declare(cxxopts
    SOURCE_DIR        cxxopts
    GIT_REPOSITORY    https://github.com/jarro2783/cxxopts.git
    GIT_TAG           44380e5
    GIT_SHALLOW       TRUE
    GIT_PROGRESS      TRUE
    GIT_SUBMODULES    ""
)
FetchContent_MakeAvailable(cxxopts)
find_path(CXXOPTS_INCLUDE_DIR
    NAMES cxxopts.hpp
    HINTS "${cxxopts_SOURCE_DIR}/include/"
    REQUIRED)
target_include_directories(bcp2-mapf PRIVATE ${CXXOPTS_INCLUDE_DIR})

# Download tracy.
if(CMAKE_BUILD_TYPE STREQUAL "Profile")
    FetchContent_Declare(tracy
        SOURCE_DIR        tracy
        GIT_REPOSITORY    https://github.com/wolfpld/tracy.git
        GIT_TAG           v0.12.2
        GIT_SHALLOW       TRUE
        GIT_PROGRESS      TRUE
        GIT_SUBMODULES    ""
    )
    FetchContent_MakeAvailable(tracy)
    find_path(TRACY_INCLUDE_DIR
        NAMES tracy/profiler/build/tracy-profiler
        HINTS "."
        OPTIONAL)
    target_include_directories(bcp2-mapf PRIVATE ${TRACY_INCLUDE_DIR})
    target_link_libraries(bcp2-mapf PRIVATE Tracy::TracyClient)
    target_compile_options(bcp2-mapf PRIVATE -DTRACY_ENABLE)
    option(TRACY_ENABLE "" ON)
    option(TRACY_ON_DEMAND "" ON)
endif ()

# Set general options.
target_compile_options(bcp2-mapf PRIVATE -DRANDOM_SEED=1234)

# Set separator options.
# target_compile_options(bcp2-mapf PRIVATE -DUSE_RECTANGLE_CLIQUE_CUTS)

# Set pricer options.
# target_compile_options(bcp2-mapf PRIVATE -DUSE_INDEPENDENT_TIME_EXPANDED_ASTAR_PRICER)
# target_compile_options(bcp2-mapf PRIVATE -DUSE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER)
# target_compile_options(bcp2-mapf PRIVATE -DUSE_SHARED_TIME_EXPANDED_ASTAR_PRICER)
target_compile_options(bcp2-mapf PRIVATE -DUSE_SHARED_TIME_INTERVAL_ASTAR_PRICER)
target_compile_options(bcp2-mapf PRIVATE -DUSE_RESERVATION_LOOKUP)

# Set warnings.
target_compile_options(bcp2-mapf PRIVATE
                       -Wall
                       -Wextra
                       -Wignored-qualifiers
                       -Werror=return-type
                       -Wno-sign-compare
                       -Wno-narrowing)

# Set flags.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_COMPILER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_LINKER_FLAGS}")
target_compile_options(bcp2-mapf PRIVATE -march=native)

# Set other settings for different build types.
message("Build type ${CMAKE_BUILD_TYPE}")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(bcp2-mapf PRIVATE -DDEBUG)
else ()
    target_compile_options(bcp2-mapf PRIVATE -O3 -g -DNDEBUG -funroll-loops)
endif ()
if(CMAKE_BUILD_TYPE STREQUAL "ASan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
endif ()
if(CMAKE_BUILD_TYPE STREQUAL "UBSan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
endif ()

# Write debug symbols for a few files.
# set_source_files_properties(bcp/master/master.cpp PROPERTIES COMPILE_FLAGS "-O0 -g -DDEBUG")
# set_source_files_properties(bcp/master/gurobi_lp.cpp PROPERTIES COMPILE_FLAGS "-O0 -g -DDEBUG")

# Turn on link-time optimization for Linux.
if (CMAKE_BUILD_TYPE STREQUAL "Release" AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif ()
