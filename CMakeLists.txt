cmake_minimum_required(VERSION 3.21)
project(bcp2-mapf)

# Verbose.
# set(CMAKE_VERBOSE_MAKEFILE on)

# Set C++ version.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

# Set release build.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

# Set source files.
set(BCP_MAPF_SOURCE_FILES
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
    bcp/master/gurobi_lp.cpp
    bcp/master/master.cpp
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
    bcp/types/memory_pool.cpp
    bcp/types/tracy.cpp
    )

# Find Gurobi binary.
find_path(GUROBI_INCLUDE_DIR
    NAMES gurobi_c.h
    HINTS "${GUROBI_DIR}/include" "${GUROBI_DIR}/*/include"
          "$ENV{GUROBI_DIR}/include" "$ENV{GUROBI_DIR}/*/include"
          "$ENV{GUROBI_HOME}/include"
    REQUIRED)
find_library(GUROBI_LIBRARY
    NAMES gurobi120 gurobi110
    HINTS "${GUROBI_DIR}/lib" "${GUROBI_DIR}/*/lib"
          "$ENV{GUROBI_DIR}/lib" "$ENV{GUROBI_DIR}/*/lib"
          "$ENV{GUROBI_HOME}/lib"
    REQUIRED)
message("Found Gurobi at ${GUROBI_LIBRARY} and ${GUROBI_INCLUDE_DIR}")

# Find boost.
# find_path(BOOST_INCLUDE_DIR
#     NAMES boost/heap/d_ary_heap.hpp
#     HINTS "${BOOST_DIR}" "$ENV{BOOST_DIR}")
# if (NOT BOOST_INCLUDE_DIR)

    # Download boost if not found.
    set(BOOST_VERSION 1.88.0)
    message(STATUS "Downloading Boost libraries")
    include(FetchContent)
    # Set(FETCHCONTENT_QUIET FALSE)
    FetchContent_Declare(boost
        URL https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-b2-nodocs.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(boost)
    message("Downloaded Boost to ${boost_SOURCE_DIR}")
    set(BOOST_INCLUDE_DIR "${boost_SOURCE_DIR}")

    # Confirm boost is found.
    find_path(BOOST_INCLUDE_DIR
        NAMES boost/heap/d_ary_heap.hpp
        HINTS "${BOOST_INCLUDE_DIR}"
        REQUIRED)

# endif ()
message("Found Boost libraries at ${BOOST_INCLUDE_DIR}")

# Find tracy.
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    find_path(TRACY_DIR
        NAMES tracy/profiler/build/tracy-profiler
        HINTS "."
        OPTIONAL)
    if (TRACY_DIR)
        message("Found tracy profiler at ${TRACY_DIR}")
        option(TRACY_ENABLE "" ON)
        option(TRACY_ON_DEMAND "" ON)
        add_subdirectory(tracy EXCLUDE_FROM_ALL)
    endif ()
endif ()

# Declare executables.
add_executable(bcp2-mapf ${BCP_MAPF_SOURCE_FILES})
target_include_directories(bcp2-mapf PRIVATE ./ bcp/)
if (TRACY_ENABLE)
add_executable(bcp2-mapf-tracy ${BCP_MAPF_SOURCE_FILES})
target_include_directories(bcp2-mapf-tracy PRIVATE ./ bcp/)
endif ()

# Include Gurobi.
target_include_directories(bcp2-mapf PRIVATE ${GUROBI_INCLUDE_DIR})
target_link_libraries(bcp2-mapf ${GUROBI_LIBRARY})
if (TRACY_ENABLE)
target_include_directories(bcp2-mapf-tracy PRIVATE ${GUROBI_INCLUDE_DIR})
target_link_libraries(bcp2-mapf-tracy ${GUROBI_LIBRARY})
endif ()

# Include fmt.
add_subdirectory(fmt EXCLUDE_FROM_ALL)
target_include_directories(bcp2-mapf PRIVATE fmt/include/)
target_link_libraries(bcp2-mapf fmt::fmt-header-only)
if (TRACY_ENABLE)
target_include_directories(bcp2-mapf-tracy PRIVATE fmt/include/)
target_link_libraries(bcp2-mapf-tracy fmt::fmt-header-only)
endif ()

# Include ranges.
target_include_directories(bcp2-mapf PRIVATE range-v3/include/)
if (TRACY_ENABLE)
target_include_directories(bcp2-mapf-tracy PRIVATE range-v3/include/)
endif ()

# Include cxxopts.
target_include_directories(bcp2-mapf PRIVATE cxxopts/include/)
if (TRACY_ENABLE)
target_include_directories(bcp2-mapf-tracy PRIVATE cxxopts/include/)
endif ()

# Include boost.
target_include_directories(bcp2-mapf PRIVATE ${BOOST_INCLUDE_DIR})
if (TRACY_ENABLE)
target_include_directories(bcp2-mapf-tracy PRIVATE ${BOOST_INCLUDE_DIR})
endif ()

# Include tracy.
if (TRACY_ENABLE)
target_include_directories(bcp2-mapf-tracy PUBLIC tracy/public/)
target_link_libraries(bcp2-mapf-tracy Tracy::TracyClient)
target_compile_options(bcp2-mapf-tracy PRIVATE -DTRACY_ENABLE)
endif ()

# Set general options.
target_compile_options(bcp2-mapf PRIVATE -DRANDOM_SEED=1234)
if (TRACY_ENABLE)
target_compile_options(bcp2-mapf-tracy PRIVATE -DRANDOM_SEED=1234)
endif ()

# Set separator options.
# target_compile_options(bcp2-mapf PRIVATE -DUSE_RECTANGLE_CLIQUE_CUTS)
if (TRACY_ENABLE)
# target_compile_options(bcp2-mapf-tracy PRIVATE -DUSE_RECTANGLE_CLIQUE_CUTS)
endif ()

# Set pricer options.
# target_compile_options(bcp2-mapf PRIVATE -DUSE_INDEPENDENT_TIME_EXPANDED_ASTAR_PRICER)
# target_compile_options(bcp2-mapf PRIVATE -DUSE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER)
# target_compile_options(bcp2-mapf PRIVATE -DUSE_SHARED_TIME_EXPANDED_ASTAR_PRICER)
target_compile_options(bcp2-mapf PRIVATE -DUSE_SHARED_TIME_INTERVAL_ASTAR_PRICER)
target_compile_options(bcp2-mapf PRIVATE -DUSE_RESERVATION_LOOKUP)
if (TRACY_ENABLE)
# target_compile_options(bcp2-mapf-tracy PRIVATE -DUSE_INDEPENDENT_TIME_EXPANDED_ASTAR_PRICER)
# target_compile_options(bcp2-mapf-tracy PRIVATE -DUSE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER)
# target_compile_options(bcp2-mapf-tracy PRIVATE -DUSE_SHARED_TIME_EXPANDED_ASTAR_PRICER)
target_compile_options(bcp2-mapf-tracy PRIVATE -DUSE_SHARED_TIME_INTERVAL_ASTAR_PRICER)
target_compile_options(bcp2-mapf-tracy PRIVATE -DUSE_RESERVATION_LOOKUP)
endif ()

# Set warnings.
target_compile_options(bcp2-mapf PRIVATE
                       -Wall
                       -Wextra
                       -Wignored-qualifiers
                       -Werror=return-type
                       -Wno-sign-compare
                       -Wno-narrowing)
if (TRACY_ENABLE)
target_compile_options(bcp2-mapf-tracy PRIVATE
                       -Wall
                       -Wextra
                       -Wignored-qualifiers
                       -Werror=return-type
                       -Wno-sign-compare
                       -Wno-narrowing)
endif ()

# Set flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_COMPILER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_LINKER_FLAGS}")
target_compile_options(bcp2-mapf PRIVATE -march=native)
if (TRACY_ENABLE)
target_compile_options(bcp2-mapf-tracy PRIVATE -march=native)
endif ()
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(bcp2-mapf PRIVATE -DDEBUG -DCHECK_USING_ASTAR) # -D_GLIBCXX_DEBUG -DLIBCXX_ENABLE_DEBUG_MODE
    # target_compile_options(bcp2-mapf PRIVATE -DDEBUG -DCHECK_USING_TIASTAR) # -D_GLIBCXX_DEBUG -DLIBCXX_ENABLE_DEBUG_MODE
    message("Configured in debug mode")
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(bcp2-mapf PRIVATE -Og -DNDEBUG)
    message("Configured in release mode with debug info")
else ()
    target_compile_options(bcp2-mapf PRIVATE -O3 -g -DNDEBUG -funroll-loops)
    if (TRACY_ENABLE)
    target_compile_options(bcp2-mapf-tracy PRIVATE -O3 -g -DNDEBUG -funroll-loops)
    endif ()
    message("Configured in release mode")
endif ()

# Write debug symbols for a few files.
# set_source_files_properties(bcp/master/master.cpp PROPERTIES COMPILE_FLAGS "-g -O0 -DDEBUG")
# set_source_files_properties(bcp/master/gurobi_lp.cpp PROPERTIES COMPILE_FLAGS "-g -O0 -DDEBUG")

# Use address sanitizer.
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

# Use undefined behavior sanitizer.
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")

# Turn on link-time optimization for Linux.
if (CMAKE_BUILD_TYPE STREQUAL "Release" AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif ()
