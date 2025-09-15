/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include <fmt/format.h>
#include <fmt/ranges.h>

// clang-format off

#ifdef DEBUG
#define PRINTLN(format, ...) do { fmt::print(format "\n", ##__VA_ARGS__); fflush(stdout); } while (false)
#else
#define PRINTLN(format, ...) do { fmt::print(format "\n", ##__VA_ARGS__); } while (false)
#endif

#ifdef PRINT_DEBUG
#define DEBUGLN(format, ...) PRINTLN(format, ##__VA_ARGS__)
#else
#define DEBUGLN(format, ...)
#endif

#ifdef DEBUG
#define ERROR(format, ...) do {                                \
    fmt::print(stderr, "Error: " format "\n", ##__VA_ARGS__);  \
    fmt::print(stderr, "Function: {}\n", __PRETTY_FUNCTION__); \
    fmt::print(stderr, "File: {}\n", __FILE__);                \
    fmt::print(stderr, "Line: {}\n", __LINE__);                \
    fflush(stderr);                                            \
    std::abort();                                              \
} while (false)
#else
#define ERROR(format, ...) do {                               \
    fmt::print(stderr, "Error: " format "\n", ##__VA_ARGS__); \
    std::abort();                                             \
} while (false)
#endif

#define DEBUG_SEP(...) DEBUGLN("====================================================================================================")
#define PRINT_SEP(...) PRINTLN("====================================================================================================")

#define ASSERT(condition, ...) do { if (!(condition)) { ERROR(__VA_ARGS__); } } while (false)

#ifdef DEBUG
#define DEBUG_ASSERT(...) ASSERT((__VA_ARGS__), "{}", #__VA_ARGS__)
#else
#define DEBUG_ASSERT(...)
#endif

#define NOT_YET_IMPLEMENTED() do { \
    ERROR("Reached unimplemented section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#define NOT_YET_CHECKED() do { \
    ERROR("Reached unchecked section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)

#define UNREACHABLE() do { \
    ERROR("Reached unreachable section in function \"{}\"", __PRETTY_FUNCTION__); \
} while (false)
