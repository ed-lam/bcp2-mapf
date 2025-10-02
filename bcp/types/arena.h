/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include "types/debug.h"
#include "types/pointers.h"
#include <new>

struct Chunk
{
    static constexpr Size64 chunk_size = 1024 * 1024; // 1 MiB

    UniquePtr<Byte[]> mem;
    UniquePtr<Chunk> next;

    Chunk() :
        mem(std::make_unique_for_overwrite<Byte[]>(chunk_size)), // Throws if failure
        next()
    {
    }
    ~Chunk() = default;
    Chunk(const Chunk&) = delete;
    Chunk(Chunk&&) noexcept = default;
    Chunk& operator=(const Chunk&) = delete;
    Chunk& operator=(Chunk&&) noexcept = default;
};

class Arena
{
    Chunk first_;        // First chunk of memory
    Chunk* head_;        // Current chunk of memory
    Size64 offset_;      // Index to allocate in current chunk
    Size64 object_size_; // Size for the next allocation

  public:
    // Constructors and destructor
    Arena();
    ~Arena() = default;
    Arena(const Arena&) = delete;
    Arena(Arena&&) noexcept;
    Arena& operator=(const Arena&) = delete;
    Arena& operator=(Arena&&) noexcept;

    // Getters
    inline auto object_size() const
    {
        return object_size_;
    }

    // Get a memory allocation to store an object
    template <Bool commit, Bool zero_out>
    Byte* get_buffer();

    // Commit the memory allocation
    void commit_buffer();

    // Clear all storage
    void reset(const Size64 object_size);

  private:
    // Allocate
    void allocate_blocks();
};
