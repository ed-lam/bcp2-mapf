/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#include "types/arena.h"
#include "types/tracy.h"

#define TRACY_COLOUR tracy::Color::ColorType::WhiteSmoke

Arena::Arena() :
    first_(),
    head_(),
    offset_(),
    object_size_()
{
    reset(1);
}

Arena::Arena(Arena&& other) noexcept :
    first_(std::move(other.first_)),
    head_(other.head_ == &other.first_ ? &first_ : other.head_),
    offset_(other.offset_),
    object_size_(other.object_size_)
{
}

Arena& Arena::operator=(Arena&& other) noexcept
{
    first_ = std::move(other.first_);
    head_ = other.head_ == &other.first_ ? &first_ : other.head_;
    offset_ = other.offset_;
    object_size_ = other.object_size_;
    return *this;
}

void Arena::reset(const Size64 object_size)
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(object_size > 0);
    DEBUG_ASSERT(object_size <= Chunk::chunk_size);
    head_ = &first_;
    offset_ = 0;
    object_size_ = ((object_size + 7) & (-8)); // Round up to next multiple of 8
}

template <Bool commit, Bool zero_out>
Byte* Arena::get_buffer()
{
    ZoneScopedC(TRACY_COLOUR);

    // Advance to the next chunk. Allocate if required.
    DEBUG_ASSERT(object_size_ > 0);
    if (offset_ + object_size_ > Chunk::chunk_size)
    {
        if (head_->next)
        {
            head_ = head_->next.get();
        }
        else
        {
            head_->next = std::make_unique<Chunk>();
            ASSERT(head_->next, "Failed to allocate chunk of memory in arena");
            head_ = head_->next.get();
        }
        offset_ = 0;
    }
    auto ptr = head_->mem.get() + offset_;

    // Zero out the memory if needed.
    if constexpr (zero_out)
    {
        std::memset(ptr, 0, object_size_);
    }

    // Commit the buffer if needed.
    if constexpr (commit)
    {
        commit_buffer();
    }

    return ptr;
}

template Byte* Arena::get_buffer<true, true>();
template Byte* Arena::get_buffer<true, false>();
template Byte* Arena::get_buffer<false, true>();
template Byte* Arena::get_buffer<false, false>();

void Arena::commit_buffer()
{
    ZoneScopedC(TRACY_COLOUR);

    DEBUG_ASSERT(offset_ + object_size_ <= Chunk::chunk_size);
    offset_ += object_size_;
}
