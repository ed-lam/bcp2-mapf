/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#ifdef TRACY_ENABLE

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <tracy/Tracy.hpp>
#pragma GCC diagnostic pop

void* operator new(std::size_t size);
void operator delete(void* ptr) noexcept;

#else

#define ZoneScoped(...)
#define ZoneScopedC(...)
#define ZoneScopedNC(...)
#define ZoneValue(...)

#endif
