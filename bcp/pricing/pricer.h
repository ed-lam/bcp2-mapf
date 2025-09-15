/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "pricing/independent_time_expanded_astar_pricer.h"
#include "pricing/independent_time_interval_astar_pricer.h"
#include "pricing/shared_time_expanded_astar_pricer.h"
#include "pricing/shared_time_interval_astar_pricer.h"

#ifdef USE_INDEPENDENT_TIME_EXPANDED_ASTAR_PRICER
using Pricer = IndependentTimeExpandedAStarPricer;
#endif

#ifdef USE_INDEPENDENT_TIME_INTERVAL_ASTAR_PRICER
using Pricer = IndependentTimeIntervalAStarPricer;
#endif

#ifdef USE_SHARED_TIME_EXPANDED_ASTAR_PRICER
using Pricer = SharedTimeExpandedAStarPricer;
#endif

#ifdef USE_SHARED_TIME_INTERVAL_ASTAR_PRICER
using Pricer = SharedTimeIntervalAStarPricer;
#endif
