/*
This file is part of BCP2-MAPF. BCP2-MAPF is licensed under the PolyForm
Noncommercial License 1.0.0. A copy of this license can found in LICENSE.md.

Author: Edward Lam <ed@ed-lam.com>
*/

#pragma once

#include "types/basic_types.h"
#include <random>

using RandomNumberGenerator = std::mt19937;
using UniformDistribution = std::uniform_real_distribution<Float>;
using DiscreteDistribution = std::discrete_distribution<UInt32>;
