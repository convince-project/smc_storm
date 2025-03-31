/*
 * Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries
 *
 * This file is part of smc_storm.
 *
 * smc_storm is free software: you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * smc_storm is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with smc_storm.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <utility>
#include <vector>

namespace smc_storm::state_generation {
// A pair containing the action ID and the related reward
template <typename ValueType>
using ActionIdAndReward = std::pair<uint64_t, ValueType>;

template <typename ValueType>
using AvailableActions = std::vector<ActionIdAndReward<ValueType>>;
}  // namespace smc_storm::state_generation
