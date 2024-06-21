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

#pragma once

#include <inttypes.h>

namespace smc_storm::state_properties {

typedef uint8_t StateInfoType;

/*!
 * @brief Keep a list of constexpr values that represent state properties. A state can get more than one value assigned.
 */
namespace state_info {
constexpr StateInfoType NO_INFO = 0U;          // The state does not match with any of the State Infos  below
constexpr StateInfoType IS_TERMINAL = 1U;      // The state is Terminal (no action brings to a different state)
constexpr StateInfoType BREAK_CONDITION = 2U;  // The state doesn't satisfy the condition formula
constexpr StateInfoType SATISFY_TARGET = 4U;   // The state satisfies the target formula

inline bool checkNoInfo(const StateInfoType info) {
    return info == NO_INFO;
}

inline bool checkIsTerminal(const StateInfoType info) {
    return (info & IS_TERMINAL) != 0U;
}

inline bool checkBreakCondition(const StateInfoType info) {
    return (info & BREAK_CONDITION) != 0U;
}

inline bool checkSatisfyTarget(const StateInfoType info) {
    return (info & SATISFY_TARGET) != 0U;
}
}  // namespace state_info
}  // namespace smc_storm::state_properties
