/*
 * Copyright (c) 2025 Robert Bosch GmbH and its subsidiaries
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

#include <random>
#include <utility>
#include <vector>

#include "state_generation/available_actions.hpp"
#include <storm/utility/ConstantsComparator.h>

namespace smc_storm::state_generation {

template <typename ValueType>
class ActionScheduler {
  public:
    using ActionId = uint64_t;
    ActionScheduler(std::default_random_engine& random_generator);

    /*!
     * @brief Sample a single action among the provided ones depending on the sampling strategy (currently only uniform strategy)
     * @param current_state The current state to sample the next action from
     * @return The index of the next action to take
     */
    ActionId sampleAction(const AvailableActions<ValueType>& actions) const;

  private:
    // A random number generator
    mutable std::reference_wrapper<std::default_random_engine> _random_generator;
};

}  // namespace smc_storm::state_generation
