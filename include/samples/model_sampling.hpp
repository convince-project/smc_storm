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

#include <random>
#include <utility>
#include <vector>

#include <storm/utility/ConstantsComparator.h>

namespace smc_storm::samples {

template <typename StateType, typename ValueType>
class ExplorationInformation;

template <typename StateType, typename ValueType>
class ModelSampling {
  public:
    typedef StateType ActionType;
    ModelSampling();
    ModelSampling(const uint_fast32_t& seed);

    /*!
     * @brief Sample a single action leaving the current state depending on the sampling strategy (for StatisticalMC)
     * @param current_state_id The state ID the action must start from
     * @param exploration_information Model-related data, associating state and action IDs together
     * @return The sampled action ID
     */
    ActionType sampleActionOfState(
        const StateType& current_state_id, const ExplorationInformation<StateType, ValueType>& exploration_information) const;

    /*!
     * @brief Sample the state that will be reached by selecting a specific action (for StatisticalMC)
     * @param chosenAction The ID of the selected action
     * @param exploration_information Model-related data, associating state and action IDs together
     * @return The sampled state ID
     */
    StateType sampleSuccessorFromAction(
        const ActionType& chosenAction, const ExplorationInformation<StateType, ValueType>& exploration_information) const;

  private:
    // A random number generator
    mutable std::default_random_engine _random_generator;
    const storm::utility::ConstantsComparator<ValueType> _comparator;
};

}  // namespace smc_storm::samples
