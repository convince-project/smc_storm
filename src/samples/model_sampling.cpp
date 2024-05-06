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

#include <chrono>

#include <storm/settings/SettingsManager.h>
#include <storm/settings/modules/CoreSettings.h>
#include <storm/settings/modules/ExplorationSettings.h>

#include <storm/utility/macros.h>

#include "samples/exploration_information.h"
#include "samples/model_sampling.h"

namespace smc_storm::samples {

template<typename StateType, typename ValueType>
ModelSampling<StateType, ValueType>::ModelSampling()
: randomGenerator(std::chrono::system_clock::now().time_since_epoch().count()),
  // TODO: Check if it makes sense to make this entry configurable. For now it isn't required
  comparator(1e-06) {
    // Nothing to do
}

template<typename StateType, typename ValueType>
ModelSampling<StateType, ValueType>::ModelSampling(uint_fast32_t const& seed)
: randomGenerator(seed),
  // TODO: Check if it makes sense to make this entry configurable. For now it isn't required
  comparator(1e-06) {
    // Nothing to do
}

// Methods used by Statistical Model Checking Engine
template<typename StateType, typename ValueType>
StateType ModelSampling<StateType, ValueType>::sampleActionOfState(
    StateType const& currentStateId, ExplorationInformation<StateType, ValueType> const& explorationInformation) const {
    // TODO: For now we pick a random action leaving the current state, with uniform probability
    // For DTMCs this will make no difference (we only have one leaving action), for MDPs we can implement smarter strategies later on
    const size_t rowGroupId = explorationInformation.getRowGroup(currentStateId);
    const ActionType firstActionId = explorationInformation.getStartRowOfGroup(rowGroupId);
    const size_t nActions = explorationInformation.getRowGroupSize(rowGroupId);
    if (nActions == 1U) {
        return firstActionId;
    }
    std::uniform_int_distribution<ActionType> distribution(firstActionId, firstActionId + nActions - 1U);
    return distribution(randomGenerator);
}

template<typename StateType, typename ValueType>
StateType ModelSampling<StateType, ValueType>::sampleSuccessorFromAction(
    ActionType const& chosenAction, ExplorationInformation<StateType, ValueType> const& explorationInformation) const {
    std::vector<storm::storage::MatrixEntry<StateType, ValueType>> const& row = explorationInformation.getRowOfMatrix(chosenAction);
    if (row.size() == 1) {
        return row.front().getColumn();
    }
    std::vector<ValueType> probabilities(row.size());
    std::transform(row.begin(), row.end(), probabilities.begin(),
                   [](storm::storage::MatrixEntry<StateType, ValueType> const& entry) { return entry.getValue(); });
    std::discrete_distribution<StateType> distribution(probabilities.begin(), probabilities.end());
        return row[distribution(randomGenerator)].getColumn();
}

template class ModelSampling<uint32_t, double>;

}  // namespace smc_storm::samples
