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

#include "samples/exploration_information.hpp"
#include "samples/model_sampling.hpp"

namespace smc_storm::samples {

template<typename StateType, typename ValueType>
ModelSampling<StateType, ValueType>::ModelSampling()
: _random_generator(std::chrono::system_clock::now().time_since_epoch().count()),
  // TODO: Check if it makes sense to make this entry configurable. For now it isn't required
  _comparator(1e-06) {
    // Nothing to do
}

template<typename StateType, typename ValueType>
ModelSampling<StateType, ValueType>::ModelSampling(uint_fast32_t const& seed)
: _random_generator(seed),
  // TODO: Check if it makes sense to make this entry configurable. For now it isn't required
  _comparator(1e-06) {
    // Nothing to do
}

// Methods used by Statistical Model Checking Engine
template<typename StateType, typename ValueType>
StateType ModelSampling<StateType, ValueType>::sampleActionOfState(
    StateType const& current_state_id, ExplorationInformation<StateType, ValueType> const& exploration_information) const {
    // TODO: For now we pick a random action leaving the current state, with uniform probability
    // For DTMCs this will make no difference (we only have one leaving action), for MDPs we can implement smarter strategies later on
    const size_t row_group_id = exploration_information.getRowGroup(current_state_id);
    const ActionType first_action_id = exploration_information.getStartRowOfGroup(row_group_id);
    const size_t n_actions = exploration_information.getRowGroupSize(row_group_id);
    if (n_actions == 1U) {
        return first_action_id;
    }
    std::uniform_int_distribution<ActionType> distribution(first_action_id, first_action_id + n_actions - 1U);
    return distribution(_random_generator);
}

template<typename StateType, typename ValueType>
StateType ModelSampling<StateType, ValueType>::sampleSuccessorFromAction(
    ActionType const& chosen_action, ExplorationInformation<StateType, ValueType> const& exploration_information) const {
    std::vector<storm::storage::MatrixEntry<StateType, ValueType>> const& row = exploration_information.getRowOfMatrix(chosen_action);
    if (row.size() == 1) {
        return row.front().getColumn();
    }
    std::vector<ValueType> probabilities(row.size());
    std::transform(row.begin(), row.end(), probabilities.begin(),
                   [](storm::storage::MatrixEntry<StateType, ValueType> const& entry) { return entry.getValue(); });
    std::discrete_distribution<StateType> distribution(probabilities.begin(), probabilities.end());
        return row[distribution(_random_generator)].getColumn();
}

template class ModelSampling<uint32_t, double>;

}  // namespace smc_storm::samples
