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

#include <storm/settings/modules/CoreSettings.h>
#include <storm/settings/SettingsManager.h>

#include <storm/exceptions/IllegalFunctionCallException.h>
#include <storm/utility/macros.h>

#include "samples/model_sampling.hpp"

namespace smc_storm::samples {

template <typename StateType, typename ValueType>
ModelSampling<StateType, ValueType>::ModelSampling()
    : _random_generator(std::chrono::system_clock::now().time_since_epoch().count()),
      // TODO: Check if it makes sense to make this entry configurable. For now it isn't required
      _comparator(1e-06) {
    // Nothing to do
}

template <typename StateType, typename ValueType>
ModelSampling<StateType, ValueType>::ModelSampling(const uint_fast32_t& seed)
    : _random_generator(seed),
      // TODO: Check if it makes sense to make this entry configurable. For now it isn't required
      _comparator(1e-06) {
    // Nothing to do
}

// Methods used by Statistical Model Checking Engine
template <typename StateType, typename ValueType>
typename ModelSampling<StateType, ValueType>::ActionType ModelSampling<StateType, ValueType>::sampleActionOfState(
    const state_properties::StateDescription<StateType, ValueType>& current_state) const {
    // TODO: For now we pick a random action leaving the current state, with
    // uniform probability For DTMCs this will make no difference (we only have
    // one leaving action), for MDPs we can implement smarter strategies later on
    const size_t n_actions = current_state.getActions().size();
    STORM_LOG_THROW(n_actions > 0U, storm::exceptions::IllegalFunctionCallException, "No actions available in the current state.");
    if (n_actions == 1U) {
        return 0U;
    }
    return std::uniform_int_distribution<ActionType>(0U, n_actions - 1U)(_random_generator);
}

template <typename StateType, typename ValueType>
StateType ModelSampling<StateType, ValueType>::sampleSuccessorFromAction(
    const ActionType& chosenAction, const state_properties::StateDescription<StateType, ValueType>& current_state) const {
    // Randomly select a target state, based on the transition probabilities
    const auto& action_targets = current_state.getActions()[chosenAction].first;
    if (action_targets.size() == 1) {
        return action_targets.front().second;
    }
    std::vector<ValueType> probabilities(action_targets.size());
    std::transform(action_targets.begin(), action_targets.end(), probabilities.begin(), [](const std::pair<ValueType, StateType>& entry) {
        return entry.first;
    });
    std::discrete_distribution<size_t> distribution(probabilities.begin(), probabilities.end());
    return action_targets[distribution(_random_generator)].second;
}

template class ModelSampling<uint32_t, double>;
template class ModelSampling<storm::generator::CompressedState, double>;

}  // namespace smc_storm::samples
