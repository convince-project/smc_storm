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

#include <storm/exceptions/IllegalFunctionCallException.h>
#include <storm/utility/macros.h>

#include "state_generation/exploration_information.hpp"

namespace smc_storm::state_generation {

template <typename StateType, typename ValueType>
ExplorationInformation<StateType, ValueType>::ExplorationInformation() {
    // Do nothing
}

template <typename StateType, typename ValueType>
const state_properties::StateDescription<StateType, ValueType>&
ExplorationInformation<StateType, ValueType>::ExplorationInformation::getStateDescription(const StateType& state_id) const {
    const auto states_vect_idx = _state_to_vect_idx.at(state_id);
    STORM_LOG_THROW(
        states_vect_idx != UNEXPLORED_MARKER, storm::exceptions::IllegalFunctionCallException,
        "The state " << state_id << " has not been explored yet.");
    return *_state_description_ptrs.at(states_vect_idx);
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addStateDescription(
    const StateType& state_id, std::unique_ptr<state_properties::StateDescription<StateType, ValueType>> state_description_ptr) {
    if (state_id >= _state_to_vect_idx.size()) {
        _state_to_vect_idx.resize(state_id + 1, UNEXPLORED_MARKER);
    }
    STORM_LOG_THROW(
        _state_to_vect_idx[state_id] == UNEXPLORED_MARKER, storm::exceptions::IllegalFunctionCallException,
        "The state " << state_id << " has already been explored.");
    const auto state_vect_idx = _state_description_ptrs.size();
    _state_to_vect_idx[state_id] = state_vect_idx;
    _state_description_ptrs.emplace_back(std::move(state_description_ptr));
}

template <typename StateType, typename ValueType>
typename ExplorationInformation<StateType, ValueType>::const_iterator ExplorationInformation<StateType, ValueType>::findUnexploredState(
    const StateType& state) const {
    return _unexplored_states.find(state);
}

template <typename StateType, typename ValueType>
typename ExplorationInformation<StateType, ValueType>::const_iterator ExplorationInformation<StateType, ValueType>::unexploredStatesEnd()
    const {
    return _unexplored_states.end();
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::removeUnexploredState(const_iterator it) {
    _unexplored_states.erase(it);
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addUnexploredState(
    const StateType& state_id, const storm::generator::CompressedState& compressed_state) {
    if (state_id >= _state_to_vect_idx.size()) {
        _state_to_vect_idx.resize(state_id + 1, UNEXPLORED_MARKER);
    }
    _unexplored_states[state_id] = compressed_state;
}

template class ExplorationInformation<uint32_t, double>;
}  // namespace smc_storm::state_generation
