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

#include "samples/exploration_information.hpp"

namespace smc_storm::samples {

template <typename StateType, typename ValueType>
ExplorationInformation<StateType, ValueType>::ExplorationInformation(const bool store_expanded_states)
    : _store_expanded_states(store_expanded_states) {
    // Do nothing
}

template <typename StateType, typename ValueType>
const storm::generator::CompressedState& ExplorationInformation<StateType, ValueType>::getCompressedState(const StateType& state) const {
    STORM_LOG_THROW(_store_expanded_states, storm::exceptions::IllegalFunctionCallException, "Compressed state storage is not enabled.");
    return _state_to_compressed_state.at(state);
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
    _state_to_row_group_mapping.push_back(_unexplored_marker);
    _unexplored_states[state_id] = compressed_state;
    if (_store_expanded_states) {
        _state_to_compressed_state.push_back(compressed_state);
    }
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::assignStateToRowGroup(const StateType& state, const ActionType& row_group) {
    _state_to_row_group_mapping[state] = row_group;
}

template <typename StateType, typename ValueType>
StateType ExplorationInformation<StateType, ValueType>::assignStateToNextRowGroup(const StateType& state) {
    _state_to_row_group_mapping[state] = row_group_indices.size() - 1;
    return _state_to_row_group_mapping[state];
}

template <typename StateType, typename ValueType>
StateType ExplorationInformation<StateType, ValueType>::getNextRowGroup() const {
    return row_group_indices.size() - 1;
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::newRowGroup(const ActionType& action) {
    row_group_indices.push_back(action);
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::newRowGroup() {
    newRowGroup(_action_to_target_states.size());
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::terminateCurrentRowGroup() {
    row_group_indices.push_back(_action_to_target_states.size());
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::moveActionToBackOfMatrix(const ActionType& action) {
    _action_to_target_states.emplace_back(std::move(_action_to_target_states[action]));
}

template <typename StateType, typename ValueType>
StateType ExplorationInformation<StateType, ValueType>::getActionCount() const {
    return _action_to_target_states.size();
}

template <typename StateType, typename ValueType>
size_t ExplorationInformation<StateType, ValueType>::getNumberOfUnexploredStates() const {
    return _unexplored_states.size();
}

template <typename StateType, typename ValueType>
size_t ExplorationInformation<StateType, ValueType>::getNumberOfDiscoveredStates() const {
    return _state_to_row_group_mapping.size();
}

template <typename StateType, typename ValueType>
const StateType& ExplorationInformation<StateType, ValueType>::getRowGroup(const StateType& state) const {
    return _state_to_row_group_mapping[state];
}

template <typename StateType, typename ValueType>
const StateType& ExplorationInformation<StateType, ValueType>::getUnexploredMarker() const {
    return _unexplored_marker;
}

template <typename StateType, typename ValueType>
bool ExplorationInformation<StateType, ValueType>::isUnexplored(const StateType& state) const {
    return _state_to_row_group_mapping[state] == _unexplored_marker;
}

template <typename StateType, typename ValueType>
state_properties::StateInfoType ExplorationInformation<StateType, ValueType>::getStateInfo(const StateType& state) const {
    const auto& state_info_it = _states_info.find(state);
    if (state_info_it != _states_info.end()) {
        return state_info_it->second;
    }
    return state_properties::state_info::NO_INFO;
}

template <typename StateType, typename ValueType>
const ValueType& ExplorationInformation<StateType, ValueType>::getStateReward(const StateType& state_id) const {
    return _state_to_reward.at(state_id);
}

template <typename StateType, typename ValueType>
bool ExplorationInformation<StateType, ValueType>::isTerminal(const StateType& state) const {
    const auto stateInfo = getStateInfo(state);
    return state_properties::state_info::checkIsTerminal(stateInfo);
}

template <typename StateType, typename ValueType>
const typename ExplorationInformation<StateType, ValueType>::ActionType& ExplorationInformation<StateType, ValueType>::getStartRowOfGroup(
    const StateType& group) const {
    return row_group_indices[group];
}

template <typename StateType, typename ValueType>
size_t ExplorationInformation<StateType, ValueType>::getRowGroupSize(const StateType& group) const {
    return row_group_indices[group + 1] - row_group_indices[group];
}

template <typename StateType, typename ValueType>
bool ExplorationInformation<StateType, ValueType>::onlyOneActionAvailable(const StateType& group) const {
    return getRowGroupSize(group) == 1;
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addStateInfo(const StateType& state, const StateInfoType info) {
    if (state_properties::state_info::checkNoInfo(info)) {
        _states_info.erase(state);
    }
    _states_info[state] = info;
    auto state_info_it = _states_info.find(state);
    if (state_info_it == _states_info.end()) {
        if (!state_properties::state_info::checkNoInfo(info)) {
            _states_info.insert({state, info});
        }
    } else {
        state_info_it->second = state_info_it->second | info;
    }
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addStateReward(const StateType& state_id, const ValueType& state_reward) {
    if (_state_to_reward.size() <= state_id) {
        _state_to_reward.resize(state_id + 1U, std::numeric_limits<ValueType>::infinity());
    }
    _state_to_reward.at(state_id) = state_reward;
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addTerminalState(const StateType& state) {
    addStateInfo(state, state_properties::state_info::IS_TERMINAL);
}

template <typename StateType, typename ValueType>
std::vector<storm::storage::MatrixEntry<StateType, ValueType>>& ExplorationInformation<StateType, ValueType>::getRowOfMatrix(
    const ActionType& row) {
    return _action_to_target_states[row];
}

template <typename StateType, typename ValueType>
const std::vector<storm::storage::MatrixEntry<StateType, ValueType>>& ExplorationInformation<StateType, ValueType>::getRowOfMatrix(
    const ActionType& row) const {
    return _action_to_target_states[row];
}

template <typename StateType, typename ValueType>
const ValueType& ExplorationInformation<StateType, ValueType>::getActionReward(const ActionType& action_id) const {
    return _action_to_reward.at(action_id);
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addActionsToMatrix(const size_t& count) {
    _action_to_target_states.resize(_action_to_target_states.size() + count);
}

template <typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addActionReward(const ActionType& action_id, const ValueType& action_reward) {
    if (_action_to_reward.size() <= action_id) {
        _action_to_reward.resize(_action_to_target_states.size(), std::numeric_limits<ValueType>::infinity());
    }
    _action_to_reward.at(action_id) = action_reward;
}

template class ExplorationInformation<uint32_t, double>;
}  // namespace smc_storm::samples
