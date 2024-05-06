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

#include <storm/utility/macros.h>

#include "samples/exploration_information.h"

namespace smc_storm::samples {

template<typename StateType, typename ValueType>
ExplorationInformation<StateType, ValueType>::ExplorationInformation() {
    // Do nothing
}

template<typename StateType, typename ValueType>
typename ExplorationInformation<StateType, ValueType>::const_iterator ExplorationInformation<StateType, ValueType>::findUnexploredState(
    StateType const& state) const {
    return unexploredStates.find(state);
}

template<typename StateType, typename ValueType>
typename ExplorationInformation<StateType, ValueType>::const_iterator ExplorationInformation<StateType, ValueType>::unexploredStatesEnd() const {
    return unexploredStates.end();
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::removeUnexploredState(const_iterator it) {
    unexploredStates.erase(it);
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addUnexploredState(StateType const& stateId, storm::generator::CompressedState const& compressedState) {
    stateToRowGroupMapping.push_back(_unexplored_marker);
    unexploredStates[stateId] = compressedState;
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::assignStateToRowGroup(StateType const& state, ActionType const& rowGroup) {
    stateToRowGroupMapping[state] = rowGroup;
}

template<typename StateType, typename ValueType>
StateType ExplorationInformation<StateType, ValueType>::assignStateToNextRowGroup(StateType const& state) {
    stateToRowGroupMapping[state] = rowGroupIndices.size() - 1;
    return stateToRowGroupMapping[state];
}

template<typename StateType, typename ValueType>
StateType ExplorationInformation<StateType, ValueType>::getNextRowGroup() const {
    return rowGroupIndices.size() - 1;
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::newRowGroup(ActionType const& action) {
    rowGroupIndices.push_back(action);
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::newRowGroup() {
    newRowGroup(actionToTargetStates.size());
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::terminateCurrentRowGroup() {
    rowGroupIndices.push_back(actionToTargetStates.size());
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::moveActionToBackOfMatrix(ActionType const& action) {
    actionToTargetStates.emplace_back(std::move(actionToTargetStates[action]));
}

template<typename StateType, typename ValueType>
StateType ExplorationInformation<StateType, ValueType>::getActionCount() const {
    return actionToTargetStates.size();
}

template<typename StateType, typename ValueType>
size_t ExplorationInformation<StateType, ValueType>::getNumberOfUnexploredStates() const {
    return unexploredStates.size();
}

template<typename StateType, typename ValueType>
size_t ExplorationInformation<StateType, ValueType>::getNumberOfDiscoveredStates() const {
    return stateToRowGroupMapping.size();
}

template<typename StateType, typename ValueType>
StateType const& ExplorationInformation<StateType, ValueType>::getRowGroup(StateType const& state) const {
    return stateToRowGroupMapping[state];
}

template<typename StateType, typename ValueType>
StateType const& ExplorationInformation<StateType, ValueType>::getUnexploredMarker() const {
    return _unexplored_marker;
}

template<typename StateType, typename ValueType>
bool ExplorationInformation<StateType, ValueType>::isUnexplored(StateType const& state) const {
    return stateToRowGroupMapping[state] == _unexplored_marker;
}

template<typename StateType, typename ValueType>
properties::StateInfoType ExplorationInformation<StateType, ValueType>::getStateInfo(StateType const& state) const {
    const auto& stateInfoIt = statesInfo.find(state);
    if (stateInfoIt != statesInfo.end()) {
        return stateInfoIt->second;
    }
    return properties::state_info::NO_INFO;
}

template<typename StateType, typename ValueType>
ValueType const& ExplorationInformation<StateType, ValueType>::getStateReward(StateType const& stateId) const {
    return stateToReward.at(stateId);
}

template<typename StateType, typename ValueType>
bool ExplorationInformation<StateType, ValueType>::isTerminal(StateType const& state) const {
    const auto stateInfo = getStateInfo(state);
    return properties::state_info::checkIsTerminal(stateInfo);
}

template<typename StateType, typename ValueType>
typename ExplorationInformation<StateType, ValueType>::ActionType const& ExplorationInformation<StateType, ValueType>::getStartRowOfGroup(
    StateType const& group) const {
    return rowGroupIndices[group];
}

template<typename StateType, typename ValueType>
size_t ExplorationInformation<StateType, ValueType>::getRowGroupSize(StateType const& group) const {
    return rowGroupIndices[group + 1] - rowGroupIndices[group];
}

template<typename StateType, typename ValueType>
bool ExplorationInformation<StateType, ValueType>::onlyOneActionAvailable(StateType const& group) const {
    return getRowGroupSize(group) == 1;
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addStateInfo(StateType const& state, StateInfoType const info) {
    if (properties::state_info::checkNoInfo(info)) {
        statesInfo.erase(state);
    }
    statesInfo[state] = info;
    auto stateInfoIt = statesInfo.find(state);
    if (stateInfoIt == statesInfo.end()) {
        if (!properties::state_info::checkNoInfo(info)) {
            statesInfo.insert({state, info});
        }
    } else {
        stateInfoIt->second = stateInfoIt->second | info;
    }
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addStateReward(StateType const& stateId, ValueType const& stateReward) {
    if (stateToReward.size() <= stateId) {
        stateToReward.resize(stateId + 1U, std::numeric_limits<ValueType>::infinity());
    }
    stateToReward.at(stateId) = stateReward;
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addTerminalState(StateType const& state) {
    addStateInfo(state, properties::state_info::IS_TERMINAL);
}

template<typename StateType, typename ValueType>
std::vector<storm::storage::MatrixEntry<StateType, ValueType>>& ExplorationInformation<StateType, ValueType>::getRowOfMatrix(ActionType const& row) {
    return actionToTargetStates[row];
}

template<typename StateType, typename ValueType>
std::vector<storm::storage::MatrixEntry<StateType, ValueType>> const& ExplorationInformation<StateType, ValueType>::getRowOfMatrix(
    ActionType const& row) const {
    return actionToTargetStates[row];
}

template<typename StateType, typename ValueType>
ValueType const& ExplorationInformation<StateType, ValueType>::getActionReward(ActionType const& actionId) const {
    return actionToReward.at(actionId);
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addActionsToMatrix(size_t const& count) {
    actionToTargetStates.resize(actionToTargetStates.size() + count);
}

template<typename StateType, typename ValueType>
void ExplorationInformation<StateType, ValueType>::addActionReward(ActionType const& actionId, ValueType const& actionReward) {
    if (actionToReward.size() <= actionId) {
        actionToReward.resize(actionToTargetStates.size(), std::numeric_limits<ValueType>::infinity());
    }
    actionToReward.at(actionId) = actionReward;
}

template class ExplorationInformation<uint32_t, double>;
}  // namespace smc_storm::samples
