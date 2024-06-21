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

#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include <storm/solver/OptimizationDirection.h>

#include <storm/generator/CompressedState.h>

#include <storm/storage/BoostTypes.h>
#include <storm/storage/SparseMatrix.h>

#include "state_properties/state_info.hpp"

namespace smc_storm::samples {
/*!
 * @brief Class holding information about the explored states
 * @tparam StateType variable type for state and action identifiers
 * @tparam ValueType variable type for the results (e.g. Rewards)
 */
template <typename StateType, typename ValueType>
class ExplorationInformation {
  public:
    typedef state_properties::StateInfoType StateInfoType;
    typedef StateType ActionType;
    typedef std::unordered_map<StateType, StateInfoType> StateInfoMap;
    typedef std::unordered_map<StateType, storm::generator::CompressedState> IdToStateMap;
    typedef typename IdToStateMap::const_iterator const_iterator;
    typedef std::vector<std::vector<storm::storage::MatrixEntry<StateType, ValueType>>> MatrixType;

    /*!
     * @brief Constructor for the ExplorationInformation class
     * @param store_expanded_states If true, the class will store all expanded raw states in memory
     */
    ExplorationInformation(const bool store_expanded_states);

    /*!
     * @brief Return the compressed state associated to the input state id
     * @param state the ID of the requested compressed state
     * @return A compressed state
     */
    const storm::generator::CompressedState& getCompressedState(const StateType& state) const;

    const_iterator findUnexploredState(const StateType& state) const;

    const_iterator unexploredStatesEnd() const;

    void removeUnexploredState(const_iterator it);

    void addUnexploredState(const StateType& state_id, const storm::generator::CompressedState& compressed_state);

    void assignStateToRowGroup(const StateType& state, const ActionType& row_group);

    StateType assignStateToNextRowGroup(const StateType& state);

    StateType getNextRowGroup() const;

    void newRowGroup(const ActionType& action);

    void newRowGroup();

    void terminateCurrentRowGroup();

    void moveActionToBackOfMatrix(const ActionType& action);

    StateType getActionCount() const;

    size_t getNumberOfUnexploredStates() const;

    size_t getNumberOfDiscoveredStates() const;

    const StateType& getRowGroup(const StateType& state) const;

    const StateType& getUnexploredMarker() const;

    bool isUnexplored(const StateType& state) const;

    /*!
     * @brief Check if the input state does not need to be expanded further
     * States that are not to expand are: VERIFY_PROPERTY, BREAK_CONDITION, IS_TERMINAL
     * @param state The ID of the state we need to check
     * @return true if the state is final, false otherwise
     */
    bool isTerminal(const StateType& state) const;

    /*!
     * @brief Return the StateInfo of the input state (if available)
     * @param state The ID of the state we are evaluating
     * @return Optionally, the StateInfo assigned to a specific state
     */
    StateInfoType getStateInfo(const StateType& state) const;

    const ValueType& getStateReward(const StateType& stateId) const;

    const ActionType& getStartRowOfGroup(const StateType& group) const;

    size_t getRowGroupSize(const StateType& group) const;

    bool onlyOneActionAvailable(const StateType& group) const;

    /*!
     * @brief Assign a specific StateInfo to the provided state.
     * @param state The ID of the state to mark
     * @param info The new label to assign to a specific state: it overwrites.
     */
    void addStateInfo(const StateType& state, const StateInfoType info);

    void addStateReward(const StateType& state_id, const ValueType& state_reward);

    /*!
     * @brief Used for compatibility: assigns the IS_TERMINAL label to the provided state
     * @param state the ID of the state to mark
     */
    void addTerminalState(const StateType& state);

    std::vector<storm::storage::MatrixEntry<StateType, ValueType>>& getRowOfMatrix(const ActionType& row);

    const std::vector<storm::storage::MatrixEntry<StateType, ValueType>>& getRowOfMatrix(const ActionType& row) const;

    const ValueType& getActionReward(const ActionType& action_id) const;

    void addActionsToMatrix(const size_t& count);

    void addActionReward(const ActionType& action_id, const ValueType& action_reward);

  private:
    // The value used to mark unexplored states
    static constexpr ActionType UNEXPLORED_MARKER{std::numeric_limits<ActionType>::max()};

    // Flag to signal whether to store the compressed states or not
    const bool _store_expanded_states;

    // Assigns to each actionId (row) a vector of (targetStateId, Likelihood) (of type MatrixEntry)
    MatrixType _action_to_target_states;

    // Assigns a reward to each action ID
    std::vector<ValueType> _action_to_reward;

    // This maps a a rowGroupId (related to a stateId via _state_to_row_group_mapping) to the 1st actionId leaving from that state.
    // The last actionId associated to that rowGroupId will be (row_group_indices[rowGroupId+1] - 1)
    std::vector<StateType> _row_group_indices;

    // Map each StateId to a rowGroupId (or unexploredMarker, if no rowGroupId yet)
    std::vector<StateType> _state_to_row_group_mapping;

    // Assigns for each explored state a reward. Unexplored states will have infinite reward
    std::vector<ValueType> _state_to_reward;

    // Keep the compressed state in memory in case we need to output the generated traces
    std::vector<storm::generator::CompressedState> _state_to_compressed_state;

    IdToStateMap _unexplored_states;

    StateInfoMap _states_info;
};
}  // namespace smc_storm::samples
