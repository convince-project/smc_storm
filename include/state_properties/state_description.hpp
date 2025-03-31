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

#include "state_properties/state_info.hpp"
#include <optional>
#include <storm/generator/CompressedState.h>
#include <vector>

namespace smc_storm::state_properties {
/*!
 * @brief Class representing a single state of the model
 * @tparam StateType Can be either the State ID or a CompressedState description
 * @tparam ValueType The type to use for the rewards and the probability (normally double)
 */
template <typename StateType, typename ValueType>
class StateDescription {
  protected:
    // Each entry is the transition probability and the target state
    using ActionTransitions = std::vector<std::pair<ValueType, StateType>>;
    std::optional<storm::generator::CompressedState> _compressed_state = std::nullopt;
    state_properties::StateInfoType _state_info = state_properties::state_info::NO_INFO;
    ValueType _reward = 0.0;
    // Each entry is a pair of the action transition and the related reward
    std::vector<std::pair<ActionTransitions, ValueType>> _actions;

  public:
    /*!
     * @brief Constructor for the StateDescription class
     * @param state_info The information about the state
     * @param reward The reward associated to the state
     */
    StateDescription() {
        // Nothing to do
    }

    void setStateInfo(const state_properties::StateInfoType state_info) {
        _state_info = state_info;
    }

    void setReward(const ValueType reward) {
        _reward = reward;
    }

    void setCompressedState(const storm::generator::CompressedState& compressed_state) {
        _compressed_state = compressed_state;
    }

    /*!
     * @brief Add a new action to the state
     * @param action_transitions The transitions associated to the action
     * @param action_reward The reward associated to the action
     */
    void addAction(std::vector<std::pair<ValueType, StateType>>& action_transitions, const ValueType action_reward) {
        _actions.emplace_back(action_transitions, action_reward);
    }

    /*!
     * @brief Get the information associated to the state
     * @return The state information
     */
    const state_properties::StateInfoType getStateInfo() const {
        return _state_info;
    }

    /*!
     * @brief Get the reward associated to the state
     * @return The reward
     */
    const ValueType getReward() const {
        return _reward;
    }

    /*!
     * @brief Get the transitions associated to the action
     * @param action_id The ID of the action
     * @return The transitions
     */
    const std::vector<std::pair<ActionTransitions, ValueType>>& getActions() const {
        return _actions;
    }

    const storm::generator::CompressedState& getCompressedState() const {
        return *_compressed_state;
    }
};
}  // namespace smc_storm::state_properties
