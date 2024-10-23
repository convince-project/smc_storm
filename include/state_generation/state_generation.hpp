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

#include <functional>
#include <memory>

#include <storm/generator/CompressedState.h>
#include <storm/generator/NextStateGenerator.h>
#include <storm/storage/SymbolicModelDescription.h>

#include "state_generation/state_expansion_handler.hpp"
#include "state_properties/property_description.hpp"
#include "state_properties/state_description.hpp"

namespace smc_storm {
namespace state_generation {
template <typename StateType>
concept StoreExpandedStates = !std::is_same_v<StateType, storm::generator::CompressedState>;

/*!
 * @brief Class that loads an input model and property to generate states and verify if the input property holds.
 * @tparam StateType Variable type to identify states and actions
 * @tparam ValueType Variable type of computed results (e.g. rewards)
 */
template <typename StateType, typename ValueType>
class StateGeneration {
    using StateIdType = std::conditional_t<StoreExpandedStates<StateType>, StateType, uint32_t>;

  public:
    /*!
     * @brief Extension of the constructor above, in case we evaluate Rewards
     * @param model The model to generate the next states from
     * @param formula The formula to evaluate on the generated states
     * @param reward_model Name of the reward model to use for assigning costs on states and actions. Empty for P properties
     * @param store_compressed_states Whether to store the explored compressed states (for trace generation)
     */
    StateGeneration(
        const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
        const bool store_compressed_states);

    /*!
     * @brief Get the variable information related to the loaded model
     * @return A const reference to the VariableInformation instance
     */
    inline const storm::generator::VariableInformation& getVariableInformation() const {
        return _generator_ptr->getVariableInformation();
    }

    /*!
     * @brief Load the initial state in the NextStateGenerator (basically a model reset).
     */
    void loadInitialState();

    /*!
     * @brief Load a compressed state in the NextStateGenerator
     * @param state The compressed state to load
     */
    void load(const StateType& state_id);

    const state_properties::StateDescription<StateType, ValueType>& processLoadedState();

    const state_properties::PropertyDescription& getPropertyDescription() const {
        return _property_description;
    }

  private:
    void initStateToIdCallback();

    void initNextStateGenerator(const storm::storage::SymbolicModelDescription& model, const std::string& reward_model);

    /*!
     * @brief Expand the loaded state to get the next states
     * @return A generator used to access the generated information
     */
    storm::generator::StateBehavior<ValueType, StateType> expand();

    void exploreState(const StateIdType state_id, const storm::generator::CompressedState& compressed_state);

    void computeInitialStates();

    /*!
     * @brief Check if a reward model is loaded and therefore it needs to be evaluated
     * @return true if we loaded a reward model, false otherwise
     */
    inline bool rewardLoaded() const {
        return std::numeric_limits<size_t>::max() != _reward_model_index;
    }

    // Generator extracting the next states from the current one and the input model.
    std::unique_ptr<storm::generator::NextStateGenerator<ValueType, StateIdType>> _generator_ptr;
    // Current state.
    StateType _loaded_state;
    std::unique_ptr<state_properties::StateDescription<StateType, ValueType>> _state_description_ptr;
    // Vector of initial states (normally only one).
    std::vector<StateType> _initial_states;

    state_generation::StateExpansionHandler<StateType, ValueType> _state_expansion_handler;

    state_properties::PropertyDescription _property_description;
    size_t _reward_model_index = std::numeric_limits<size_t>::max();

    const bool _store_compressed_states;

    std::function<StateIdType(const storm::generator::CompressedState&)> _state_to_id_callback;
};

}  // namespace state_generation
}  // namespace smc_storm
