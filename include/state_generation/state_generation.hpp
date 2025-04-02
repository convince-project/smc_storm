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
#include "state_generation/state_generation_base.hpp"
#include "state_properties/property_description.hpp"
#include "state_properties/state_description.hpp"

namespace smc_storm {
namespace state_generation {

/*!
 * @brief Class that loads an input model and property to generate states and verify if the input property holds.
 * @tparam StateType Variable type to identify states and actions
 * @tparam ValueType Variable type of computed results (e.g. rewards)
 */
template <typename StateType, typename ValueType>
class StateGeneration : public StateGenerationBase<ValueType> {
  public:
    using StateDescription = state_properties::StateDescription<StateType, ValueType>;
    /*!
     * @brief Extension of the constructor above, in case we evaluate Rewards
     * @param model The model to generate the next states from
     * @param formula The formula to evaluate on the generated states
     * @param reward_model Name of the reward model to use for assigning costs on states and actions. Empty for P properties
     * @param store_compressed_states Whether to store the explored compressed states (for trace generation)
     * @param random_generator A random number generator, used to pick one of the possible destinations
     */
    StateGeneration(
        const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
        const bool store_compressed_states, std::default_random_engine& random_generator);

    /*!
     * @brief Get the variable information related to the loaded model
     * @return A const reference to the VariableInformation instance
     */
    inline const storm::generator::VariableInformation& getVariableInformation() const {
        return _generator_ptr->getVariableInformation();
    }

    /*!
     * @brief Load the initial state in the model.
     */
    void resetModel() override;

    /*!
     * @brief Return the set of available actions from the current state
     * @return A vector of pairs with action ID and associated reward
     */
    const AvailableActions<ValueType>& getAvailableActions() override;

    /// @copydoc StateGenerationBase<ValueType>::runAction(const uint64_t action_id)
    ValueType runAction(const uint64_t action_id) override;

    /*!
     * @brief Get the reward associated to the current state
     * @return The reward associated to the curr. state
     */
    ValueType getStateReward() override;

    /*!
     * @brief Retrieve the state's information, checking how it evaluates against the requested property
     * @return Whether the state satisfies or not the property, or is in a terminal state
     */
    state_properties::StateInfoType getStateInfo() override;

    const storm::generator::CompressedState& getCurrentState() const;

  private:
    void initStateToIdCallback();

    void initNextStateGenerator(const storm::storage::SymbolicModelDescription& model, const std::string& reward_model);

    void loadNewState(const StateType state_id);

    std::unique_ptr<StateDescription> exploreState(const StateType state_id, const storm::generator::CompressedState& compressed_state);

    void computeInitialStates();

    /*!
     * @brief Check if a reward model is loaded and therefore it needs to be evaluated
     * @return true if we loaded a reward model, false otherwise
     */
    inline bool rewardLoaded() const {
        return std::numeric_limits<size_t>::max() != _reward_model_index;
    }

    // Generator extracting the next states from the current one and the input model.
    std::unique_ptr<storm::generator::NextStateGenerator<ValueType, StateType>> _generator_ptr;
    // Current state.
    StateType _loaded_state;
    std::optional<std::reference_wrapper<const StateDescription>> _loaded_state_description;
    // Vector of initial states (normally only one).
    std::vector<StateType> _initial_states;
    AvailableActions<ValueType> _available_actions;

    state_generation::StateExpansionHandler<StateType, ValueType> _state_expansion_handler;

    std::function<StateType(const storm::generator::CompressedState&)> _state_to_id_callback;

    size_t _reward_model_index = std::numeric_limits<size_t>::max();
};

}  // namespace state_generation
}  // namespace smc_storm
