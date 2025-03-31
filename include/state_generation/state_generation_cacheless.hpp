/*
 * Copyright (c) 2025 Robert Bosch GmbH and its subsidiaries
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

#include <storm/storage/SymbolicModelDescription.h>

#include "model_checker/smc_plugin_instance.hpp"
#include "state_generation/jani_smc_states_expansion.hpp"
#include "state_generation/state_generation_base.hpp"
#include "state_properties/state_variable_information.hpp"

namespace smc_storm {
namespace state_generation {

/*!
 * @brief Class that loads an input model and property to generate states and verify if the input property holds.
 * @tparam StateType Variable type to identify states and actions
 * @tparam ValueType Variable type of computed results (e.g. rewards)
 */
template <typename ValueType>
class StateGenerationCacheless : public StateGenerationBase<ValueType> {
  public:
    /*!
     * @brief Extension of the constructor above, in case we evaluate Rewards
     * @param model The model to generate the next states from
     * @param formula The formula to evaluate on the generated states
     * @param reward_model Name of the reward model to use for assigning costs on states and actions. Empty for P properties
     * @param random_generator A random number generator, used to pick one of the possible destinations
     */
    StateGenerationCacheless(
        const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
        std::default_random_engine& random_generator, const std::vector<model_checker::SmcPluginInstance>& loaded_plugins);

    /*!
     * @brief Get the variable information related to the loaded model
     * @return A const reference to the VariableInformation instance
     */
    inline const state_properties::StateVariableInformation<ValueType>& getVariableInformation() const {
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

    inline const state_properties::StateVariableData<ValueType>& getCurrentState() const {
        return _current_state->get();
    }

  private:
    void initNextStateGenerator(
        const storm::storage::SymbolicModelDescription& model, const std::string& reward_model,
        const std::vector<model_checker::SmcPluginInstance>& loaded_plugins);

    // Generator extracting the next states from the current one and the input model.
    std::unique_ptr<state_generation::JaniSmcStatesExpansion<ValueType>> _generator_ptr;

    // Reference to the current state loaded in the model
    std::optional<std::reference_wrapper<const state_properties::StateVariableData<ValueType>>> _current_state;
    AvailableActions<ValueType> _available_actions;
    bool _deadlock_found = false;

    // Name of the loaded reward functions
    std::optional<std::string> _reward_model_name;
};

}  // namespace state_generation
}  // namespace smc_storm
