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
#include <random>

#include <storm/generator/CompressedState.h>
#include <storm/generator/NextStateGenerator.h>
#include <storm/storage/SymbolicModelDescription.h>

#include "state_generation/available_actions.hpp"
#include "state_generation/state_expansion_handler.hpp"
#include "state_properties/property_description.hpp"
#include "state_properties/state_info.hpp"

namespace smc_storm::state_generation {

/*!
 * @brief Class that loads an input model and property to generate states and verify if the input property holds.
 * @tparam ValueType Variable type of computed results (e.g. rewards)
 */
template <typename ValueType>
class StateGenerationBase {
  protected:
    state_properties::PropertyDescription _property_description;
    mutable std::reference_wrapper<std::default_random_engine> _random_generator;
    const bool _store_compressed_states;

    inline StateGenerationBase(
        const storm::logic::Formula& formula, const bool store_compressed_states, std::default_random_engine& random_generator)
        : _property_description(formula), _random_generator{random_generator}, _store_compressed_states{store_compressed_states} {}

  public:
    virtual ~StateGenerationBase() = default;

    /*!
     * @brief Get the loaded property description
     * @return A PropertyDescription object, containing the property under evaluation
     */
    inline const state_properties::PropertyDescription& getPropertyDescription() const {
        return _property_description;
    }

    /*!
     * @brief Load the initial state in the model.
     */
    virtual void resetModel() = 0;

    /*!
     * @brief Return the set of available actions from the current state
     * @return A vector of pairs with action ID and associated reward
     */
    virtual const AvailableActions<ValueType>& getAvailableActions() = 0;

    /*!
     * @brief Execute a specific action: if the action has multiple destinations, pick one based on likelihood
     * @param action_id The ID of the action to execute
     * @return The single transition reward, obtained by picking a specific destination
     */
    virtual ValueType runAction(const uint64_t action_id) = 0;

    /*!
     * @brief Get the reward associated to the current state
     * @return The reward associated to the curr. state
     */
    virtual ValueType getStateReward() = 0;

    /*!
     * @brief Retrieve the state's information, checking how it evaluates against the requested property
     * @return Whether the state satisfies or not the property, or is in a terminal state
     */
    virtual state_properties::StateInfoType getStateInfo() = 0;
};

}  // namespace smc_storm::state_generation
