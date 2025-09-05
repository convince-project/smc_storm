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

#include <storm/storage/expressions/ExpressionManager.h>

#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidOperationException.h>

#include "state_generation/state_generation_cacheless.hpp"

namespace smc_storm::state_generation {

template <typename ValueType>
StateGenerationCacheless<ValueType>::StateGenerationCacheless(
    const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
    std::default_random_engine& random_generator, const std::vector<model_checker::SmcPluginInstance>& loaded_plugins)
    : StateGenerationBase<ValueType>(formula, true, random_generator) {
    initNextStateGenerator(model, reward_model, loaded_plugins);
}

template <typename ValueType>
void StateGenerationCacheless<ValueType>::initNextStateGenerator(
    const storm::storage::SymbolicModelDescription& model, const std::string& reward_model,
    const std::vector<model_checker::SmcPluginInstance>& loaded_plugins) {
    std::map<std::string, storm::expressions::Expression> label_to_expression_mapping = {};
    if (!reward_model.empty()) {
        _reward_model_name = reward_model;
    }
    // Prepare the next state generator
    STORM_LOG_THROW(model.isJaniModel(), storm::exceptions::InvalidModelException, "The cacheless mode is supported only for Jani models.");
    const storm::jani::Model& jani_model = model.asJaniModel();
    _generator_ptr = std::make_unique<typename state_generation::JaniSmcStatesExpansion<ValueType>>(
        jani_model, _reward_model_name, loaded_plugins, this->_random_generator.get());
    // This is needed because in some properties we have references to global transient variables. However, this should be discouraged!
    for (const storm::jani::Variable& variable : jani_model.getGlobalVariables().getBooleanVariables()) {
        if (variable.isTransient()) {
            label_to_expression_mapping[variable.getName()] = jani_model.getLabelExpression(variable);
        }
    }
    // Prepare the expressions composing the property under verification
    this->_property_description.generateExpressions(model.getManager(), label_to_expression_mapping);
}

template <typename ValueType>
void StateGenerationCacheless<ValueType>::resetModel() {
    _deadlock_found = false;
    _current_state = _generator_ptr->setInitialState();
    if (!_current_state->get().empty()) {
        _available_actions = _generator_ptr->getAvailableActions();
    }
}

template <typename ValueType>
const AvailableActions<ValueType>& StateGenerationCacheless<ValueType>::getAvailableActions() {
    return _available_actions;
}

template <typename ValueType>
ValueType StateGenerationCacheless<ValueType>::runAction(const uint64_t action_id) {
    const auto& possible_destinations = _generator_ptr->getDestinationsFromAction(action_id);
    size_t dest_vect_idx = 0U;
    if (possible_destinations.size() > 1U) {
        std::vector<ValueType> prob_vect(possible_destinations.size());
        std::transform(
            possible_destinations.begin(), possible_destinations.end(), prob_vect.begin(), [](const auto& entry) { return entry.second; });
        dest_vect_idx = std::discrete_distribution<size_t>(prob_vect.begin(), prob_vect.end())(this->_random_generator.get());
    }
    const uint64_t destination_id = possible_destinations[dest_vect_idx].first;
    const state_properties::StateVariableData<ValueType> previous_state(*_current_state);
    ValueType destination_reward;
    std::tie(_current_state, destination_reward) = _generator_ptr->setNextState(action_id, destination_id);
    if (_available_actions.size() == 1U && possible_destinations.size() == 1U) {
        // Evaluate whether we found a deadlock: right now this is done only in case of 1 action and 1 destination
        _deadlock_found = (previous_state == _current_state->get());
    }
    _available_actions = _generator_ptr->getAvailableActions();
    return destination_reward;
}

template <typename ValueType>
ValueType StateGenerationCacheless<ValueType>::getStateReward() {
    return _generator_ptr->getStateReward();
}

template <typename ValueType>
state_properties::StateInfoType StateGenerationCacheless<ValueType>::getStateInfo() {
    state_properties::StateInfoType state_info = state_properties::state_info::NO_INFO;
    if (_generator_ptr->satisfies(this->_property_description.getTargetExpression())) {
        state_info |= state_properties::state_info::SATISFY_TARGET;
    }
    if (!_generator_ptr->satisfies(this->_property_description.getConditionExpression())) {
        state_info |= state_properties::state_info::BREAK_CONDITION;
    }
    if (_available_actions.empty() || _deadlock_found) {
        state_info |= state_properties::state_info::IS_TERMINAL;
    }
    return state_info;
}

template class StateGenerationCacheless<double>;
}  // namespace smc_storm::state_generation
