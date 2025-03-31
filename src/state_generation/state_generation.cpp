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

#include <storm/storage/expressions/ExpressionManager.h>

#include <storm/generator/JaniNextStateGenerator.h>
#include <storm/generator/PrismNextStateGenerator.h>

#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidOperationException.h>
#include <storm/utility/constants.h>

#include "state_generation/state_generation.hpp"

namespace smc_storm::state_generation {

template <typename StateType, typename ValueType>
StateGeneration<StateType, ValueType>::StateGeneration(
    const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
    const bool store_compressed_states, std::default_random_engine& random_generator)
    : StateGenerationBase<ValueType>(formula, store_compressed_states, random_generator) {
    initNextStateGenerator(model, reward_model);
    initStateToIdCallback();
    computeInitialStates();
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::resetModel() {
    loadNewState(_initial_states.at(0));
}

template <typename StateType, typename ValueType>
const AvailableActions<ValueType>& StateGeneration<StateType, ValueType>::getAvailableActions() {
    const auto& actions = _loaded_state_description.value().get().getActions();
    _available_actions.resize(actions.size());
    for (size_t idx = 0U; idx < actions.size(); idx++) {
        _available_actions[idx] = std::make_pair(idx, actions[idx].second);
    }
    return _available_actions;
}

template <typename StateType, typename ValueType>
ValueType StateGeneration<StateType, ValueType>::runAction(const uint64_t action_id) {
    const auto& available_destinations = _loaded_state_description.value().get().getActions()[action_id].first;
    size_t selected_dest_id = 0U;
    if (available_destinations.size() > 1U) {
        std::vector<ValueType> dest_probabilities(available_destinations.size());
        std::transform(available_destinations.begin(), available_destinations.end(), dest_probabilities.begin(), [](const auto& entry) {
            return entry.first;
        });
        selected_dest_id =
            std::discrete_distribution<size_t>(dest_probabilities.begin(), dest_probabilities.end())(this->_random_generator.get());
    }
    loadNewState(available_destinations[selected_dest_id].second);
    // The destinations rewards are already factored in the action reward in Storm's NextStateGenerators
    return storm::utility::zero<ValueType>();
}

template <typename StateType, typename ValueType>
ValueType StateGeneration<StateType, ValueType>::getStateReward() {
    return _loaded_state_description.value().get().getReward();
}

template <typename StateType, typename ValueType>
state_properties::StateInfoType StateGeneration<StateType, ValueType>::getStateInfo() {
    return _loaded_state_description.value().get().getStateInfo();
}

template <typename StateType, typename ValueType>
const storm::generator::CompressedState& StateGeneration<StateType, ValueType>::getCurrentState() const {
    return _loaded_state_description.value().get().getCompressedState();
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::loadNewState(const StateType state_id) {
    _loaded_state = state_id;
    if (_state_expansion_handler.getExplorationInformation().isUnexplored(_loaded_state)) {
        auto unexplored_it = _state_expansion_handler.getExplorationInformation().findUnexploredState(_loaded_state);
        auto state_desc_ptr = exploreState(_loaded_state, unexplored_it->second);
        _loaded_state_description = *state_desc_ptr;
        _state_expansion_handler.getExplorationInformation().removeUnexploredState(unexplored_it);
        _state_expansion_handler.getExplorationInformation().addStateDescription(_loaded_state, std::move(state_desc_ptr));
    } else {
        _loaded_state_description = _state_expansion_handler.getExplorationInformation().getStateDescription(_loaded_state);
    }
}

template <typename StateType, typename ValueType>
std::unique_ptr<typename StateGeneration<StateType, ValueType>::StateDescription> StateGeneration<StateType, ValueType>::exploreState(
    const StateType state_id, const storm::generator::CompressedState& compressed_state) {
    // At start, we know nothing about this state
    state_properties::StateInfoType state_info = state_properties::state_info::NO_INFO;

    auto state_desc_ptr = std::make_unique<StateDescription>();

    if (this->_store_compressed_states) {
        state_desc_ptr->setCompressedState(compressed_state);
    }
    _generator_ptr->load(compressed_state);
    const storm::generator::StateBehavior<ValueType, StateType> expanded_state = _generator_ptr->expand(_state_to_id_callback);

    if (rewardLoaded()) {
        state_desc_ptr->setReward(expanded_state.getStateRewards().at(_reward_model_index));
    }

    if (_generator_ptr->satisfies(this->_property_description.getTargetExpression())) {
        state_info |= state_properties::state_info::SATISFY_TARGET;
    }
    if (_generator_ptr->satisfies(this->_property_description.getConditionExpression())) {
        // Check if there is any transition to a different state
        bool other_successor = false;
        for (const auto& choice : expanded_state) {
            for (const auto& [next_state_id, _] : choice) {
                if (next_state_id != _loaded_state) {
                    // We found a successor that goes to a new state, no termination yet!
                    other_successor = true;
                    break;
                }
            }
        }
        if (!other_successor) {
            // Can't find a successor: we reached a terminal state
            state_info |= state_properties::state_info::IS_TERMINAL;
        } else {
            // If the state isn't a dead-end, explore the next states
            std::vector<std::pair<ValueType, StateType>> action_transitions;
            for (const auto& choice : expanded_state) {
                const ValueType action_reward = rewardLoaded() ? choice.getRewards().at(_reward_model_index) : 0.0;
                action_transitions.clear();
                action_transitions.reserve(choice.size());
                for (const auto& [next_state_id, likelihood] : choice) {
                    action_transitions.emplace_back(likelihood, next_state_id);
                }
                state_desc_ptr->addAction(action_transitions, action_reward);
            }
        }

    } else {
        state_info |= state_properties::state_info::BREAK_CONDITION;
    }
    state_desc_ptr->setStateInfo(state_info);
    return state_desc_ptr;
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::computeInitialStates() {
    _initial_states = _generator_ptr->getInitialStates(_state_to_id_callback);
    STORM_LOG_THROW(
        _initial_states.size() == 1, storm::exceptions::InvalidModelException,
        "Currently only models with one initial state are supported by the exploration engine.");
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::initStateToIdCallback() {
    _state_expansion_handler.init(_generator_ptr->getStateSize());
    _state_to_id_callback =
        std::bind(&StateExpansionHandler<StateType, ValueType>::stateExpansionCallback, &_state_expansion_handler, std::placeholders::_1);
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::initNextStateGenerator(
    const storm::storage::SymbolicModelDescription& model, const std::string& reward_model) {
    std::map<std::string, storm::expressions::Expression> label_to_expression_mapping = {};
    const bool build_rewards = !reward_model.empty();
    const storm::generator::NextStateGeneratorOptions gen_options(build_rewards, true);
    // Prepare the next state generator
    _generator_ptr.reset();
    if (model.isJaniModel()) {
        const storm::jani::Model& jani_model = model.asJaniModel();
        _generator_ptr = std::make_unique<storm::generator::JaniNextStateGenerator<ValueType, StateType>>(jani_model, gen_options);
        for (const storm::jani::Variable& variable : jani_model.getGlobalVariables().getBooleanVariables()) {
            if (variable.isTransient()) {
                label_to_expression_mapping[variable.getName()] = jani_model.getLabelExpression(variable);
            }
        }
    } else if (model.isPrismProgram()) {
        const storm::prism::Program& prism_program = model.asPrismProgram();
        _generator_ptr = std::make_unique<storm::generator::PrismNextStateGenerator<ValueType, StateType>>(prism_program, gen_options);
        label_to_expression_mapping = prism_program.getLabelToExpressionMapping();
    }
    STORM_LOG_THROW(
        _generator_ptr, storm::exceptions::InvalidOperationException, "Cannot make a next state generator with provided model.");
    if (build_rewards) {
        const size_t n_reward_models = _generator_ptr->getNumberOfRewardModels();
        for (size_t rew_idx = 0U; rew_idx < n_reward_models; ++rew_idx) {
            const auto& reward_info = _generator_ptr->getRewardModelInformation(rew_idx);
            if (reward_model == reward_info.getName()) {
                _reward_model_index = rew_idx;
            }
        }
        STORM_LOG_THROW(rewardLoaded(), storm::exceptions::UnexpectedException, "Cannot find required model " << reward_model);
    }
    // Prepare the expressions we need to verify
    this->_property_description.generateExpressions(model.getManager(), label_to_expression_mapping);
}

template class StateGeneration<uint32_t, double>;
}  // namespace smc_storm::state_generation
