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

#include "model_checker/state_generation.hpp"

namespace smc_storm::model_checker {

template <typename StateType, typename ValueType>
StateGeneration<StateType, ValueType>::StateGeneration(
    const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
    const bool store_compressed_states)
    : _property_description(formula), _store_compressed_states(store_compressed_states) {
    initNextStateGenerator(model, reward_model);
    initStateToIdCallback();
    computeInitialStates();
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::loadInitialState() {
    load(_initial_states.at(0));
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::load(const StateType& state) {
    _loaded_state = state;
}

template <typename StateType, typename ValueType>
const state_properties::StateDescription<StateType, ValueType>& StateGeneration<StateType, ValueType>::processLoadedState() {
    if constexpr (StoreExpandedStates<StateType>) {
        if (_state_expansion_handler.getExplorationInformation().isUnexplored(_loaded_state)) {
            auto unexplored_it = _state_expansion_handler.getExplorationInformation().findUnexploredState(_loaded_state);
            exploreState(_loaded_state, unexplored_it->second);
            _state_expansion_handler.getExplorationInformation().removeUnexploredState(unexplored_it);
            _state_expansion_handler.getExplorationInformation().addStateDescription(_loaded_state, std::move(_state_description_ptr));
        }
        return _state_expansion_handler.getExplorationInformation().getStateDescription(_loaded_state);
    } else {
        exploreState(_state_expansion_handler.getHashValue(_loaded_state), _loaded_state);
        return *_state_description_ptr;
    }
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::exploreState(
    const StateIdType state_id, const storm::generator::CompressedState& compressed_state) {
    // At start, we know nothing about this state
    state_properties::StateInfoType state_info = state_properties::state_info::NO_INFO;

    _state_description_ptr = std::make_unique<state_properties::StateDescription<StateType, ValueType>>();
    if (_store_compressed_states) {
        _state_description_ptr->setCompressedState(compressed_state);
    }
    if constexpr (!StoreExpandedStates<StateType>) {
        // This is the data structure holding the next states generated during expansion
        _state_expansion_handler.clearNextStates();
    }
    _generator_ptr->load(compressed_state);
    const storm::generator::StateBehavior<ValueType, StateIdType> expanded_state = _generator_ptr->expand(_state_to_id_callback);

    if (rewardLoaded()) {
        _state_description_ptr->setReward(expanded_state.getStateRewards().at(_reward_model_index));
    }

    if (_generator_ptr->satisfies(_property_description.getTargetExpression())) {
        state_info |= state_properties::state_info::SATISFY_TARGET;
    }
    if (_generator_ptr->satisfies(_property_description.getConditionExpression())) {
        // Check if there is any transition to a different state
        bool other_successor = false;
        if constexpr (StoreExpandedStates<StateType>) {
            // In case StoreExpandedStates<StateType> is true, we can simply check the ID
            for (const auto& choice : expanded_state) {
                for (const auto& [next_state_id, _] : choice) {
                    if (next_state_id != _loaded_state) {
                        // We found a successor that goes to a new state, no termination yet!
                        other_successor = true;
                        break;
                    }
                }
            }
        } else {
            other_successor = std::any_of(
                _state_expansion_handler.getNextStates().begin(), _state_expansion_handler.getNextStates().end(),
                [this, state_id](const storm::generator::CompressedState& next_state) {
                    return _state_expansion_handler.getHashValue(next_state) != state_id;
                });
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
                    if constexpr (StoreExpandedStates<StateType>) {
                        action_transitions.emplace_back(likelihood, next_state_id);
                    } else {
                        action_transitions.emplace_back(likelihood, _state_expansion_handler.getNextStates().at(next_state_id));
                    }
                }
                _state_description_ptr->addAction(action_transitions, action_reward);
            }
        }

    } else {
        state_info |= state_properties::state_info::BREAK_CONDITION;
    }
    _state_description_ptr->setStateInfo(state_info);
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::computeInitialStates() {
    if constexpr (StoreExpandedStates<StateType>) {
        _initial_states = _generator_ptr->getInitialStates(_state_to_id_callback);
    } else {
        _state_expansion_handler.clearNextStates();
        _generator_ptr->getInitialStates(_state_to_id_callback);
        _initial_states = _state_expansion_handler.getNextStates();
    }
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
        _generator_ptr = std::make_unique<storm::generator::JaniNextStateGenerator<ValueType, StateIdType>>(jani_model, gen_options);
        for (const storm::jani::Variable& variable : jani_model.getGlobalVariables().getBooleanVariables()) {
            if (variable.isTransient()) {
                label_to_expression_mapping[variable.getName()] = jani_model.getLabelExpression(variable);
            }
        }
    } else if (model.isPrismProgram()) {
        const storm::prism::Program& prism_program = model.asPrismProgram();
        _generator_ptr = std::make_unique<storm::generator::PrismNextStateGenerator<ValueType, StateIdType>>(prism_program, gen_options);
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
    _property_description.generateExpressions(model.getManager(), label_to_expression_mapping);
}

template class StateGeneration<uint32_t, double>;
template class StateGeneration<storm::generator::CompressedState, double>;
}  // namespace smc_storm::model_checker