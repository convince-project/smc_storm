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

#include <storm/exceptions/InvalidOperationException.h>

#include "model_checker/state_generation.hpp"
#include "samples/exploration_information.hpp"

namespace smc_storm::model_checker {

template <typename StateType, typename ValueType>
StateGeneration<StateType, ValueType>::StateGeneration(
    const storm::storage::SymbolicModelDescription& model, const storm::logic::Formula& formula, const std::string& reward_model,
    samples::ExplorationInformation<StateType, ValueType>& exploration_information)
    : _property_description(formula) {
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
    // Prepare the state storage object
    _state_storage_ptr = std::make_unique<storm::storage::sparse::StateStorage<StateType>>(_generator_ptr->getStateSize());
    // Prepare the expressions we need to verify
    _property_description.generateExpressions(model.getManager(), label_to_expression_mapping);
    initStateToIdCallback(exploration_information);
    if (build_rewards) {
        const size_t n_reward_models = _generator_ptr->getNumberOfRewardModels();
        for (size_t rewIdx = 0U; rewIdx < n_reward_models; ++rewIdx) {
            const auto& reward_info = _generator_ptr->getRewardModelInformation(rewIdx);
            if (reward_model == reward_info.getName()) {
                _reward_model_index = rewIdx;
            }
        }
        STORM_LOG_THROW(rewardLoaded(), storm::exceptions::UnexpectedException, "Cannot find required model " << reward_model);
    }
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::initStateToIdCallback(
    samples::ExplorationInformation<StateType, ValueType>& exploration_information) {
    _state_to_id_callback = [&exploration_information, this](const storm::generator::CompressedState& state) -> StateType {
        StateType new_index = _state_storage_ptr->getNumberOfStates();

        // Check, if the state was already registered.
        std::pair<StateType, std::size_t> actual_index_bucket_pair =
            (_state_storage_ptr->stateToId).findOrAddAndGetBucket(state, new_index);

        if (actual_index_bucket_pair.first == new_index) {
            exploration_information.addUnexploredState(new_index, state);
        }

        return actual_index_bucket_pair.first;
    };
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::load(const storm::generator::CompressedState& state) {
    _generator_ptr->load(state);
}

template <typename StateType, typename ValueType>
std::vector<StateType> StateGeneration<StateType, ValueType>::getInitialStates() {
    return _state_storage_ptr->initialStateIndices;
}

template <typename StateType, typename ValueType>
storm::generator::StateBehavior<ValueType, StateType> StateGeneration<StateType, ValueType>::expand() {
    return _generator_ptr->expand(_state_to_id_callback);
}

template <typename StateType, typename ValueType>
bool StateGeneration<StateType, ValueType>::isConditionState() const {
    return _generator_ptr->satisfies(_property_description.getConditionExpression());
}

template <typename StateType, typename ValueType>
bool StateGeneration<StateType, ValueType>::isTargetState() const {
    return _generator_ptr->satisfies(_property_description.getTargetExpression());
}

template <typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::computeInitialStates() {
    _state_storage_ptr->initialStateIndices = _generator_ptr->getInitialStates(_state_to_id_callback);
}

template <typename StateType, typename ValueType>
StateType StateGeneration<StateType, ValueType>::getFirstInitialState() const {
    return _state_storage_ptr->initialStateIndices.front();
}

template <typename StateType, typename ValueType>
std::size_t StateGeneration<StateType, ValueType>::getNumberOfInitialStates() const {
    return _state_storage_ptr->initialStateIndices.size();
}

template class StateGeneration<uint32_t, double>;
}  // namespace smc_storm::model_checker