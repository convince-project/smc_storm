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

#include <storm/generator/PrismNextStateGenerator.h>
#include <storm/generator/JaniNextStateGenerator.h>

#include <storm/exceptions/InvalidOperationException.h>

#include "model_checker/state_generation.h"
#include "samples/exploration_information.h"

namespace smc_storm::model_checker {

template<typename StateType, typename ValueType>
StateGeneration<StateType, ValueType>::StateGeneration(
    storm::storage::SymbolicModelDescription const& model, storm::logic::Formula const& formula,
    samples::ExplorationInformation<StateType, ValueType>& explorationInformation, bool const buildRewards)
    : propertyDescription(formula)
{
    std::map<std::string, storm::expressions::Expression> labelToExpressionMapping = {};
    const storm::generator::NextStateGeneratorOptions genOptions(buildRewards, true);
    // Prepare the next state generator
    generatorPtr.reset();
    if (model.isJaniModel()) {
        const storm::jani::Model& janiModel = model.asJaniModel();
        generatorPtr = std::make_unique<storm::generator::JaniNextStateGenerator<ValueType, StateType>>(janiModel, genOptions);
        for (const storm::jani::Variable& variable : janiModel.getGlobalVariables().getBooleanVariables()) {
            if (variable.isTransient()) {
                labelToExpressionMapping[variable.getName()] = janiModel.getLabelExpression(variable);
            }
        }
    } else if (model.isPrismProgram()) {
        const storm::prism::Program& prismProgram = model.asPrismProgram();
        generatorPtr = std::make_unique<storm::generator::PrismNextStateGenerator<ValueType, StateType>>(prismProgram, genOptions);
        labelToExpressionMapping = prismProgram.getLabelToExpressionMapping();
    }
    STORM_LOG_THROW(generatorPtr, storm::exceptions::InvalidOperationException, "Cannot make a next state generator with provided model.");
    // Prepare the state storage object
    stateStoragePtr = std::make_unique<storm::storage::sparse::StateStorage<StateType>>(generatorPtr->getStateSize());
    // Prepare the expressions we need to verify
    propertyDescription.generateExpressions(model.getManager(), labelToExpressionMapping);
    initStateToIdCallback(explorationInformation);
}

template<typename StateType, typename ValueType>
StateGeneration<StateType, ValueType>::StateGeneration(
    storm::storage::SymbolicModelDescription const& model, storm::logic::Formula const& formula,
    std::string const& rewardModel, samples::ExplorationInformation<StateType, ValueType>& explorationInformation)
    : StateGeneration(model, formula, explorationInformation, true)
{
    const size_t nRewardModels = generatorPtr->getNumberOfRewardModels();
    for (size_t rewIdx = 0U; rewIdx < nRewardModels; ++rewIdx) {
        const auto& rewardInfo = generatorPtr->getRewardModelInformation(rewIdx);
        if (rewardModel == rewardInfo.getName()) {
            rewardModelIndex = rewIdx;
        }
    }
    STORM_LOG_THROW(rewardLoaded(), storm::exceptions::UnexpectedException, "Cannot find required model " << rewardModel);
}

template<typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::initStateToIdCallback(samples::ExplorationInformation<StateType, ValueType>& explorationInformation) {
    stateToIdCallback = [&explorationInformation, this](storm::generator::CompressedState const& state) -> StateType {
        StateType newIndex = stateStoragePtr->getNumberOfStates();

        // Check, if the state was already registered.
        std::pair<StateType, std::size_t> actualIndexBucketPair = (stateStoragePtr->stateToId).findOrAddAndGetBucket(state, newIndex);

        if (actualIndexBucketPair.first == newIndex) {
            explorationInformation.addUnexploredState(newIndex, state);
        }

        return actualIndexBucketPair.first;
    };
}

template<typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::load(storm::generator::CompressedState const& state) {
    generatorPtr->load(state);
}

template<typename StateType, typename ValueType>
std::vector<StateType> StateGeneration<StateType, ValueType>::getInitialStates() {
    return stateStoragePtr->initialStateIndices;
}

template<typename StateType, typename ValueType>
storm::generator::StateBehavior<ValueType, StateType> StateGeneration<StateType, ValueType>::expand() {
    return generatorPtr->expand(stateToIdCallback);
}

template<typename StateType, typename ValueType>
bool StateGeneration<StateType, ValueType>::isConditionState() const {
    return generatorPtr->satisfies(propertyDescription.getConditionExpression());
}

template<typename StateType, typename ValueType>
bool StateGeneration<StateType, ValueType>::isTargetState() const {
    return generatorPtr->satisfies(propertyDescription.getTargetExpression());
}

template<typename StateType, typename ValueType>
void StateGeneration<StateType, ValueType>::computeInitialStates() {
    stateStoragePtr->initialStateIndices = generatorPtr->getInitialStates(stateToIdCallback);
}

template<typename StateType, typename ValueType>
StateType StateGeneration<StateType, ValueType>::getFirstInitialState() const {
    return stateStoragePtr->initialStateIndices.front();
}

template<typename StateType, typename ValueType>
std::size_t StateGeneration<StateType, ValueType>::getNumberOfInitialStates() const {
    return stateStoragePtr->initialStateIndices.size();
}

template class StateGeneration<uint32_t, double>;
}  // namespace smc_storm::model_checker