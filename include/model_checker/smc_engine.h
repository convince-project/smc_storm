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

#include <storm/modelchecker/AbstractModelChecker.h>

// This is the generic description able to contain both Jani and Prism models
#include <storm/storage/SymbolicModelDescription.h>

#include <storm/generator/CompressedState.h>
#include <storm/generator/VariableInformation.h>

#include <storm/utility/ConstantsComparator.h>

#include "settings/smc_settings.hpp"
#include "state_properties/state_info.h"
#include "samples/sampling_results.h"
#include "samples/model_sampling.h"
#include "samples/trace_information.hpp"
#include "samples/traces_exporter.h"

// Fwd declaration for storm::Environment
namespace storm {
class Environment;
}  // namespace storm

namespace smc_storm {
// Additional fwd declarations
namespace state_properties {
class PropertyDescription;
} // namespace state_properties

namespace samples {
template<typename StateType, typename ValueType>
class ExplorationInformation;
}  // namespace samples

namespace model_checker {

template<typename StateType, typename ValueType>
class StateGeneration;

template<typename ModelType, typename StateType = uint32_t>
class StatisticalExplorationModelChecker : public storm::modelchecker::AbstractModelChecker<ModelType> {
   public:
    typedef typename ModelType::ValueType ValueType;
    typedef StateType ActionType;

    StatisticalExplorationModelChecker(storm::storage::SymbolicModelDescription const& in_model, const settings::SmcSettings& settings);

    virtual bool canHandle(storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task) const override;

    static bool canHandleStatic(storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task);

    virtual std::unique_ptr<storm::modelchecker::CheckResult> computeProbabilities(storm::Environment const& env, storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task) override;

    virtual std::unique_ptr<storm::modelchecker::CheckResult> computeReachabilityRewards(
        storm::Environment const& env, storm::logic::RewardMeasureType reward_type, storm::modelchecker::CheckTask<storm::logic::EventuallyFormula, ValueType> const& check_task) override;

   private:
    bool verifyModelValid() const;

    bool verifySettingsValid() const;

    bool traceStorageEnabled() const;

    std::vector<uint_fast32_t> getThreadSeeds() const;

    /*!
     * @brief Generates samples from a model and counts how many times a given property is satisfied
     * @param check_task The property we are evaluating on the loaded model
     * @param model_sampler Object used to randomly sample next action and states
     * @param sampling_results Object keeping track of the previous sample results and defining if new samples are needed
     */
    void performSampling(
        storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task,
        samples::ModelSampling<StateType, ValueType> const& model_sampler, samples::SamplingResults& sampling_results);

    /*!
     * @brief Sample a single path until we reach a state it doesn't make sense to expand further
     * @param state_generation A representation of the model, to sample the states from
     * @param exploration_information Information about the previous explorations (to optimize computation)
     * @param model_sampler Object used to randomly sample next action and states
     * @return A pair with the Information attached to the reached state and the accumulated reward
     */
    samples::TraceInformation samplePathFromInitialState(
        StateGeneration<StateType, ValueType>& state_generation, samples::ExplorationInformation<StateType, ValueType>& exploration_information,
        samples::ModelSampling<StateType, ValueType> const& model_sampler) const;

    /*!
     * @brief Explore a single, unexplored state
     * @param state_generation A representation of the model, to sample the states from
     * @param current_state_id The ID of the state we are exploring
     * @param currentState A compressed representation of the actual state
     * @param exploration_information Information about the previous explorations (to optimize computation)
     * @return A description of the input state for evaluation
     */
    state_properties::StateInfoType exploreState(
        StateGeneration<StateType, ValueType>& state_generation, StateType const& current_state_id,
        samples::ExplorationInformation<StateType, ValueType>& exploration_information) const;

    // The model to check.
    const storm::storage::SymbolicModelDescription _model;
    const settings::SmcSettings _settings;
    std::unique_ptr<samples::TracesExporter> _traces_exporter_ptr;
};
}  // namespace model_checker
}  // namespace smc_storm

