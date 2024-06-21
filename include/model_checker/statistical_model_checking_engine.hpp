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

#include "samples/model_sampling.hpp"
#include "samples/sampling_results.hpp"
#include "samples/trace_information.hpp"
#include "samples/traces_exporter.hpp"
#include "settings/smc_settings.hpp"
#include "state_properties/state_info.hpp"

// Fwd declaration for storm::Environment
namespace storm {
class Environment;
}  // namespace storm

namespace smc_storm {
// Additional fwd declarations
namespace state_properties {
class PropertyDescription;
}  // namespace state_properties

namespace samples {
template <typename StateType, typename ValueType>
class ExplorationInformation;
}  // namespace samples

namespace model_checker {

template <typename StateType, typename ValueType>
class StateGeneration;

/*!
 * @brief The implementation of the ModelChecking engine
 * @tparam ModelType Definition of the kind of model to evaluate (e.g. DTMC, MDP, ...)
 * @tparam StateType Variable type used to identify the states in the model
 */
template <typename ModelType, typename StateType = uint32_t>
class StatisticalModelCheckingEngine : public storm::modelchecker::AbstractModelChecker<ModelType> {
  public:
    typedef typename ModelType::ValueType ValueType;
    typedef StateType ActionType;

    /*!
     * @brief Constructor for the StatisticalModelCheckingEngine
     * @param in_model the model to perform the evaluation on
     * @param settings A collection of settings, used to configure the engine
     */
    StatisticalModelCheckingEngine(const storm::storage::SymbolicModelDescription& in_model, const settings::SmcSettings& settings);

    /*!
     * @brief Check if the provided property is supported by the engine
     * @param check_task The property to verify
     * @return true if the property can be handled, false otherwise
     */
    virtual bool canHandle(const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task) const override;

    /*!
     * @brief Static version of the engine's compatibility check for the provided property
     * @param check_task The property to verify
     * @return true if the property can be handled, false otherwise
     */
    static bool canHandleStatic(const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task);

    /*!
     * @brief Evaluate the loaded model on a P property
     * @param env Variable carrying information for other Model Checkers in STORM. Unused here.
     * @param check_task The property to verify
     * @return The result of the evaluation
     */
    virtual std::unique_ptr<storm::modelchecker::CheckResult> computeProbabilities(
        [[maybe_unused]] const storm::Environment& env,
        const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task) override;

    /*!
     * @brief Evaluate the loaded model on a R property
     * @param env Variable carrying information for other Model Checkers in STORM. Unused here.
     * @param check_task The property to verify
     * @return The result of the evaluation
     */
    virtual std::unique_ptr<storm::modelchecker::CheckResult> computeReachabilityRewards(
        [[maybe_unused]] const storm::Environment& env, storm::logic::RewardMeasureType reward_type,
        const storm::modelchecker::CheckTask<storm::logic::EventuallyFormula, ValueType>& check_task) override;

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
        const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task,
        const samples::ModelSampling<StateType, ValueType>& model_sampler, samples::SamplingResults& sampling_results);

    /*!
     * @brief Sample a single path until we reach a state it doesn't make sense to expand further
     * @param state_generation A representation of the model, to sample the states from
     * @param exploration_information Information about the previous explorations (to optimize computation)
     * @param model_sampler Object used to randomly sample next action and states
     * @return A pair with the Information attached to the reached state and the accumulated reward
     */
    samples::TraceInformation samplePathFromInitialState(
        StateGeneration<StateType, ValueType>& state_generation,
        samples::ExplorationInformation<StateType, ValueType>& exploration_information,
        const samples::ModelSampling<StateType, ValueType>& model_sampler) const;

    /*!
     * @brief Explore a single, unexplored state
     * @param state_generation A representation of the model, to sample the states from
     * @param current_state_id The ID of the state we are exploring
     * @param currentState A compressed representation of the actual state
     * @param exploration_information Information about the previous explorations (to optimize computation)
     * @return A description of the input state for evaluation
     */
    state_properties::StateInfoType exploreState(
        StateGeneration<StateType, ValueType>& state_generation, const StateType& current_state_id,
        samples::ExplorationInformation<StateType, ValueType>& exploration_information) const;

    // The model to check.
    const storm::storage::SymbolicModelDescription _model;
    const settings::SmcSettings _settings;
    std::unique_ptr<samples::TracesExporter> _traces_exporter_ptr;
};
}  // namespace model_checker
}  // namespace smc_storm
