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

#include "model_checker/smc_plugin_instance.hpp"
#include "samples/sampling_results.hpp"
#include "samples/trace_information.hpp"
#include "samples/traces_exporter.hpp"
#include "settings/smc_settings.hpp"
#include "state_generation/action_scheduler.hpp"
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

namespace state_generation {
template <typename StateType, typename ValueType>
class StateGeneration;

template <typename ValueType>
class StateGenerationCacheless;
}  // namespace state_generation

namespace model_checker {
/*!
 * @brief The implementation of the ModelChecking engine
 * @tparam ModelType Definition of the kind of model to evaluate (e.g. DTMC, MDP, ...)
 * @tparam Whether we are caching the previously expanded data, to re-use them and speed-up evaluation
 */
template <typename ModelType, bool CacheData>
class StatisticalModelCheckingEngine : public storm::modelchecker::AbstractModelChecker<ModelType> {
  public:
    using ValueType = typename ModelType::ValueType;
    using StateType = uint32_t;
    using StateGeneratorType = std::conditional_t<
        CacheData, state_generation::StateGeneration<StateType, ValueType>, state_generation::StateGenerationCacheless<ValueType>>;
    using TraceExportType = std::conditional_t<CacheData, samples::CompressedStateTraceExporter, samples::UncompressedStateTraceExporter>;

    /*!
     * @brief Constructor for the StatisticalModelCheckingEngine
     * @param in_model the model to perform the evaluation on
     * @param settings A collection of settings, used to configure the engine
     * @param loaded_plugins All the plugins loaded from the model definition.
     */
    StatisticalModelCheckingEngine(
        const storm::storage::SymbolicModelDescription& in_model, const settings::SmcSettings& settings,
        const std::vector<SmcPluginInstance>& loaded_plugins = {});

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
        [[maybe_unused]] const storm::Environment& env,
        const storm::modelchecker::CheckTask<storm::logic::EventuallyFormula, ValueType>& check_task) override;

  private:
    /*!
     * @brief Instantiate a StateGenerator object depending on the loaded settings
     * @param random_generator A random number generator used for sampling destinations
     * @return A generic state generator instance
     */
    std::unique_ptr<StateGeneratorType> generateStateGenerator(
        const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task,
        std::default_random_engine& random_generator) const;

    void instantiateTraceGenerator(const StateGeneratorType& state_generator);

    bool verifyModelValid() const;

    bool verifySettingsValid() const;

    bool traceStorageEnabled() const;

    std::vector<uint_fast32_t> getThreadSeeds() const;

    /*!
     * @brief Generates samples from a model and counts how many times a given property is satisfied
     * @param check_task The property we are evaluating on the loaded model
     * @param rnd_gen A random number generator unique to sampling instance
     * @param sampling_results Object keeping track of the previous sample results and defining if new samples are needed
     * @param thread_id The ID of the thread performing the sampling, contained in the set [0, n_threads)
     */
    void performSampling(
        const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task, std::default_random_engine& rnd_gen,
        samples::SamplingResults& sampling_results, const size_t thread_id);

    /*!
     * @brief Sample a single path until we reach a state it doesn't make sense to expand further
     * @param state_generation A representation of the model, to sample the states from
     * @param action_sampler Object used to randomly sample next action to take
     * @return A pair with the Information attached to the reached state and the accumulated reward
     */
    samples::TraceInformation samplePathFromInitialState(
        StateGeneratorType& state_generation, const state_generation::ActionScheduler<ValueType>& action_sampler) const;

    // The model to check.
    std::reference_wrapper<const storm::storage::SymbolicModelDescription> _model;
    std::reference_wrapper<const settings::SmcSettings> _settings;
    std::reference_wrapper<const std::vector<SmcPluginInstance>> _loaded_plugins;
    std::unique_ptr<TraceExportType> _traces_exporter_ptr;
};
}  // namespace model_checker
}  // namespace smc_storm
