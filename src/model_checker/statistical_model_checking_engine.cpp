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

#include <filesystem>
#include <thread>

#include <storm/storage/expressions/ExpressionEvaluator.h>

#include <storm/generator/CompressedState.h>

#include <storm/storage/MaximalEndComponentDecomposition.h>
#include <storm/storage/SparseMatrix.h>

#include <storm/storage/jani/Model.h>
#include <storm/storage/prism/Program.h>

#include <storm/logic/FragmentSpecification.h>

#include <storm/modelchecker/results/ExplicitQuantitativeCheckResult.h>

#include <storm/models/sparse/Dtmc.h>
#include <storm/models/sparse/Mdp.h>
#include <storm/models/sparse/StandardRewardModel.h>

#include <storm/settings/modules/CoreSettings.h>
#include <storm/settings/modules/ExplorationSettings.h>
#include <storm/settings/SettingsManager.h>

#include <storm/utility/constants.h>
#include <storm/utility/graph.h>
#include <storm/utility/macros.h>
#include <storm/utility/prism.h>
#include <storm/utility/random.h>

#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidOperationException.h>
#include <storm/exceptions/InvalidPropertyException.h>
#include <storm/exceptions/InvalidSettingsException.h>

#include "state_generation/state_generation.hpp"
#include "state_properties/property_type.hpp"

#include "model_checker/statistical_model_checking_engine.hpp"

namespace smc_storm {
namespace model_checker {
template <typename ModelType, bool StoreExploredStates>
bool StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::canHandleStatic(
    const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task) {
    // Prepare the fragment defined our supported formulas
    // This allows all (atomic) property operators + the unary NOT logic operator on atomic properties (e.g. !F(s=2))
    storm::logic::FragmentSpecification const fragment =
        storm::logic::prctl().setNestedOperatorsAllowed(false).setUnaryBooleanPathFormulasAllowed(true).setOperatorAtTopLevelRequired(
            false);
    const storm::logic::Formula& formula = check_task.getFormula();
    const bool is_formula_supported = formula.isInFragment(fragment);
    const bool is_only_init_state_relevant = check_task.isOnlyInitialStatesRelevantSet();
    // Conditional logging prints when the condition does not hold!
    STORM_LOG_WARN_COND(is_formula_supported, "The input property " << formula.toString() << " is not supported by the SMC engine.");
    STORM_LOG_WARN_COND(is_only_init_state_relevant, "Property doesn't require only the initial state to be verified.");
    return is_formula_supported && is_only_init_state_relevant;
}

template <typename ModelType, bool StoreExploredStates>
bool StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::traceStorageEnabled() const {
    return !_settings.traces_file.empty();
}

template <typename ModelType, bool StoreExploredStates>
std::vector<uint_fast32_t> StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::getThreadSeeds() const {
    // Get an initial seed for random sampling in each thread
    const uint_fast32_t init_random_seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::vector<uint_fast32_t> thread_seeds{};
    thread_seeds.reserve(_settings.n_threads);
    for (size_t iter = 0U; iter < _settings.n_threads; ++iter) {
        thread_seeds.emplace_back(init_random_seed + iter);
    }
    return thread_seeds;
}

template <typename ModelType, bool StoreExploredStates>
StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::StatisticalModelCheckingEngine(
    const storm::storage::SymbolicModelDescription& in_model, const settings::SmcSettings& settings)
    : _model{in_model}, _settings{settings} {
    // Verify model validity
    STORM_LOG_THROW(verifyModelValid(), storm::exceptions::InvalidModelException, "Invalid model provided: cannot evaluate!");
    STORM_LOG_THROW(verifySettingsValid(), storm::exceptions::InvalidSettingsException, "Invalid settings provided: cannot evaluate!");
}

template <typename ModelType, bool StoreExploredStates>
bool StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::canHandle(
    const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task) const {
    return canHandleStatic(check_task);
}

template <typename ModelType, bool StoreExploredStates>
bool StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::verifyModelValid() const {
    bool is_model_valid = false;
    bool is_model_deterministic = false;
    if (_model.isJaniModel()) {
        is_model_valid = true;
        is_model_deterministic = _model.asJaniModel().isDeterministicModel();
    } else if (_model.isPrismProgram()) {
        is_model_valid = true;
        is_model_deterministic = _model.asPrismProgram().isDeterministicModel();
    }
    STORM_LOG_ERROR_COND(is_model_valid, "The input model is neither in Jani nor in PRISM. This cannot be verified!");
    return is_model_valid;
}

template <typename ModelType, bool StoreExploredStates>
bool StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::verifySettingsValid() const {
    bool is_settings_valid = true;
    if (_settings.n_threads == 0U) {
        STORM_LOG_ERROR("The number of threads must be greater than 0!");
        is_settings_valid = false;
    }
    if (traceStorageEnabled()) {
        if (_settings.n_threads > 1U) {
            STORM_LOG_ERROR("Traces can only be stored when using a single thread!");
            is_settings_valid = false;
        }
        STORM_LOG_WARN_COND(
            _settings.max_n_traces > 0U,
            "The amount of generated traces might be very large if left unbounded. Consider setting `--max-n-traces`.");
    } else {
        STORM_LOG_WARN_COND(
            _settings.max_n_traces == 0U, "The amount of generated traces is bounded. This might affect reliability of the results.");
    }
    return is_settings_valid;
}

template <typename ModelType, bool StoreExploredStates>
std::unique_ptr<storm::modelchecker::CheckResult> StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::computeProbabilities(
    const storm::Environment& env, const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task) {
    // Prepare the results holder
    samples::SamplingResults sampling_results(_settings, state_properties::PropertyType::P);

    const auto thread_seeds = getThreadSeeds();
    std::vector<std::thread> thread_instances;
    thread_instances.reserve(thread_seeds.size());
    for (size_t thread_id = 0U; thread_id < _settings.n_threads; ++thread_id) {
        const size_t thread_seed = thread_seeds[thread_id];
        thread_instances.emplace_back([this, &check_task, &sampling_results, thread_seed, thread_id]() {
            const samples::ModelSampling<StateType, ValueType> model_sampler(thread_seed);
            performSampling(check_task, model_sampler, sampling_results, thread_id);
        });
    }

    for (auto& instance : thread_instances) {
        instance.join();
    }

    const double estimated_success_prob = sampling_results.getProbabilityVerifiedProperty();

    if (_settings.show_statistics) {
        sampling_results.printResults();
    }

    return std::make_unique<storm::modelchecker::ExplicitQuantitativeCheckResult<ValueType>>(
        estimated_success_prob, estimated_success_prob);
}

template <typename ModelType, bool StoreExploredStates>
std::unique_ptr<storm::modelchecker::CheckResult>
StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::computeReachabilityRewards(
    const storm::Environment& env, const storm::modelchecker::CheckTask<storm::logic::EventuallyFormula, ValueType>& check_task) {
    // Prepare the results holder
    samples::SamplingResults sampling_results(_settings, state_properties::PropertyType::R);

    const auto thread_seeds = getThreadSeeds();
    std::vector<std::thread> thread_instances;
    thread_instances.reserve(thread_seeds.size());
    for (size_t thread_id = 0U; thread_id < _settings.n_threads; ++thread_id) {
        const size_t thread_seed = thread_seeds[thread_id];
        thread_instances.emplace_back([this, &check_task, &sampling_results, thread_seed, thread_id]() {
            const samples::ModelSampling<StateType, ValueType> model_sampler(thread_seed);
            performSampling(check_task, model_sampler, sampling_results, thread_id);
        });
    }

    for (auto& instance : thread_instances) {
        instance.join();
    }

    const double avg_reward = sampling_results.getEstimatedReward();

    if (_settings.show_statistics) {
        sampling_results.printResults();
    }

    return std::make_unique<storm::modelchecker::ExplicitQuantitativeCheckResult<ValueType>>(avg_reward, avg_reward);
}

template <typename ModelType, bool StoreExploredStates>
void StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::performSampling(
    const storm::modelchecker::CheckTask<storm::logic::Formula, ValueType>& check_task,
    const samples::ModelSampling<StateType, ValueType>& model_sampler, samples::SamplingResults& sampling_results, const size_t thread_id) {
    // TODO: Make sure that this is never set in case of P properties
    const std::string reward_model = check_task.isRewardModelSet() ? check_task.getRewardModel() : "";
    // Prepare exploration information holder.
    const bool export_traces = traceStorageEnabled();
    // Prepare object for sampling next states
    state_generation::StateGeneration<StateType, ValueType> state_generation(_model, check_task.getFormula(), reward_model, export_traces);

    if (export_traces) {
        // Prepare the traces export object
        _traces_exporter_ptr = std::make_unique<samples::TracesExporter>(_settings.traces_file, state_generation.getVariableInformation());
    }
    samples::BatchResults batch_results = sampling_results.getBatchResultInstance();

    // Now perform the actual sampling
    while (sampling_results.newBatchNeeded(thread_id)) {
        batch_results.reset();
        while (batch_results.batchIncomplete()) {
            // Sample a new path
            const auto result = samplePathFromInitialState(state_generation, model_sampler);
            batch_results.addResult(result);
        }
        sampling_results.addBatchResults(batch_results, thread_id);
    }
    _traces_exporter_ptr.reset();
}

template <typename ModelType, bool StoreExploredStates>
samples::TraceInformation StatisticalModelCheckingEngine<ModelType, StoreExploredStates>::samplePathFromInitialState(
    state_generation::StateGeneration<StateType, ValueType>& state_generation,
    const samples::ModelSampling<StateType, ValueType>& model_sampler) const {
    samples::TraceInformation trace_information;
    trace_information.trace_length = 0U;
    trace_information.outcome = samples::TraceResult::NO_INFO;
    trace_information.reward = 0.0;
    const auto& property = state_generation.getPropertyDescription();
    const bool check_global_property = property.getIsTerminalVerified();
    const size_t lower_bound = property.getLowerBound();
    const size_t upper_bound = property.getUpperBound();
    // As long as we didn't find a terminal (accepting or rejecting) state in the search, sample a new successor.
    samples::TraceResult path_result = samples::TraceResult::NO_INFO;
    state_generation.loadInitialState();
    while (path_result == samples::TraceResult::NO_INFO) {
        const auto& state_processing_result = state_generation.processLoadedState();
        if (_traces_exporter_ptr) {
            _traces_exporter_ptr->addNextState(state_processing_result.getCompressedState());
        }
        trace_information.reward += state_processing_result.getReward();

        // Do the check before emplacing back to get the correct n. of steps!
        const bool min_steps_done = trace_information.trace_length >= lower_bound;
        const auto& state_info = state_processing_result.getStateInfo();
        // Once here, we know the info related to the current state and we can decide whether to stop or continue!
        if (min_steps_done && state_properties::state_info::checkSatisfyTarget(state_info) && !check_global_property) {
            path_result = samples::TraceResult::VERIFIED;
        } else if (state_properties::state_info::checkBreakCondition(state_info)) {
            path_result = samples::TraceResult::NOT_VERIFIED;
        } else if (state_properties::state_info::checkIsTerminal(state_info)) {
            path_result = (check_global_property && min_steps_done) ? samples::TraceResult::VERIFIED : samples::TraceResult::NOT_VERIFIED;
        } else {
            // The state was neither a target nor a terminal state
            // Check if we reached the max n. of samples or we can continue sampling
            if (trace_information.trace_length >= upper_bound) {
                // If we went over the upper bound, stop sampling
                // TODO: Global properties might require special handling, however STORM has no support for Bounded Global properties
                STORM_LOG_TRACE("We reached the Upper bound. Stopping path expansion!");
                path_result = samples::TraceResult::NOT_VERIFIED;
            } else if (trace_information.trace_length >= _settings.max_trace_length && _settings.max_trace_length > 0) {
                // If the sampled path gets too large, abort!
                STORM_LOG_TRACE("Explored path longer than max value. Aborting!");
                break;
            } else {
                // Sample the next state by picking the next action.
                const size_t chosen_action_id = model_sampler.sampleActionOfState(state_processing_result);
                const StateType next_state = model_sampler.sampleSuccessorFromAction(chosen_action_id, state_processing_result);
                trace_information.reward += state_processing_result.getActions()[chosen_action_id].second;
                state_generation.load(next_state);
                ++trace_information.trace_length;
            }
        }
    }
    trace_information.outcome = path_result;
    // Export the trace results and start a new one, if requested
    if (_traces_exporter_ptr) {
        _traces_exporter_ptr->addCurrentTraceResult(trace_information);
    }
    return trace_information;
}

template class StatisticalModelCheckingEngine<storm::models::sparse::Dtmc<double>, true>;
template class StatisticalModelCheckingEngine<storm::models::sparse::Mdp<double>, true>;
template class StatisticalModelCheckingEngine<storm::models::sparse::Dtmc<double>, false>;
template class StatisticalModelCheckingEngine<storm::models::sparse::Mdp<double>, false>;
}  // namespace model_checker
}  // namespace smc_storm