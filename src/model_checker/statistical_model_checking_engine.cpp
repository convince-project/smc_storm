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

#include <thread>
#include <filesystem>

#include <storm/storage/expressions/ExpressionEvaluator.h>

#include <storm/generator/CompressedState.h>

#include <storm/storage/MaximalEndComponentDecomposition.h>
#include <storm/storage/SparseMatrix.h>

#include <storm/storage/prism/Program.h>
#include <storm/storage/jani/Model.h>

#include <storm/logic/FragmentSpecification.h>

#include <storm/modelchecker/results/ExplicitQuantitativeCheckResult.h>

#include <storm/models/sparse/Dtmc.h>
#include <storm/models/sparse/Mdp.h>
#include <storm/models/sparse/StandardRewardModel.h>

#include <storm/settings/SettingsManager.h>
#include <storm/settings/modules/CoreSettings.h>
#include <storm/settings/modules/ExplorationSettings.h>

#include <storm/utility/constants.h>
#include <storm/utility/graph.h>
#include <storm/utility/macros.h>
#include <storm/utility/prism.h>
#include <storm/utility/random.h>

#include <storm/exceptions/InvalidOperationException.h>
#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidSettingsException.h>
#include <storm/exceptions/InvalidPropertyException.h>

#include "state_properties/property_type.hpp"
#include "samples/exploration_information.h"
#include "model_checker/state_generation.h"

#include "model_checker/statistical_model_checking_engine.hpp"

namespace smc_storm {
namespace model_checker {
template<typename ModelType, typename StateType>
bool StatisticalModelCheckingEngine<ModelType, StateType>::canHandleStatic(storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task) {
    // Prepare the fragment defined our supported formulas
    // This allows all (atomic) property operators + the unary NOT logic operator on atomic properties (e.g. !F(s=2))
    storm::logic::FragmentSpecification const fragment =
        storm::logic::prctl().setNestedOperatorsAllowed(false).setUnaryBooleanPathFormulasAllowed(true).setOperatorAtTopLevelRequired(false);
    storm::logic::Formula const& formula = check_task.getFormula();
    const bool is_formula_supported = formula.isInFragment(fragment);
    const bool is_only_init_state_relevant = check_task.isOnlyInitialStatesRelevantSet();
    // Conditional logging prints when the condition does not hold!
    STORM_LOG_WARN_COND(is_formula_supported, "The input property " << formula.toString() << " is not supported by the SMC engine.");
    STORM_LOG_WARN_COND(is_only_init_state_relevant, "Property doesn't require only the initial state to be verified.");
    return is_formula_supported && is_only_init_state_relevant;
}

template<typename ModelType, typename StateType>
bool StatisticalModelCheckingEngine<ModelType, StateType>::traceStorageEnabled() const {
    return !_settings.traces_file.empty();
}

template<typename ModelType, typename StateType>
std::vector<uint_fast32_t> StatisticalModelCheckingEngine<ModelType, StateType>::getThreadSeeds() const {
    // Get an initial seed for random sampling in each thread
    const uint_fast32_t init_random_seed = std::chrono::system_clock::now().time_since_epoch().count();
    storm::utility::RandomProbabilityGenerator<ValueType> random_generator(init_random_seed); 
    std::vector<uint_fast32_t> thread_seeds {};
    thread_seeds.reserve(_settings.n_threads);
    for (size_t iter = 0U; iter < _settings.n_threads; ++iter) {
        thread_seeds.emplace_back(random_generator.random_uint(0, std::numeric_limits<uint_fast32_t>::max()));
    }
    return thread_seeds;
}

template<typename ModelType, typename StateType>
StatisticalModelCheckingEngine<ModelType, StateType>::StatisticalModelCheckingEngine(storm::storage::SymbolicModelDescription const& in_model, const settings::SmcSettings& settings)
: _model{in_model},
  _settings{settings} {
    // Verify model validity
    STORM_LOG_THROW(verifyModelValid(), storm::exceptions::InvalidModelException, "Invalid model provided: cannot evaluate!");
    STORM_LOG_THROW(verifySettingsValid(), storm::exceptions::InvalidSettingsException, "Invalid settings provided: cannot evaluate!");
}

template<typename ModelType, typename StateType>
bool StatisticalModelCheckingEngine<ModelType, StateType>::canHandle(storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task) const {
    return canHandleStatic(check_task);
}

template<typename ModelType, typename StateType>
bool StatisticalModelCheckingEngine<ModelType, StateType>::verifyModelValid() const {
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

template<typename ModelType, typename StateType>
bool StatisticalModelCheckingEngine<ModelType, StateType>::verifySettingsValid() const {
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
        STORM_LOG_WARN_COND(_settings.max_n_traces > 0U, "The amount of generated traces might be very large if left unbounded. Consider setting `--max-n-traces`.");
    } else {
        STORM_LOG_WARN_COND(_settings.max_n_traces == 0U, "The amount of generated traces is bounded. This might affect reliability of the results.");
    }
    return is_settings_valid;
}

template<typename ModelType, typename StateType>
std::unique_ptr<storm::modelchecker::CheckResult> StatisticalModelCheckingEngine<ModelType, StateType>::computeProbabilities(
    storm::Environment const& env, storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task) {
    // Prepare the results holder
    samples::SamplingResults sampling_results(_settings, state_properties::PropertyType::P);

    const auto thread_seeds = getThreadSeeds();
    std::vector<std::thread> thread_instances;
    thread_instances.reserve(thread_seeds.size());
    for (const auto& thread_seed : thread_seeds) {
        thread_instances.emplace_back([this, &check_task, &sampling_results, thread_seed](){
            const samples::ModelSampling<StateType, ValueType> model_sampler(thread_seed);
            performSampling(check_task, model_sampler, sampling_results);
        });
    }

    for (auto& instance : thread_instances) {
        instance.join();
    }

    const double estimated_success_prob = sampling_results.getProbabilityVerifiedProperty();

    if (_settings.show_statistics) {
        sampling_results.printResults();
    }

    return std::make_unique<storm::modelchecker::ExplicitQuantitativeCheckResult<ValueType>>(estimated_success_prob, estimated_success_prob);
}


template<typename ModelType, typename StateType>
std::unique_ptr<storm::modelchecker::CheckResult> StatisticalModelCheckingEngine<ModelType, StateType>::computeReachabilityRewards(
    storm::Environment const& env, storm::logic::RewardMeasureType reward_type, storm::modelchecker::CheckTask<storm::logic::EventuallyFormula, ValueType> const& check_task)
{
    STORM_LOG_THROW(reward_type == storm::logic::RewardMeasureType::Expectation, storm::exceptions::InvalidPropertyException, "Variance reward measures not supported.");
    // Prepare the results holder
    samples::SamplingResults sampling_results(_settings, state_properties::PropertyType::R);

    const auto thread_seeds = getThreadSeeds();
    std::vector<std::thread> thread_instances;
    thread_instances.reserve(thread_seeds.size());
    for (const auto& thread_seed : thread_seeds) {
        thread_instances.emplace_back([this, &check_task, &sampling_results, thread_seed](){
            const samples::ModelSampling<StateType, ValueType> model_sampler(thread_seed);
            performSampling(check_task, model_sampler, sampling_results);
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

template<typename ModelType, typename StateType>
void StatisticalModelCheckingEngine<ModelType, StateType>::performSampling(
    storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& check_task, samples::ModelSampling<StateType, ValueType> const& model_sampler,
    samples::SamplingResults& sampling_results)
{
    // TODO: Make sure that this is never set in case of P properties
    const std::string reward_model = check_task.isRewardModelSet() ? check_task.getRewardModel() : "";
    // Prepare exploration information holder.
    const bool export_traces = traceStorageEnabled();
    samples::ExplorationInformation<StateType, ValueType> exploration_information(export_traces);
    exploration_information.newRowGroup(0);

    // Prepare object for sampling next states
    StateGeneration<StateType, ValueType> state_generation(_model, check_task.getFormula(), reward_model, exploration_information);
    // Generate the initial state so we know where to start the simulation.
    state_generation.computeInitialStates();
    STORM_LOG_THROW(state_generation.getNumberOfInitialStates() == 1, storm::exceptions::InvalidModelException,
                    "Currently only models with one initial state are supported by the exploration engine.");

    if (export_traces) {   
        // Prepare the traces export object
        _traces_exporter_ptr = std::make_unique<samples::TracesExporter>(_settings.traces_file, state_generation.getVariableInformation());
    }
    samples::BatchResults batch_results = sampling_results.getBatchResultInstance();

    // Now perform the actual sampling
    while (sampling_results.newBatchNeeded()) {
        batch_results.reset();
        while (batch_results.batchIncomplete()){
            // Sample a new path
            const auto result = samplePathFromInitialState(state_generation, exploration_information, model_sampler);
            batch_results.addResult(result);
        }
        sampling_results.addBatchResults(batch_results);
    }
    _traces_exporter_ptr.reset();
}

template<typename ModelType, typename StateType>
samples::TraceInformation StatisticalModelCheckingEngine<ModelType, StateType>::samplePathFromInitialState(
    StateGeneration<StateType, ValueType>& state_generation, samples::ExplorationInformation<StateType, ValueType>& exploration_information,
    samples::ModelSampling<StateType, ValueType> const& model_sampler) const
{
    samples::TraceInformation trace_information;
    trace_information.trace_length = 0U;
    trace_information.outcome = samples::TraceResult::NO_INFO;
    trace_information.reward = 0.0;
    // Start the search from the initial state.
    StateType current_state_id = state_generation.getFirstInitialState();

    const bool check_global_property = state_generation.getIsTerminalVerified();

    // As long as we didn't find a terminal (accepting or rejecting) state in the search, sample a new successor.
    samples::TraceResult path_result = samples::TraceResult::NO_INFO;
    while (path_result == samples::TraceResult::NO_INFO) {
        STORM_LOG_TRACE("Current state is: " << current_state_id << ".");
        // If we need to export the traces, the exporter instance will exist
        if (_traces_exporter_ptr) {
            _traces_exporter_ptr->addNextState(exploration_information.getCompressedState(current_state_id));
        }
        state_properties::StateInfoType state_info = state_properties::state_info::NO_INFO;

        // If the state is not yet explored, we need to retrieve its behaviors.
        if (exploration_information.isUnexplored(current_state_id)) {
            STORM_LOG_TRACE("State was not yet explored.");
            // Explore the previously unexplored state.
            state_info = exploreState(state_generation, current_state_id, exploration_information);
        } else {
            state_info = exploration_information.getStateInfo(current_state_id);
        }

        // Add the reward for being at the current state
        if (state_generation.rewardLoaded()) {
            // TODO: this if statement is extremely redundant!
            trace_information.reward += exploration_information.getStateReward(current_state_id);
        }

        // Do the check before emplacing back to get the correct n. of steps!
        const bool min_steps_done = trace_information.trace_length >= state_generation.getLowerBound();
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
            if (trace_information.trace_length >= state_generation.getUpperBound()) {
                // If we went over the upper bound, stop sampling
                // TODO: Global properties might require special handling, however STORM has no support for Bounded Global properties
                STORM_LOG_TRACE("We reached the Upper bound. Stopping path expansion!");
                path_result = samples::TraceResult::NOT_VERIFIED;
            } else if(trace_information.trace_length >= _settings.max_trace_length && _settings.max_trace_length > 0) {
                // If the sampled path gets too large, abort!
                STORM_LOG_TRACE("Explored path longer than max value. Aborting!");
                break;
            } else {
                // Sample the next state by picking the next action.
                const ActionType chosen_action = model_sampler.sampleActionOfState(current_state_id, exploration_information);
                STORM_LOG_TRACE("Sampled action " << chosen_action << " in state " << current_state_id << ".");
                // Add reward for the chosen action
                if (state_generation.rewardLoaded()) {
                    // TODO: Redundant if!
                    trace_information.reward += exploration_information.getActionReward(chosen_action);
                }
                // Get the next state given the chosen action and the prob. distr. of reachable states.
                const StateType next_state_id = model_sampler.sampleSuccessorFromAction(chosen_action, exploration_information);
                STORM_LOG_TRACE("Sampled successor " << next_state_id << " according to action " << chosen_action << " of state " << current_state_id << ".");
                // Update the n. of steps and current state
                current_state_id = next_state_id;
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

template<typename ModelType, typename StateType>
state_properties::StateInfoType StatisticalModelCheckingEngine<ModelType, StateType>::exploreState(
        StateGeneration<StateType, ValueType>& state_generation, StateType const& current_state_id,
        samples::ExplorationInformation<StateType, ValueType>& exploration_information) const {
    // At start, we know nothing about this state
    state_properties::StateInfoType state_info = state_properties::state_info::NO_INFO;
    const size_t reward_index = state_generation.getRewardIndex();
    const bool rewards_needed = state_generation.rewardLoaded();

    auto unexplored_it = exploration_information.findUnexploredState(current_state_id);
    STORM_LOG_THROW(exploration_information.unexploredStatesEnd() != unexplored_it, storm::exceptions::UnexpectedException, "Cannot retrieve unexplored state.");
    storm::generator::CompressedState const& compressed_state = unexplored_it->second;

    exploration_information.assignStateToNextRowGroup(current_state_id);
    STORM_LOG_TRACE("Assigning row group " << exploration_information.getRowGroup(current_state_id) << " to state " << current_state_id << ".");
    state_generation.load(compressed_state);

    // Get the expanded current state
    storm::generator::StateBehavior<ValueType, StateType> const expanded_state = state_generation.expand();

    if (rewards_needed) {
        exploration_information.addStateReward(current_state_id, expanded_state.getStateRewards().at(reward_index));
    }

    if (state_generation.isTargetState()) {
        state_info |= state_properties::state_info::SATISFY_TARGET;
    }
    if (state_generation.isConditionState()) {
        // Clumsily check whether we have found a state that forms a trivial BMEC.
        bool otherSuccessor = false;
        for (auto const& choice : expanded_state) {
            for (auto const& [next_state_id, _] : choice) {
                if (next_state_id != current_state_id) {
                    // We found a successor that goes to a new state, no termination yet!
                    otherSuccessor = true;
                    break;
                }
            }
        }
        if (!otherSuccessor) {
            // Can't find a successor: we reached a terminal state
            state_info |= state_properties::state_info::IS_TERMINAL;
        } else {
            // If the state was neither a trivial (non-accepting) terminal state nor a target state, we
            // need to store its behavior.
            // Next, we insert the state + related actions into our matrix structure.
            StateType start_action = exploration_information.getActionCount();
            exploration_information.addActionsToMatrix(expanded_state.getNumberOfChoices());

            ActionType local_action = 0;

            for (auto const& single_action : expanded_state) {
                const ActionType action_id = start_action + local_action;
                if (rewards_needed) {
                    exploration_information.addActionReward(action_id, single_action.getRewards().at(reward_index));
                }
                for (auto const& [next_state_id, likelihood] : single_action) {
                    exploration_information.getRowOfMatrix(action_id).emplace_back(next_state_id, likelihood);
                    STORM_LOG_TRACE("Found transition " << current_state_id << "-[" << (action_id) << ", " << likelihood << "]-> "
                                                        << next_state_id << ".");
                }
                ++local_action;
            }

            // Terminate the row group.
            exploration_information.terminateCurrentRowGroup();
        }

    } else {
        state_info |= state_properties::state_info::BREAK_CONDITION;
    }

    if (!state_properties::state_info::checkNoInfo(state_info)) {
        exploration_information.addStateInfo(current_state_id, state_info);
    }

    // Check whether we need to add a dummy action to the exploration information object
    if (state_properties::state_info::checkBreakCondition(state_info) || state_properties::state_info::checkIsTerminal(state_info)) {
        exploration_information.addActionsToMatrix(1);
        exploration_information.newRowGroup();
    }
    exploration_information.removeUnexploredState(unexplored_it);
    return state_info;
}

template class StatisticalModelCheckingEngine<storm::models::sparse::Dtmc<double>, uint32_t>;
template class StatisticalModelCheckingEngine<storm::models::sparse::Mdp<double>, uint32_t>;
}  // namespace model_checker
}  // namespace smc_storm