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
#include <storm/exceptions/InvalidPropertyException.h>
#include <storm/exceptions/NotSupportedException.h>

#include "samples/exploration_information.h"
#include "model_checker/state_generation.h"

#include "model_checker/smc_engine.h"

namespace smc_storm {
namespace model_checker {
template<typename ModelType, typename StateType>
bool StatisticalExplorationModelChecker<ModelType, StateType>::canHandleStatic(storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& checkTask) {
    // Prepare the fragment defined our supported formulas
    // This allows all (atomic) property operators + the unary NOT logic operator on atomic properties (e.g. !F(s=2))
    storm::logic::FragmentSpecification const fragment =
        storm::logic::prctl().setNestedOperatorsAllowed(false).setUnaryBooleanPathFormulasAllowed(true).setOperatorAtTopLevelRequired(false);
    storm::logic::Formula const& formula = checkTask.getFormula();
    const bool isFormulaSupported = formula.isInFragment(fragment);
    const bool isOnlyInitStateRelevant = checkTask.isOnlyInitialStatesRelevantSet();
    // Conditional logging prints when the condition does not hold!
    STORM_LOG_WARN_COND(isFormulaSupported, "The input property " << formula.toString() << " is not supported by the SMC engine.");
    STORM_LOG_WARN_COND(isOnlyInitStateRelevant, "Property doesn't require only the initial state to be verified.");
    return isFormulaSupported && isOnlyInitStateRelevant;
}

template<typename ModelType, typename StateType>
std::vector<uint_fast32_t> StatisticalExplorationModelChecker<ModelType, StateType>::getThreadSeeds() const {
    // Get an initial seed for random sampling in each thread
    const uint_fast32_t initRandomSeed = std::chrono::system_clock::now().time_since_epoch().count();
    storm::utility::RandomProbabilityGenerator<ValueType> randomGenerator(initRandomSeed); 
    std::vector<uint_fast32_t> threadSeeds {};
    threadSeeds.reserve(_settings.n_threads);
    for (size_t iter = 0U; iter < _settings.n_threads; ++iter) {
        threadSeeds.emplace_back(randomGenerator.random_uint(0, std::numeric_limits<uint_fast32_t>::max()));
    }
    return threadSeeds;
}

template<typename ModelType, typename StateType>
StatisticalExplorationModelChecker<ModelType, StateType>::StatisticalExplorationModelChecker(storm::storage::SymbolicModelDescription const& inModel, const settings::SmcSettings& settings)
    : model{inModel},
      _settings{settings} {
    // Intentionally left empty.
}

template<typename ModelType, typename StateType>
bool StatisticalExplorationModelChecker<ModelType, StateType>::canHandle(storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& checkTask) const {
    return canHandleStatic(checkTask);
}

template<typename ModelType, typename StateType>
bool StatisticalExplorationModelChecker<ModelType, StateType>::verifyModelValid() const {
    bool isModelValid = false;
    bool isModelDeterministic = false;
    if (model.isJaniModel()) {
        isModelValid = true;
        isModelDeterministic = model.asJaniModel().isDeterministicModel();
    } else if (model.isPrismProgram()) {
        isModelValid = true;
        isModelDeterministic = model.asPrismProgram().isDeterministicModel();
    }
    STORM_LOG_ERROR_COND(isModelValid, "The input model is neither in Jani nor in PRISM. This cannot be verified!");
    return isModelValid;
}

template<typename ModelType, typename StateType>
std::unique_ptr<storm::modelchecker::CheckResult> StatisticalExplorationModelChecker<ModelType, StateType>::computeProbabilities(
    storm::Environment const& env, storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& checkTask) {
    // Verify model validity
    STORM_LOG_THROW(verifyModelValid(), storm::exceptions::InvalidPropertyException,
                    "Invalid model provided: cannot evaluate!");
    // Prepare the results holder
    const size_t batchSize = 100U;
    samples::SamplingResults samplingResults(batchSize, samples::SamplingResults::PropertyType::P, _settings.epsilon, _settings.confidence, _settings.stat_method);

    const auto threadSeeds = getThreadSeeds();
    std::vector<std::thread> threadInstances;
    threadInstances.reserve(threadSeeds.size());
    for (const auto& threadSeed : threadSeeds) {
        threadInstances.emplace_back([this, &checkTask, &samplingResults, threadSeed](){
            const samples::ModelSampling<StateType, ValueType> modelSampler(threadSeed);
            performProbabilitySampling(checkTask, modelSampler, samplingResults);
        });
    }

    for (auto& instance : threadInstances) {
        instance.join();
    }

    const double estimatedSuccessProb = samplingResults.getProbabilityVerifiedProperty();

    if (_settings.show_statistics) {
        samplingResults.printResults();
    }

    return std::make_unique<storm::modelchecker::ExplicitQuantitativeCheckResult<ValueType>>(estimatedSuccessProb, estimatedSuccessProb);
}


template<typename ModelType, typename StateType>
std::unique_ptr<storm::modelchecker::CheckResult> StatisticalExplorationModelChecker<ModelType, StateType>::computeReachabilityRewards(
    storm::Environment const& env, storm::logic::RewardMeasureType rewardType, storm::modelchecker::CheckTask<storm::logic::EventuallyFormula, ValueType> const& checkTask)
{
    STORM_LOG_THROW(rewardType == storm::logic::RewardMeasureType::Expectation, storm::exceptions::NotSupportedException, "Variance reward measures not supported.");
    // Prepare the results holder
    const size_t batchSize = 100U;
    samples::SamplingResults samplingResults(batchSize, samples::SamplingResults::PropertyType::R, _settings.epsilon, _settings.confidence, _settings.stat_method);

    const auto threadSeeds = getThreadSeeds();
    std::vector<std::thread> threadInstances;
    threadInstances.reserve(threadSeeds.size());
    for (const auto& threadSeed : threadSeeds) {
        threadInstances.emplace_back([this, &checkTask, &samplingResults, threadSeed](){
            const samples::ModelSampling<StateType, ValueType> modelSampler(threadSeed);
            performRewardSampling(checkTask, modelSampler, samplingResults);
        });
    }

    for (auto& instance : threadInstances) {
        instance.join();
    }

    const double avgReward = samplingResults.getEstimatedReward();

    if (_settings.show_statistics) {
        samplingResults.printResults();
    }

    return std::make_unique<storm::modelchecker::ExplicitQuantitativeCheckResult<ValueType>>(avgReward, avgReward);
}

template<typename ModelType, typename StateType>
void StatisticalExplorationModelChecker<ModelType, StateType>::performProbabilitySampling(
    storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& checkTask, samples::ModelSampling<StateType, ValueType> const& modelSampler,
    samples::SamplingResults& samplingResults) const
{
    // Prepare exploration information holder.
    samples::ExplorationInformation<StateType, ValueType> explorationInformation;
    explorationInformation.newRowGroup(0);

    // Prepare object for sampling next states
    StateGeneration<StateType, ValueType> stateGeneration(model, checkTask.getFormula(), explorationInformation);
    // Generate the initial state so we know where to start the simulation.
    stateGeneration.computeInitialStates();
    STORM_LOG_THROW(stateGeneration.getNumberOfInitialStates() == 1, storm::exceptions::NotSupportedException,
                    "Currently only models with one initial state are supported by the exploration engine.");

    // TODO: The property type is known to the samplingResults object: consider generating the Batch container from there
    samples::SamplingResults::BatchResults batchResults(samplingResults.getBatchSize(), samples::SamplingResults::PropertyType::P);

    // Now perform the actual sampling
    while (samplingResults.newBatchNeeded()) {
        batchResults.reset();
        while (batchResults.batchIncomplete()){
            // Sample a new path
            const auto result = samplePathFromInitialState(stateGeneration, explorationInformation, modelSampler);
            batchResults.addResult(result);
        }
        samplingResults.addBatchResults(batchResults);
    }
}

template<typename ModelType, typename StateType>
void StatisticalExplorationModelChecker<ModelType, StateType>::performRewardSampling(
    storm::modelchecker::CheckTask<storm::logic::Formula, ValueType> const& checkTask, samples::ModelSampling<StateType, ValueType> const& modelSampler, samples::SamplingResults& samplingResults) const
{
    // Prepare exploration information holder.
    samples::ExplorationInformation<StateType, ValueType> explorationInformation;
    explorationInformation.newRowGroup(0);

    // Prepare object for sampling next states
    StateGeneration<StateType, ValueType> stateGeneration(model, checkTask.getFormula(), checkTask.getRewardModel(), explorationInformation);
    // Generate the initial state so we know where to start the simulation.
    stateGeneration.computeInitialStates();
    STORM_LOG_THROW(stateGeneration.getNumberOfInitialStates() == 1, storm::exceptions::NotSupportedException,
                    "Currently only models with one initial state are supported by the exploration engine.");

    samples::SamplingResults::BatchResults batchResults(samplingResults.getBatchSize(), samples::SamplingResults::PropertyType::R);

    // Now perform the actual sampling
    while (samplingResults.newBatchNeeded()) {
        batchResults.reset();
        while (batchResults.batchIncomplete()) {
            // Sample a new path
            const auto result = samplePathFromInitialState(stateGeneration, explorationInformation, modelSampler);
            batchResults.addResult(result);
        }
        samplingResults.addBatchResults(batchResults);
    }
}

template<typename ModelType, typename StateType>
samples::TraceInformation StatisticalExplorationModelChecker<ModelType, StateType>::samplePathFromInitialState(
    StateGeneration<StateType, ValueType>& stateGeneration, samples::ExplorationInformation<StateType, ValueType>& explorationInformation,
    samples::ModelSampling<StateType, ValueType> const& modelSampler) const
{
    samples::TraceInformation traceInformation;
    traceInformation.traceLength = 0U;
    traceInformation.outcome = samples::TraceResult::NO_INFO;
    traceInformation.reward = 0.0;
    // Start the search from the initial state.
    StateType currentStateId = stateGeneration.getFirstInitialState();

    const bool checkGlobalProperty = stateGeneration.getIsTerminalVerified();

    // As long as we didn't find a terminal (accepting or rejecting) state in the search, sample a new successor.
    samples::TraceResult pathResult = samples::TraceResult::NO_INFO;
    while (pathResult == samples::TraceResult::NO_INFO) {
        STORM_LOG_TRACE("Current state is: " << currentStateId << ".");
        properties::StateInfoType stateInfo = properties::state_info::NO_INFO;

        // If the state is not yet explored, we need to retrieve its behaviors.
        if (explorationInformation.isUnexplored(currentStateId)) {
            STORM_LOG_TRACE("State was not yet explored.");
            // Explore the previously unexplored state.
            stateInfo = exploreState(stateGeneration, currentStateId, explorationInformation);
        } else {
            stateInfo = explorationInformation.getStateInfo(currentStateId);
        }

        // Add the reward for being at the current state
        if (stateGeneration.rewardLoaded()) {
            // TODO: this if statement is extremely redundant!
            traceInformation.reward += explorationInformation.getStateReward(currentStateId);
        }

        // Do the check before emplacing back to get the correct n. of steps!
        const bool minStepsDone = traceInformation.traceLength >= stateGeneration.getLowerBound();
        // Once here, we know the info related to the current state and we can decide whether to stop or continue!
        if (minStepsDone && properties::state_info::checkSatisfyTarget(stateInfo) && !checkGlobalProperty) {
            pathResult = samples::TraceResult::VERIFIED;
        } else if (properties::state_info::checkBreakCondition(stateInfo)) {
            pathResult = samples::TraceResult::NOT_VERIFIED;
        } else if (properties::state_info::checkIsTerminal(stateInfo)) {
            pathResult = (checkGlobalProperty && minStepsDone) ? samples::TraceResult::VERIFIED : samples::TraceResult::NOT_VERIFIED;
        } else {
            // The state was neither a target nor a terminal state
            // Check if we reached the max n. of samples or we can continue sampling
            if (traceInformation.traceLength >= stateGeneration.getUpperBound()) {
                // If we went over the upper bound, stop sampling
                // TODO: Global properties might require special handling, however STORM has no support for Bounded Global properties
                STORM_LOG_TRACE("We reached the Upper bound. Stopping path expansion!");
                pathResult = samples::TraceResult::NOT_VERIFIED;
            } else if(traceInformation.traceLength >= _settings.max_trace_length && _settings.max_trace_length > 0) {
                // If the sampled path gets too large, abort!
                STORM_LOG_TRACE("Explored path longer than max value. Aborting!");
                break;
            } else {
                // Sample the next state by picking the next action.
                const ActionType chosenAction = modelSampler.sampleActionOfState(currentStateId, explorationInformation);
                STORM_LOG_TRACE("Sampled action " << chosenAction << " in state " << currentStateId << ".");
                // Add reward for the chosen action
                if (stateGeneration.rewardLoaded()) {
                    // TODO: Redundant if!
                    traceInformation.reward += explorationInformation.getActionReward(chosenAction);
                }
                // Get the next state given the chosen action and the prob. distr. of reachable states.
                const StateType nextStateId = modelSampler.sampleSuccessorFromAction(chosenAction, explorationInformation);
                STORM_LOG_TRACE("Sampled successor " << nextStateId << " according to action " << chosenAction << " of state " << currentStateId << ".");
                // Update the n. of steps and current state
                currentStateId = nextStateId;
                ++traceInformation.traceLength;
            }
        }
    }
    traceInformation.outcome = pathResult;
    return traceInformation;
}

template<typename ModelType, typename StateType>
properties::StateInfoType StatisticalExplorationModelChecker<ModelType, StateType>::exploreState(
        StateGeneration<StateType, ValueType>& stateGeneration, StateType const& currentStateId,
        samples::ExplorationInformation<StateType, ValueType>& explorationInformation) const {
    // At start, we know nothing about this state
    properties::StateInfoType stateInfo = properties::state_info::NO_INFO;
    const size_t rewardIndex = stateGeneration.getRewardIndex();
    const bool rewardsNeeded = stateGeneration.rewardLoaded();

    auto unexploredIt = explorationInformation.findUnexploredState(currentStateId);
    STORM_LOG_THROW(explorationInformation.unexploredStatesEnd() != unexploredIt, storm::exceptions::UnexpectedException, "Cannot retrieve unexplored state.");
    storm::generator::CompressedState const& compressedState = unexploredIt->second;

    explorationInformation.assignStateToNextRowGroup(currentStateId);
    STORM_LOG_TRACE("Assigning row group " << explorationInformation.getRowGroup(currentStateId) << " to state " << currentStateId << ".");
    stateGeneration.load(compressedState);

    // Get the expanded current state
    storm::generator::StateBehavior<ValueType, StateType> const expandedState = stateGeneration.expand();

    if (rewardsNeeded) {
        explorationInformation.addStateReward(currentStateId, expandedState.getStateRewards().at(rewardIndex));
    }

    if (stateGeneration.isTargetState()) {
        stateInfo |= properties::state_info::SATISFY_TARGET;
    }
    if (stateGeneration.isConditionState()) {
        // Clumsily check whether we have found a state that forms a trivial BMEC.
        bool otherSuccessor = false;
        for (auto const& choice : expandedState) {
            for (auto const& [nextStateId, _] : choice) {
                if (nextStateId != currentStateId) {
                    // We found a successor that goes to a new state, no termination yet!
                    otherSuccessor = true;
                    break;
                }
            }
        }
        if (!otherSuccessor) {
            // Can't find a successor: we reached a terminal state
            stateInfo |= properties::state_info::IS_TERMINAL;
        } else {
            // If the state was neither a trivial (non-accepting) terminal state nor a target state, we
            // need to store its behavior.
            // Next, we insert the state + related actions into our matrix structure.
            StateType startAction = explorationInformation.getActionCount();
            explorationInformation.addActionsToMatrix(expandedState.getNumberOfChoices());

            ActionType localAction = 0;

            for (auto const& singleAction : expandedState) {
                const ActionType actionId = startAction + localAction;
                if (rewardsNeeded) {
                    explorationInformation.addActionReward(actionId, singleAction.getRewards().at(rewardIndex));
                }
                for (auto const& [nextStateId, likelihood] : singleAction) {
                    explorationInformation.getRowOfMatrix(actionId).emplace_back(nextStateId, likelihood);
                    STORM_LOG_TRACE("Found transition " << currentStateId << "-[" << (actionId) << ", " << likelihood << "]-> "
                                                        << nextStateId << ".");
                }
                ++localAction;
            }

            // Terminate the row group.
            explorationInformation.terminateCurrentRowGroup();
        }

    } else {
        stateInfo |= properties::state_info::BREAK_CONDITION;
    }

    if (!properties::state_info::checkNoInfo(stateInfo)) {
        explorationInformation.addStateInfo(currentStateId, stateInfo);
    }

    // Check whether we need to add a dummy action to the exploration information object
    if (properties::state_info::checkBreakCondition(stateInfo) || properties::state_info::checkIsTerminal(stateInfo)) {
        explorationInformation.addActionsToMatrix(1);
        explorationInformation.newRowGroup();
    }
    explorationInformation.removeUnexploredState(unexploredIt);
    return stateInfo;
}

template class StatisticalExplorationModelChecker<storm::models::sparse::Dtmc<double>, uint32_t>;
template class StatisticalExplorationModelChecker<storm::models::sparse::Mdp<double>, uint32_t>;
}  // namespace model_checker
}  // namespace smc_storm