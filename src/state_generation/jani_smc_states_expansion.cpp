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

#include <storm/storage/jani/AutomatonComposition.h>
#include <storm/storage/jani/ParallelComposition.h>
#include <storm/storage/jani/traverser/RewardModelInformation.h>

#include <storm/solver/SmtSolver.h>

#include <storm/exceptions/IllegalFunctionCallException.h>
#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/NotImplementedException.h>
#include <storm/exceptions/NotSupportedException.h>
#include <storm/exceptions/WrongFormatException.h>

#include "state_generation/jani_smc_states_expansion.hpp"

namespace smc_storm::state_generation {
template <typename ValueType>
JaniSmcStatesExpansion<ValueType>::JaniSmcStatesExpansion(
    const storm::jani::Model& jani_model, const std::optional<std::string>& reward_name,
    const std::vector<model_checker::SmcPluginInstance>& external_plugins, std::default_random_engine& random_generator)
    : _random_generator{random_generator},
      _jani_model(jani_model), _external_plugins_desc{external_plugins}, _reward{
                                                                             storm::builder::RewardModelInformation(
                                                                                 "", false, false, false),
                                                                             storm::expressions::Expression(),
                                                                             storm::utility::zero<ValueType>()} {
    loadReward(reward_name);
    checkSupportedFeatures();
    loadPlugins();
    // This ensures an automaton appears only once in the model composition
    _jani_model.simplifyComposition();
    // For now, always execute edge assignments at destination.
    // Keep optimization from NextStateGenerator for later (the one for trivial reward expressions)
    // _jani_model.pushEdgeAssignmentsToDestinations();
    generateSynchInformation();
    checkUndefinedConstants();
    // For now the out of bounds config is set to the default value (32 bits + false out of bounds state)
    _variable_information = state_properties::StateVariableInformation<ValueType>(_jani_model, _system_model.parallel_automata, false);
    _transient_variable_information =
        storm::generator::TransientVariableInformation<ValueType>(_jani_model, _system_model.parallel_automata);
    // No special states to initialize for now
    _evaluator_ptr = std::make_unique<storm::expressions::ExpressionEvaluator<ValueType>>(_jani_model.getManager());
    _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
    // We assume there is no special terminal state to handle
    // We assume we do not need special state labels
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::checkSupportedFeatures() const {
    auto features = _jani_model.getModelFeatures();
    features.remove(storm::jani::ModelFeature::DerivedOperators);
    features.remove(storm::jani::ModelFeature::StateExitRewards);
    features.remove(storm::jani::ModelFeature::TrigonometricFunctions);
    STORM_LOG_THROW(
        features.empty(), storm::exceptions::NotSupportedException,
        "The SMC next-state generator does not support the following model feature(s): " << features.toString() << ".");
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::checkUndefinedConstants() const {
    if (_jani_model.hasUndefinedConstants()) {
        std::string undefined_constants = "";
        for (const auto& constant : _jani_model.getUndefinedConstants()) {
            if (!undefined_constants.empty()) {
                undefined_constants += ", ";
            }
            undefined_constants += constant.get().getName();
        }
        STORM_LOG_THROW(
            false, storm::exceptions::InvalidModelException,
            "Program still contains these undefined constants: " + undefined_constants + ".");
    }
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::loadPlugins() {
    _loaded_plugin_ptrs.clear();
    _loaded_plugin_ptrs.reserve(_external_plugins_desc.get().size());
    for (const auto& plugin_description : _external_plugins_desc.get()) {
        _loaded_plugin_ptrs.emplace_back(plugin_description.generatePluginInstance());
        STORM_LOG_THROW(_loaded_plugin_ptrs.back(), storm::exceptions::UnexpectedException, "Cannot load the requested plugin.");
        _loaded_plugin_ptrs.back()->setRandomSeed(_random_generator.get()());
        _loaded_plugin_ptrs.back()->loadParameters(plugin_description.getInitData());
    }
}

template <typename ValueType>
bool JaniSmcStatesExpansion<ValueType>::checkGlobalVariableWrittenOnce(const AutomataAndEdges& edge_set) const {
    std::unordered_map<storm::expressions::Variable, uint64_t> variable_to_automaton;
    for (const auto& [automaton_id, automaton_loc_to_edges] : edge_set) {
        STORM_LOG_THROW(automaton_loc_to_edges.size() == 1U, storm::exceptions::UnexpectedException, "Expected one location per automaton");
        for (const auto& [edge_idx, edge_ref] : automaton_loc_to_edges.begin()->second) {
            for (const auto& glob_var : edge_ref.get().getWrittenGlobalVariables()) {
                const auto& [emplace_it, emplace_success] = variable_to_automaton.emplace(glob_var, automaton_id);
                if (!emplace_success && emplace_it->second != automaton_id) {
                    return false;
                }
            }
        }
    }
    return true;
}

template <typename ValueType>
const state_properties::StateVariableData<ValueType>& JaniSmcStatesExpansion<ValueType>::setInitialState() {
    if (_initial_state.empty()) {
        // This is the first time we are initializing the model: compute the initial state
        _initial_state = _variable_information.generateVariableData();
        storm::utility::solver::SmtSolverFactory factory;
        std::unique_ptr<storm::solver::SmtSolver> solver = factory.create(_jani_model.getExpressionManager());
        for (const auto& range_expression : _jani_model.getAllRangeExpressions(_system_model.parallel_automata)) {
            solver->add(range_expression);
        }
        solver->add(_jani_model.getInitialStatesExpression(_system_model.parallel_automata));
        bool first_iteration{true};
        // Iterate as long as the CheckResult is Satisfied
        while (solver->check() == storm::solver::SmtSolver::CheckResult::Sat) {
            STORM_LOG_THROW(
                first_iteration, storm::exceptions::InvalidModelException,
                "Currently only models with one initial state are supported by the exploration engine.");
            first_iteration = false;
            size_t n_blocking_expressions = 0u;
            // Pointer to a possible solution evaluated by the solver
            auto solution_model_ptr = solver->getModel();
            // Evaluate booleans and update the blocking_expression
            for (size_t idx = 0U; idx < _variable_information.booleanVariables().size(); idx++) {
                const auto& bool_var = _variable_information.booleanVariables().at(idx);
                const bool variable_value = solution_model_ptr->getBooleanValue(bool_var.variable);
                // Add the negation of the current state variable as an assertion, to prevent the same state from being generated
                storm::expressions::Expression local_blocking_expression = variable_value ? !bool_var.variable : bool_var.variable;
                solver->add(local_blocking_expression);
                n_blocking_expressions++;
                _initial_state.setBool(idx, variable_value);
            }
            // Evaluate integers and update the blocking_expression
            for (size_t idx = 0U; idx < _variable_information.integerVariables().size(); idx++) {
                const auto& int_var = _variable_information.integerVariables().at(idx);
                const int_fast64_t variable_value = solution_model_ptr->getIntegerValue(int_var.variable);
                if (_variable_information.checkVariableBounds()) {
                    STORM_LOG_THROW(
                        !int_var.lower_bound || *int_var.lower_bound <= variable_value, storm::exceptions::WrongFormatException,
                        "The initial value for variable " << int_var.variable.getName() << " is lower than the lower bound.");
                    STORM_LOG_THROW(
                        !int_var.upper_bound || *int_var.upper_bound >= variable_value, storm::exceptions::WrongFormatException,
                        "The initial value for variable " << int_var.variable.getName() << " is higher than the upper bound.");
                }
                storm::expressions::Expression local_blocking_expression =
                    int_var.variable != solution_model_ptr->getManager().integer(variable_value);
                // Add the negation of the current state variable as an assertion, to prevent the same state from being generated
                solver->add(local_blocking_expression);
                n_blocking_expressions++;
                _initial_state.setInt(idx, variable_value);
            }
            // Evaluate real numbers and update the blocking_expression
            for (size_t idx = 0U; idx < _variable_information.realVariables().size(); idx++) {
                const auto& real_var = _variable_information.realVariables().at(idx);
                const ValueType variable_value = solution_model_ptr->getRationalValue(real_var.variable);
                if (_variable_information.checkVariableBounds()) {
                    STORM_LOG_THROW(
                        !real_var.lower_bound || *real_var.lower_bound <= variable_value, storm::exceptions::WrongFormatException,
                        "The initial value for variable " << real_var.variable.getName() << " is lower than the lower bound.");
                    STORM_LOG_THROW(
                        !real_var.upper_bound || *real_var.upper_bound >= variable_value, storm::exceptions::WrongFormatException,
                        "The initial value for variable " << real_var.variable.getName() << " is higher than the upper bound.");
                }
                storm::expressions::Expression local_blocking_expression =
                    real_var.variable != solution_model_ptr->getManager().rational(variable_value);
                // Add the negation of the current state variable as an assertion, to prevent the same state from being generated
                solver->add(local_blocking_expression);
                n_blocking_expressions++;
                _initial_state.setReal(idx, variable_value);
            }
            // Generate the initial location of all the automata
            for (size_t idx = 0U; idx < _system_model.parallel_automata.size(); idx++) {
                const storm::jani::Automaton& automaton = _system_model.parallel_automata.at(idx).get();
                const std::set<uint64_t>& init_locations = automaton.getInitialLocationIndices();
                STORM_LOG_THROW(
                    init_locations.size() == 1, storm::exceptions::InvalidModelException,
                    "The automaton " << automaton.getName() << "Has more than one initial states.");
                const uint64_t location_value = *init_locations.begin();  // Each automaton has only one initial state
                _initial_state.setLocation(idx, location_value);
            }
            if (n_blocking_expressions == 0u) {
                // Doing another loop would lead to the same solution
                break;
            }
        }
    }
    updateCurrentState(_initial_state);
    // Resetting all external plugins to the initial state.
    for (const auto& plugin_ptr : _loaded_plugin_ptrs) {
        // TODO: This returns data, that need to be assigned to the current state!
        plugin_ptr->reset();
    }
    return _current_state;
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::updateCurrentState(const state_properties::StateVariableData<ValueType>& state) {
    _current_state = state;
    _variable_information.setInEvaluator(*_evaluator_ptr, _current_state);
    if (_reward.information.hasStateRewards()) {
        // TODO: There might be the need of evaluating transient values at current location first (JaniNextStateGenerator.cpp:645)
        auto transient_variable_valuation = evaluateTransientVariablesAtLocations();
        transient_variable_valuation.setInEvaluator(*_evaluator_ptr, _additional_checks);
        _reward.current_state_reward = _evaluator_ptr->asRational(_reward.expression);
    }
    _computed_actions.clear();
    _computed_destinations.clear();
}

// Check if the current state satisfies a specific property
template <typename ValueType>
bool JaniSmcStatesExpansion<ValueType>::satisfies(const storm::expressions::Expression& expression) const {
    if (expression.isTrue()) {
        return true;
    }
    return _evaluator_ptr->asBool(expression);
}

// Get the actions that can be taken from the loaded state
template <typename ValueType>
AvailableActions<ValueType> JaniSmcStatesExpansion<ValueType>::getAvailableActions() {
    // Clear the computed actions
    _computed_actions.clear();
    _computed_destinations.clear();
    for (const SyncIdxAndEdges& composite_edge : _system_model.composite_edges) {
        const AutomataAndEdges& automata_and_edges = composite_edge.second;
        if (automata_and_edges.size() == 1U) {
            // Composite edge with 1 automaton -> This is a non-syncing edge
            const AutomatonAndEdges& automaton_and_edges = automata_and_edges.front();
            const uint64_t& automaton_id = automaton_and_edges.first;
            const uint64_t& location_id = _current_state.getLocationData().at(automaton_id);
            const LocationsAndEdges& location_to_edges = automaton_and_edges.second;
            const bool is_silent_action = !composite_edge.first;
            // Check if the current location matches the edge
            const auto& matching_edges_it = location_to_edges.find(location_id);
            if (location_to_edges.end() != matching_edges_it) {
                // The action might be valid: check the guards conditions
                bool valid_action_found = false;
                for (const auto& [aut_edge_idx, aut_edge_val] : matching_edges_it->second) {
                    // TODO: Move this to the constructor operations
                    STORM_LOG_THROW(
                        !aut_edge_val.get().hasRate(), storm::exceptions::InvalidModelException, "Found edge with rate: this is invalid");
                    if (!_evaluator_ptr->asBool(aut_edge_val.get().getGuard())) {
                        continue;
                    }
                    // Jani allows multiple viable edges with the same name (internal nondeterminism) -> we only pick the 1st viable one!
                    // The only exception is silent_actions: in this case, all options must be provided as own actions
                    if (!valid_action_found || is_silent_action) {
                        AutomataAndEdges action_edges = {{automaton_id, {{location_id, {{aut_edge_idx, aut_edge_val}}}}}};
                        if (_reward.information.hasStateActionRewards()) {
                            _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
                            const auto& edge_assignments = aut_edge_val.get().getAssignments();
                            if (!edge_assignments.empty()) {
                                storm::generator::TransientVariableValuation<ValueType> transient_vars_values;
                                for (int64_t current_level = aut_edge_val.get().getAssignments().getLowestLevel(true);
                                     current_level <= aut_edge_val.get().getAssignments().getHighestLevel(true); current_level++) {
                                    transient_vars_values.clear();
                                    executeTransientAssignments(
                                        transient_vars_values, aut_edge_val.get().getAssignments().getTransientAssignments(current_level));
                                    transient_vars_values.setInEvaluator(*_evaluator_ptr, _additional_checks);
                                }
                            }
                        }
                        _computed_actions.emplace_back(getActionReward(), std::move(action_edges));
                        _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
                        valid_action_found = true;
                    } else {
                        STORM_LOG_WARN("Multiple edges satisfying a condition were found: only the 1st one will be used.");
                        break;
                    }
                }
            }
        } else {
            // If the element has more than one set of edges, we need to perform a synchronization.
            STORM_LOG_THROW(
                composite_edge.first, storm::exceptions::InvalidModelException, "Silent action for synchronized edges not allowed.");
            // If the element has more than one set of edges, we need to perform a synchronization.
            STORM_LOG_THROW(
                automata_and_edges.size() > 1U, storm::exceptions::InvalidModelException, "At least one automaton must execute an edge.");
            // Prepare a data-structure for the extracted actions
            AutomataAndEdges action_edges;
            action_edges.reserve(automata_and_edges.size());
            // Required for computing the transient variable information afterwards
            int64_t lowest_assignment_level = std::numeric_limits<int64_t>::max();
            int64_t highest_assignment_level = std::numeric_limits<int64_t>::min();
            for (const auto& [automaton_id, location_to_edges] : composite_edge.second) {
                // Prepare the entry in the action_edges vector
                action_edges.emplace_back(automaton_id, LocationsAndEdges());
                const uint64_t& location_id = _current_state.getLocationData().at(automaton_id);
                const auto& matching_edges_it = location_to_edges.find(location_id);
                if (location_to_edges.end() != matching_edges_it) {
                    for (const auto& edge_descr : matching_edges_it->second) {
                        const storm::jani::Edge& single_edge = edge_descr.second.get();
                        if (!_evaluator_ptr->asBool(single_edge.getGuard())) {
                            continue;
                        }
                        if (action_edges.back().second.empty()) {
                            if (!single_edge.getAssignments().empty()) {
                                lowest_assignment_level =
                                    std::min(single_edge.getAssignments().getLowestLevel(true), lowest_assignment_level);
                                highest_assignment_level =
                                    std::max(single_edge.getAssignments().getHighestLevel(true), highest_assignment_level);
                            }
                            action_edges.back().second.insert({location_id, {edge_descr}});
                            // Do not break here, to print the warn once in case it is required
                        } else {
                            STORM_LOG_WARN("Multiple edges satisfying a condition were found: only the 1st one will be used.");
                            break;
                        }
                    }
                }
                if (action_edges.back().second.empty()) {
                    break;
                }
            }
            if (!action_edges.back().second.empty()) {
                // Generate the combination of edges to execute for this single action, and add to results
                // Keep in mind that we are providing only one combination per action!
                STORM_LOG_THROW(
                    checkGlobalVariableWrittenOnce(action_edges), storm::exceptions::InvalidModelException,
                    "Found multiple automata writing to the same global variable within single action.");
                // Evaluate transient assignments for the edge (required for reward eval)
                if (_reward.information.hasStateActionRewards()) {
                    _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
                    storm::generator::TransientVariableValuation<ValueType> transient_vars_values;
                    for (int64_t current_level = lowest_assignment_level; current_level <= highest_assignment_level; current_level++) {
                        transient_vars_values.clear();
                        for (const auto& [aut_id, loc_and_edges] : action_edges) {
                            const storm::jani::Edge& single_edge = loc_and_edges.begin()->second.front().second.get();
                            if (!single_edge.getAssignments().empty()) {
                                executeTransientAssignments(
                                    transient_vars_values, single_edge.getAssignments().getTransientAssignments(current_level));
                            }
                        }
                        transient_vars_values.setInEvaluator(*_evaluator_ptr, _additional_checks);
                    }
                }
                _computed_actions.emplace_back(getActionReward(), std::move(action_edges));
                _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
            }
        }
    }
    // Return the extracted actions
    AvailableActions<ValueType> available_actions;
    available_actions.reserve(_computed_actions.size());
    for (uint64_t idx = 0U; idx < _computed_actions.size(); idx++) {
        available_actions.emplace_back(idx, _computed_actions[idx].action_reward);
    }
    return available_actions;
}

// Get the destinations that can be reached from the loaded state by taking a specific action
template <typename ValueType>
typename JaniSmcStatesExpansion<ValueType>::ActionDestinations JaniSmcStatesExpansion<ValueType>::getDestinationsFromAction(
    const JaniSmcStatesExpansion<ValueType>::ActionId action_id) {
    STORM_LOG_THROW(
        !_computed_actions.empty(), storm::exceptions::IllegalFunctionCallException,
        "Trying to compute actions destinations before computing the available actions");
    _computed_destinations.clear();
    _computed_destinations.related_action = action_id;
    const ActionDescription& chosen_action = _computed_actions.at(action_id);
    ActionDestinations possible_destinations = {};
    if (chosen_action.action_edges.size() == 1U) {
        // We have to expand a non-synchronizing edge
        ValueType probability_sum = storm::utility::zero<ValueType>();
        const uint64_t automaton_id = chosen_action.action_edges.front().first;
        const storm::jani::Edge& automaton_edge = chosen_action.action_edges.front().second.begin()->second.front().second.get();
        for (const storm::jani::EdgeDestination& dest : automaton_edge.getDestinations()) {
            const ValueType dest_prob = _evaluator_ptr->asRational(dest.getProbability());
            if (dest_prob == storm::utility::zero<ValueType>()) {
                continue;
            }
            possible_destinations.emplace_back(_computed_destinations.destinations.size(), dest_prob);
            std::vector<AutomatonAndDestination> single_destination = {{automaton_id, dest}};
            _computed_destinations.destinations.emplace_back(std::move(single_destination));
            probability_sum += dest_prob;
        }
        STORM_LOG_THROW(
            _comparator.isOne(probability_sum), storm::exceptions::InvalidModelException,
            "The sum of probabilities in an action is not one.");
    } else {
        // We have to expand a synchronizing edge, generating all possible combinations
        // Make a vector with the automata ids and an iterator pointing at its first edge destination
        std::vector<std::pair<uint64_t, std::vector<storm::jani::EdgeDestination>::const_iterator>> aut_dest_its(
            chosen_action.action_edges.size());
        std::transform(
            chosen_action.action_edges.begin(), chosen_action.action_edges.end(), aut_dest_its.begin(),
            [](const AutomatonAndEdges& automaton_and_edges) {
                const storm::jani::Edge& edge = automaton_and_edges.second.begin()->second.front().second.get();
                return std::make_pair(automaton_and_edges.first, edge.getDestinations().begin());
            });
        ValueType probability_sum = storm::utility::zero<ValueType>();
        // Generate the distribution of possible outcomes
        while (true) {
            std::vector<AutomatonAndDestination> destination_composition;
            destination_composition.reserve(chosen_action.action_edges.size());
            ValueType destination_composition_prob = storm::utility::one<ValueType>();
            std::transform(
                aut_dest_its.begin(), aut_dest_its.end(), std::back_inserter(destination_composition),
                [this, &destination_composition_prob](const auto& dest_vect_it) {
                    const storm::jani::EdgeDestination& destination = *dest_vect_it.second;
                    destination_composition_prob *= _evaluator_ptr->asRational(destination.getProbability());
                    return std::make_pair(dest_vect_it.first, std::ref(destination));
                });
            if (destination_composition_prob > storm::utility::zero<ValueType>()) {
                possible_destinations.emplace_back(_computed_destinations.destinations.size(), destination_composition_prob);
                _computed_destinations.destinations.emplace_back(std::move(destination_composition));
                probability_sum += destination_composition_prob;
            }
            // Advance to the next combination (if any)
            size_t aut_it_to_advance = 0U;
            while (aut_it_to_advance < aut_dest_its.size()) {
                auto& destination_it = aut_dest_its[aut_it_to_advance].second;
                const auto& destinations =
                    chosen_action.action_edges[aut_it_to_advance].second.begin()->second.front().second.get().getDestinations();
                destination_it++;
                if (destinations.end() != destination_it) {
                    break;
                }
                destination_it = destinations.begin();
                aut_it_to_advance++;
            }
            if (aut_it_to_advance >= aut_dest_its.size()) {
                break;
            }
        }
        STORM_LOG_THROW(
            _comparator.isOne(probability_sum), storm::exceptions::InvalidModelException,
            "The sum of probabilities in an action is not one.");
    }
    return possible_destinations;
}

// Advance the automaton to the next state taking a specific action's destination
template <typename ValueType>
std::pair<std::reference_wrapper<const state_properties::StateVariableData<ValueType>>, ValueType>
JaniSmcStatesExpansion<ValueType>::setNextState(
    const JaniSmcStatesExpansion<ValueType>::ActionId action_id, const JaniSmcStatesExpansion<ValueType>::DestinationId destination_id) {
    STORM_LOG_THROW(
        !_computed_destinations.destinations.empty(), storm::exceptions::IllegalFunctionCallException,
        "Trying to set the next  state before computing the possible destinations.");
    STORM_LOG_THROW(
        action_id == _computed_destinations.related_action, storm::exceptions::IllegalFunctionCallException,
        "The requested action and the one related to the computed destinations are not matching.");
    const std::vector<AutomatonAndDestination>& selected_destination = _computed_destinations.destinations.at(destination_id);
    const AutomataAndEdges& selected_edge = _computed_actions.at(action_id).action_edges;
    STORM_LOG_THROW(!selected_destination.empty(), storm::exceptions::UnexpectedException, "Vector of automata to step must be non-empty.");
    if (selected_destination.size() == 1U) {
        expandNonSynchronizingEdge(selected_edge.front(), selected_destination.front());
    } else {
        expandSynchronizingEdge(selected_edge, selected_destination);
    }
    const ValueType transition_reward = getTransitionReward();
    _computed_actions.clear();
    _computed_destinations.clear();
    return {_current_state, transition_reward};
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::loadReward(const std::optional<std::string>& reward_name) {
    if (reward_name) {
        const std::string& reward_str = *reward_name;
        _reward.expression = _jani_model.getRewardModelExpression(reward_str);
        storm::jani::RewardModelInformation info(_jani_model, _reward.expression);
        _reward.information = storm::builder::RewardModelInformation(
            reward_str, info.hasStateRewards(), info.hasActionRewards(), info.hasTransitionRewards());
    }
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::computeAutomatonPluginAssociation(
    const storm::jani::Automaton& automaton, const uint64_t automaton_idx) {
    const std::string& automaton_name = automaton.getName();
    for (uint64_t plugin_idx = 0U; plugin_idx < _external_plugins_desc.get().size(); plugin_idx++) {
        const auto& plugin_desc = _external_plugins_desc.get()[plugin_idx];
        if (plugin_desc.getAutomatonName() == automaton_name) {
            const uint64_t action_id = plugin_desc.getActionId();
            const std::string& action_name = plugin_desc.getActionName();
            // Ensure the automaton is plugin-ready: only one edge and one location, with no assignments
            STORM_LOG_THROW(
                automaton.hasEdgeLabeledWithActionIndex(action_id), storm::exceptions::InvalidModelException,
                "Automaton " + automaton_name + " has no action named " + action_name + ".");
            for (const storm::jani::Edge& single_edge : automaton.getEdges()) {
                if (single_edge.getActionIndex() == action_id) {
                    for (const storm::jani::EdgeDestination& single_dest : single_edge.getDestinations()) {
                        STORM_LOG_THROW(
                            single_dest.getOrderedAssignments().empty(), storm::exceptions::InvalidModelException,
                            "The action from " + automaton_name + " called " + action_name +
                                " is associated to a plugin, and its edges shall not have assignments.");
                    }
                }
            }
            const bool new_insertion =
                _system_model.automaton_and_action_to_plugin_ids.emplace(std::make_pair(automaton_idx, action_id), plugin_idx).second;
            STORM_LOG_THROW(
                new_insertion, storm::exceptions::InvalidModelException,
                "The action from " + automaton_name + " called " + action_name + " is associated to multiple plugins.");
        }
    }
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::generateSynchInformation() {
    // Make sure the automata to plugin map is empty
    _system_model.automaton_and_action_to_plugin_ids.clear();
    // In case there is only one automaton in the model
    if (_jani_model.getSystemComposition().isAutomatonComposition()) {
        const uint64_t automaton_index = 0U;
        const storm::jani::Automaton& automaton =
            _jani_model.getAutomaton(_jani_model.getSystemComposition().asAutomatonComposition().getAutomatonName());
        _system_model.parallel_automata.push_back(automaton);
        computeAutomatonPluginAssociation(automaton, automaton_index);

        LocationsAndEdges locations_and_edges;
        uint64_t edge_index = 0U;
        for (const storm::jani::Edge& edge : automaton.getEdges()) {
            locations_and_edges[edge.getSourceLocationIndex()].emplace_back(edge_index, edge);
            ++edge_index;
        }
        // Put the only automaton in the system model to the _system_model.edges vector
        AutomataAndEdges automata_and_edges = {{automaton_index, std::move(locations_and_edges)}};
        _system_model.composite_edges.emplace_back(std::nullopt, std::move(automata_and_edges));
    } else {
        STORM_LOG_THROW(
            _jani_model.getSystemComposition().isParallelComposition(), storm::exceptions::WrongFormatException,
            "Expected parallel composition.");
        const storm::jani::ParallelComposition& parallel_composition = _jani_model.getSystemComposition().asParallelComposition();

        uint64_t automaton_index = 0U;
        // Process the silent actions first
        for (const auto& composition : parallel_composition.getSubcompositions()) {
            STORM_LOG_THROW(
                composition->isAutomatonComposition(), storm::exceptions::WrongFormatException, "Expected flat parallel composition.");
            const auto& automaton_composition = composition->asAutomatonComposition();
            STORM_LOG_THROW(
                automaton_composition.getInputEnabledActions().empty(), storm::exceptions::NotSupportedException,
                "Input-enabled actions are not supported right now.");

            const storm::jani::Automaton& automaton = _jani_model.getAutomaton(automaton_composition.getAutomatonName());
            _system_model.parallel_automata.push_back(automaton);
            computeAutomatonPluginAssociation(automaton, automaton_index);

            // Add edges with silent action.
            LocationsAndEdges locations_and_edges;
            uint64_t edge_index = 0;
            for (const storm::jani::Edge& edge : automaton.getEdges()) {
                if (edge.hasSilentAction()) {
                    locations_and_edges[edge.getSourceLocationIndex()].emplace_back(edge_index, edge);
                }
                ++edge_index;
            }

            if (!locations_and_edges.empty()) {
                AutomataAndEdges automata_and_edges = {{automaton_index, std::move(locations_and_edges)}};
                _system_model.composite_edges.emplace_back(std::nullopt, std::move(automata_and_edges));
            }
            ++automaton_index;
        }

        for (const auto& synched_actions : parallel_composition.getSynchronizationVectors()) {
            const std::string& composed_action_name = synched_actions.getOutput();
            uint64_t composed_action_index = _jani_model.getActionIndex(composed_action_name);

            AutomataAndEdges automata_and_edges;
            bool at_least_one_edge = true;
            uint64_t automaton_index = 0;
            for (const std::string& action_name : synched_actions.getInput()) {
                if (!storm::jani::SynchronizationVector::isNoActionInput(action_name)) {
                    LocationsAndEdges locations_and_edges;
                    uint64_t action_index = _jani_model.getActionIndex(action_name);
                    uint64_t edge_index = 0;
                    for (const storm::jani::Edge& edge : _system_model.parallel_automata[automaton_index].get().getEdges()) {
                        if (edge.getActionIndex() == action_index) {
                            locations_and_edges[edge.getSourceLocationIndex()].emplace_back(edge_index, edge);
                        }
                        ++edge_index;
                    }
                    if (locations_and_edges.empty()) {
                        at_least_one_edge = false;
                        break;
                    }
                    automata_and_edges.emplace_back(automaton_index, std::move(locations_and_edges));
                }
                ++automaton_index;
            }

            STORM_LOG_THROW(at_least_one_edge, storm::exceptions::InvalidModelException, "Found automaton with no location.");
            if (at_least_one_edge) {
                _system_model.composite_edges.emplace_back(composed_action_index, std::move(automata_and_edges));
            }
        }
    }

    STORM_LOG_TRACE("Number of synchronizations: " << _system_model.composite_edges.size() << ".");
}

template <typename ValueType>
storm::generator::TransientVariableValuation<ValueType> JaniSmcStatesExpansion<ValueType>::evaluateTransientVariablesAtLocations() const {
    storm::generator::TransientVariableValuation<ValueType> transient_variables;
    _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
    for (uint64_t automaton_idx = 0U; automaton_idx < _system_model.parallel_automata.size(); automaton_idx++) {
        const uint64_t location_id = _current_state.getLocationData().at(automaton_idx);
        const storm::jani::Automaton& jani_automaton = _system_model.parallel_automata[automaton_idx].get();
        const storm::jani::Location& jani_location = jani_automaton.getLocation(location_id);
        STORM_LOG_THROW(
            !jani_location.getAssignments().hasMultipleLevels(true), storm::exceptions::InvalidModelException,
            "Indexed assignments at locations are not supported in the JANI standard.");
        executeTransientAssignments(transient_variables, jani_location.getAssignments().getTransientAssignments());
    }
    return transient_variables;
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::expandNonSynchronizingEdge(
    const AutomatonAndEdges& selected_edge, const AutomatonAndDestination& selected_destination) {
    state_properties::StateVariableData<ValueType> next_state(_current_state);
    const uint64_t automaton_id = selected_edge.first;
    const storm::jani::Edge& edge = selected_edge.second.begin()->second.front().second.get();
    const storm::jani::EdgeDestination& destination = selected_destination.second.get();
    const bool has_transient_assignments = destination.hasTransientAssignment();
    const int64_t lowest_assignment_level = edge.getLowestAssignmentLevel();
    const int64_t highest_assignment_level = edge.getHighestAssignmentLevel();
    _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
    storm::generator::TransientVariableValuation<ValueType> transient_vars_values;
    // Enforce the 1st assignment on being done, regardless of the lowest_assignment_level value
    next_state.setLocation(automaton_id, destination.getLocationIndex());
    executeNonTransientDestinationAssignments(next_state, automaton_id, edge.getActionIndex(), destination, lowest_assignment_level);
    if (has_transient_assignments) {
        executeTransientDestinationAssignments(transient_vars_values, automaton_id, destination, lowest_assignment_level);
        transient_vars_values.setInEvaluator(*_evaluator_ptr, _additional_checks);
    }
    _variable_information.setInEvaluator(*_evaluator_ptr, next_state);
    // Increase current_level separately, to ensure it is below highest_assignment_level first
    for (int64_t current_level = lowest_assignment_level; current_level < highest_assignment_level;) {
        ++current_level;
        executeNonTransientDestinationAssignments(next_state, automaton_id, edge.getActionIndex(), destination, current_level);
        if (has_transient_assignments) {
            executeTransientDestinationAssignments(transient_vars_values, automaton_id, destination, current_level);
            transient_vars_values.setInEvaluator(*_evaluator_ptr, _additional_checks);
        }
        _variable_information.setInEvaluator(*_evaluator_ptr, next_state);
    }
    updateCurrentState(next_state);
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::expandSynchronizingEdge(
    const AutomataAndEdges& selected_action, const std::vector<AutomatonAndDestination>& selected_destinations) {
    state_properties::StateVariableData<ValueType> next_state = _current_state;
    // Prepare required info
    int64_t lowest_assignment_level = std::numeric_limits<int64_t>::max();
    int64_t highest_assignment_level = std::numeric_limits<int64_t>::min();
    std::unordered_set<uint64_t> automaton_with_transient_assign;
    for (size_t vect_idx = 0U; vect_idx < selected_action.size(); vect_idx++) {
        STORM_LOG_THROW(
            selected_action[vect_idx].first == selected_destinations[vect_idx].first, storm::exceptions::UnexpectedException,
            "The automata idxs in the edges and destinations is not matching. This is a bug!");
        const storm::jani::Edge& automaton_edge = selected_action[vect_idx].second.begin()->second.front().second.get();
        const auto& [automaton_id, automaton_dest_ref] = selected_destinations[vect_idx];
        lowest_assignment_level = std::min(lowest_assignment_level, automaton_edge.getLowestAssignmentLevel());
        highest_assignment_level = std::max(highest_assignment_level, automaton_edge.getHighestAssignmentLevel());
        if (automaton_dest_ref.get().hasTransientAssignment()) {
            automaton_with_transient_assign.emplace(automaton_id);
        }
    }
    // Advance the state
    _transient_variable_information.setDefaultValuesInEvaluator(*_evaluator_ptr);
    storm::generator::TransientVariableValuation<ValueType> transient_vars_values;
    for (size_t vect_idx = 0U; vect_idx < selected_action.size(); vect_idx++) {
        const uint64_t automaton_id = selected_destinations[vect_idx].first;
        const storm::jani::EdgeDestination& automaton_destination = selected_destinations[vect_idx].second.get();
        const uint64_t action_id = selected_action[vect_idx].second.begin()->second.front().second.get().getActionIndex();
        // Update the location of all automata
        next_state.setLocation(automaton_id, automaton_destination.getLocationIndex());
        // We have to make sure that all automata update the state at least once, even if no assignment indexes are found
        executeNonTransientDestinationAssignments(next_state, automaton_id, action_id, automaton_destination, lowest_assignment_level);
        if (automaton_with_transient_assign.contains(automaton_id)) {
            executeTransientDestinationAssignments(transient_vars_values, automaton_id, automaton_destination, lowest_assignment_level);
            transient_vars_values.setInEvaluator(*_evaluator_ptr, _additional_checks);
        }
    }
    _variable_information.setInEvaluator(*_evaluator_ptr, next_state);
    // Increase current_level separately, to ensure it is below highest_assignment_level first
    for (int64_t current_level = lowest_assignment_level; current_level < highest_assignment_level;) {
        ++current_level;
        for (size_t vect_idx = 0U; vect_idx < selected_action.size(); vect_idx++) {
            const auto& [automaton_id, automaton_dest_ref] = selected_destinations[vect_idx];
            const storm::jani::Edge& automaton_edge = selected_action[vect_idx].second.begin()->second.front().second.get();
            if (current_level >= automaton_edge.getLowestAssignmentLevel() && current_level <= automaton_edge.getHighestAssignmentLevel()) {
                executeNonTransientDestinationAssignments(
                    next_state, automaton_id, automaton_edge.getActionIndex(), automaton_dest_ref.get(), current_level);
                if (automaton_with_transient_assign.contains(automaton_id)) {
                    executeTransientDestinationAssignments(transient_vars_values, automaton_id, automaton_dest_ref.get(), current_level);
                    transient_vars_values.setInEvaluator(*_evaluator_ptr, _additional_checks);
                }
            }
        }
        _variable_information.setInEvaluator(*_evaluator_ptr, next_state);
    }
    updateCurrentState(next_state);
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::executeTransientAssignments(
    storm::generator::TransientVariableValuation<ValueType>& transient_vars,
    const storm::jani::detail::ConstAssignments& transient_assignments) const {
    auto assignments_it = transient_assignments.begin();
    const auto assignments_ite = transient_assignments.end();
    // We expect the assignments to be ordered as follows: booleans, integers and finally rationals
    // Bools processing
    auto bool_info_it = _transient_variable_information.booleanVariableInformation.begin();
    while (assignments_it != assignments_ite && assignments_it->getExpressionVariable().hasBooleanType()) {
        const auto assigned_var = assignments_it->getExpressionVariable();
        while (bool_info_it->variable != assigned_var) {
            bool_info_it++;
        }
        transient_vars.booleanValues.emplace_back(&(*bool_info_it), _evaluator_ptr->asBool(assignments_it->getAssignedExpression()));
        bool_info_it++;
        assignments_it++;
    }
    // Integers processing
    auto int_info_it = _transient_variable_information.integerVariableInformation.begin();
    while (assignments_it != assignments_ite && assignments_it->getExpressionVariable().hasIntegerType()) {
        const auto assigned_var = assignments_it->getExpressionVariable();
        while (int_info_it->variable != assigned_var) {
            int_info_it++;
        }
        int64_t assigned_value = _evaluator_ptr->asInt(assignments_it->getAssignedExpression());
        // TODO: Check integer bounds
        transient_vars.integerValues.emplace_back(&(*int_info_it), assigned_value);
        int_info_it++;
        assignments_it++;
    }
    // Rationals processing
    auto real_info_it = _transient_variable_information.rationalVariableInformation.begin();
    while (assignments_it != assignments_ite && assignments_it->getExpressionVariable().hasRationalType()) {
        const auto assigned_var = assignments_it->getExpressionVariable();
        while (real_info_it->variable != assigned_var) {
            real_info_it++;
        }
        // TODO: Check for bounds
        transient_vars.rationalValues.emplace_back(&(*real_info_it), _evaluator_ptr->asRational(assignments_it->getAssignedExpression()));
        real_info_it++;
        assignments_it++;
    }
    // Arrays are not expected here
    // Make sure we carried out all assignments
    STORM_LOG_THROW(assignments_it == assignments_ite, storm::exceptions::UnexpectedException, "Unhandled transient assignments left.");
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::executeTransientDestinationAssignments(
    storm::generator::TransientVariableValuation<ValueType>& transient_vars, const uint64_t automaton_id,
    const storm::jani::EdgeDestination& destination, int64_t assignment_level) {
    transient_vars.clear();
    const auto& all_assignments = destination.getOrderedAssignments().getTransientAssignments(assignment_level);
    executeTransientAssignments(transient_vars, all_assignments);
}

template <typename ValueType>
void JaniSmcStatesExpansion<ValueType>::executeNonTransientDestinationAssignments(
    state_properties::StateVariableData<ValueType>& state, const uint64_t automaton_id, const uint64_t action_id,
    const storm::jani::EdgeDestination& destination, int64_t assignment_level) {
    // Check if this automaton relates to a plugin
    const auto plugin_id_it = _system_model.automaton_and_action_to_plugin_ids.find({automaton_id, action_id});
    if (plugin_id_it != _system_model.automaton_and_action_to_plugin_ids.end()) {
        const uint64_t plugin_id = plugin_id_it->second;
        smc_verifiable_plugins::SmcPluginBase& plugin_instance = *_loaded_plugin_ptrs.at(plugin_id);
        const auto& plugin_description = _external_plugins_desc.get().at(plugin_id);
        smc_verifiable_plugins::SmcPluginBase::DataExchange input_data = {};
        for (const auto& input_argument : plugin_description.getInputVariablesMap()) {
            const auto& expression_type = input_argument.second.getType();
            if (expression_type.isBooleanType()) {
                input_data.emplace(input_argument.first, _evaluator_ptr->asBool(input_argument.second));
            } else if (expression_type.isIntegerType()) {
                input_data.emplace(input_argument.first, _evaluator_ptr->asInt(input_argument.second));
            } else if (expression_type.isRationalType()) {
                input_data.emplace(input_argument.first, _evaluator_ptr->asRational(input_argument.second));
            } else {
                STORM_LOG_THROW(
                    false, storm::exceptions::InvalidModelException,
                    "Cannot determine what data type to use for the input " + input_argument.first + " of " +
                        plugin_description.getAutomatonName() + " plugin.");
            }
        }
        const auto plugin_output_values = plugin_instance.nextStep(input_data);
        auto plugin_output_desc_it = plugin_description.getOutputVariablesMap().begin();
        const auto plugin_output_desc_ite = plugin_description.getOutputVariablesMap().end();
        auto bool_vars_it = _variable_information.booleanVariables().begin();
        auto int_vars_it = _variable_information.integerVariables().begin();
        auto real_vars_it = _variable_information.realVariables().begin();
        while (plugin_output_desc_it != plugin_output_desc_ite && plugin_output_desc_it->second.hasBooleanType()) {
            while (plugin_output_desc_it->second != bool_vars_it->variable) {
                bool_vars_it++;
            }
            size_t bool_idx = std::distance(_variable_information.booleanVariables().begin(), bool_vars_it);
            state.setBool(bool_idx, std::get<bool>(plugin_output_values.at(plugin_output_desc_it->first)));
            plugin_output_desc_it++;
            bool_vars_it++;
        }
        while (plugin_output_desc_it != plugin_output_desc_ite && plugin_output_desc_it->second.hasIntegerType()) {
            while (plugin_output_desc_it->second != int_vars_it->variable) {
                int_vars_it++;
            }
            size_t int_idx = std::distance(_variable_information.integerVariables().begin(), int_vars_it);
            state.setInt(int_idx, std::get<int64_t>(plugin_output_values.at(plugin_output_desc_it->first)));
            plugin_output_desc_it++;
            int_vars_it++;
        }
        while (plugin_output_desc_it != plugin_output_desc_ite && plugin_output_desc_it->second.hasRationalType()) {
            while (plugin_output_desc_it->second != real_vars_it->variable) {
                real_vars_it++;
            }
            size_t real_idx = std::distance(_variable_information.realVariables().begin(), real_vars_it);
            state.setReal(real_idx, std::get<ValueType>(plugin_output_values.at(plugin_output_desc_it->first)));
            plugin_output_desc_it++;
            real_vars_it++;
        }
        STORM_LOG_THROW(
            plugin_output_desc_it == plugin_output_desc_ite, storm::exceptions::OutOfRangeException,
            "Cannot find plugin output target variable " + plugin_output_desc_it->second.getName() + " in the model.");
    } else {
        // This orders the assignments to be booleans first, then integers and then arrays. Variables are ordered as in the var_info
        const auto& all_assignments = destination.getOrderedAssignments().getNonTransientAssignments(assignment_level);
        auto assignments_it = all_assignments.begin();
        const auto assignments_ite = all_assignments.end();
        // Booleans first
        auto bool_info_it = _variable_information.booleanVariables().begin();
        while (assignments_it != assignments_ite && assignments_it->getExpressionVariable().hasBooleanType()) {
            const auto& assigned_var = assignments_it->getExpressionVariable();
            while (bool_info_it->variable != assigned_var) {
                bool_info_it++;
            }
            const size_t bool_idx = std::distance(_variable_information.booleanVariables().begin(), bool_info_it);
            state.setBool(bool_idx, _evaluator_ptr->asBool(assignments_it->getAssignedExpression()));
            bool_info_it++;
            assignments_it++;
        }
        // Integers afterwards
        auto int_info_it = _variable_information.integerVariables().begin();
        while (assignments_it != assignments_ite && assignments_it->getExpressionVariable().hasIntegerType()) {
            const auto& assigned_var = assignments_it->getExpressionVariable();
            while (int_info_it->variable != assigned_var) {
                int_info_it++;
            }
            const size_t int_idx = std::distance(_variable_information.integerVariables().begin(), int_info_it);
            int64_t assigned_value = _evaluator_ptr->asInt(assignments_it->getAssignedExpression());
            // TODO: Check integer bounds
            state.setInt(int_idx, assigned_value);
            int_info_it++;
            assignments_it++;
        }
        // Finally, real values
        auto real_info_it = _variable_information.realVariables().begin();
        while (assignments_it != assignments_ite && assignments_it->getExpressionVariable().hasRationalType()) {
            const auto& assigned_var = assignments_it->getExpressionVariable();
            while (real_info_it->variable != assigned_var) {
                real_info_it++;
            }
            const size_t real_idx = std::distance(_variable_information.realVariables().begin(), real_info_it);
            ValueType assigned_value = _evaluator_ptr->asRational(assignments_it->getAssignedExpression());
            // TODO: Check rational bounds
            state.setReal(real_idx, assigned_value);
            real_info_it++;
            assignments_it++;
        }
        // Arrays are not expected here
        // Make sure we carried out all assignments
        STORM_LOG_THROW(assignments_it == assignments_ite, storm::exceptions::UnexpectedException, "Not all assignments were executed.");
    }
}

template class JaniSmcStatesExpansion<double>;
}  // namespace smc_storm::state_generation
