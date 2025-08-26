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

#include "model_checker/smc_plugin_instance.hpp"
#include "state_generation/available_actions.hpp"
#include "state_generation/jani_smc_model_build.hpp"
#include "state_properties/state_variable_information.hpp"
#include <optional>
#include <random>
#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <stdint.h>
#include <storm/builder/RewardModelInformation.h>
#include <storm/generator/CompressedState.h>
#include <storm/generator/TransientVariableInformation.h>
#include <storm/generator/VariableInformation.h>
#include <storm/storage/expressions/ExpressionEvaluator.h>
#include <storm/storage/jani/Model.h>
#include <storm/utility/constants.h>
#include <storm/utility/ConstantsComparator.h>
#include <unordered_map>

namespace smc_storm::state_generation {
template <typename ValueType>
class JaniSmcStatesExpansion {
  private:
    struct PairHash {
        uint64_t operator()(const std::pair<uint64_t, uint64_t>& p) const {
            // This relies on the assumption that we will have IDs below 2^32
            return (p.first << 32) | p.second;
        }
    };
    // Some types that are used in this class
    using StateType = uint32_t;
    using ActionId = uint64_t;
    using DestinationId = uint64_t;
    using DestinationWithProbability = std::pair<DestinationId, ValueType>;
    using ActionDestinations = std::vector<DestinationWithProbability>;
    using StateAndReward = std::pair<state_properties::StateVariableData<ValueType>, ValueType>;
    using AutomatonAndDestination = std::pair<uint64_t, std::reference_wrapper<const storm::jani::EdgeDestination>>;
    using AutomatonAndEdge = std::pair<uint64_t, std::reference_wrapper<const storm::jani::Edge>>;

  public:
    /*!
     * @brief Initiate the state generator class
     * @param jani_model The model to generate the states from
     * @param reward_name The reward to generate from the model (zero or one reward)
     * @param external_plugins Dummy placeholder for the external plugins. To be done!!!
     */
    JaniSmcStatesExpansion(
        const storm::jani::Model& jani_model, const std::optional<std::string>& reward_name,
        const std::vector<model_checker::SmcPluginInstance>& external_plugins, std::default_random_engine& random_generator);

    ~JaniSmcStatesExpansion() = default;

    /*!
     * @brief Getter for the variable information: needed to introspect the CompressedState
     * @return A const reference to the Variable information
     */
    inline const state_properties::StateVariableInformation<ValueType>& getVariableInformation() const {
        return _variable_information;
    }

    /*!
     * @brief Getter for the current state of the expanded model
     * @return A const reference to the current state
     */
    inline const state_properties::StateVariableData<ValueType>& getCurrentState() const {
        return _current_state;
    }

    /*!
     * @brief Generate the initial state and set it as the current one.
     * @return The generated initial state
     */
    const state_properties::StateVariableData<ValueType>& setInitialState();

    /*!
     * @brief Advance the automaton to the next state selecting a specific action and related destination
     * @param action_id The ID of the action to execute. Use getAvailableActions to get the Action IDs available from the current state.
     * @param destination_id The ID of the destination to take. Use getDestinationsFromAction to compute the available ones.
     * @return The new current_state, obtained by moving to the requested destination, together with the destination reward.
     */
    std::pair<std::reference_wrapper<const state_properties::StateVariableData<ValueType>>, ValueType> setNextState(
        const ActionId action_id, const DestinationId destination_id);

    /*!
     * @brief Check if the current state satisfies a specific expression
     * @param expression the expression to evaluate on the current state
     * @return true if the expression holds, false otherwise
     */
    bool satisfies(const storm::expressions::Expression& expression) const;

    /*!
     * @brief Get the reward associated to the current state (TODO)
     * @return A reward value
     */
    inline const ValueType getStateReward() const {
        return _reward.current_state_reward;
    }

    /*!
     * @brief Get the reward associated to the current state (TODO)
     * @return A reward value
     */
    inline const ValueType getActionReward() const {
        if (_reward.information.hasStateActionRewards()) {
            return _evaluator_ptr->asRational(_reward.expression);
        }
        return storm::utility::zero<ValueType>();
    }

    inline const ValueType getTransitionReward() const {
        if (_reward.information.hasTransitionRewards()) {
            return _evaluator_ptr->asRational(_reward.expression);
        }
        return storm::utility::zero<ValueType>();
    }

    /*!
     * @brief Get the actions that can be taken from the current state
     * @return A vector of pairs indicating the action ID and the associated reward
     */
    AvailableActions<ValueType> getAvailableActions();

    /*!
     * @brief Get the destinations that can be reached from the current state by taking a specific action
     * @return A vector of pairs indicating the destination ID and the associated probability of being selected
     */
    ActionDestinations getDestinationsFromAction(const ActionId action_id);

  private:
    /*!
     * @brief Update the current state value and the evaluator
     */
    void updateCurrentState(const state_properties::StateVariableData<ValueType>& state);

    /*!
     * @brief Ensures that the model doesn't contain unsupported features
     */
    void checkSupportedFeatures() const;

    /*!
     * @brief Extract the reward information from the provided model
     * @param reward_name The name of the reward to evaluate
     */
    void loadReward(const std::optional<std::string>& reward_name);

    /*!
     * @brief Check if there are undefined constants. Raise an error in case
     */
    void checkUndefinedConstants() const;

    /*!
     * @brief Generate the plugin objects instances from the provided plugins
     */
    void loadPlugins();

    /*!
     * @brief Check that each automaton's edge writes to a different set of global variables
     * @param edge_set The edges to check
     * @return true if the edge set is valid, false otherwise
     */
    bool checkGlobalVariableWrittenOnce(const std::vector<AutomatonAndEdge>& edge_set) const;

    /*!
     * @brief Evaluate the transient variables values at the current state's locations
     * @return A data structure holding the value of the evaluated transient variables
     */
    storm::generator::TransientVariableValuation<ValueType> evaluateTransientVariablesAtLocations() const;

    /*!
     * @brief Given a non-synchronizing edge, step forward from the current state and store the result in the _current_state variable
     * @param selected_edge The edge and related automaton used for advancing the state
     * @param selected_destination The edge's destination and related automaton used for advancing the state
     * @return Whether the expansion worked as expected or there was an error
     */
    bool expandNonSynchronizingEdge(const AutomatonAndEdge& selected_edge, const AutomatonAndDestination& selected_destination);

    /*!
     * @brief Given a synchronizing edge, step forward from the current state and store the result in the _current_state variable
     * @param selected_action The selected action, defined as a set of automata and related edges
     * @param selected_destinations The set of edge-destinations and related automata used for advancing the state
     * @return Whether the expansion worked as expected or there was an error
     */
    bool expandSynchronizingEdge(
        const std::vector<AutomatonAndEdge>& selected_action, const std::vector<AutomatonAndDestination>& selected_destinations);

    /*!
     * @brief Performs all the requested assignments and stores the result in the provided data structure
     * @param transient_vars Data structure holding the results from the assignments
     * @param transient_assignments Vector of assignments that need to be carried out
     */
    void executeTransientAssignments(
        storm::generator::TransientVariableValuation<ValueType>& transient_vars,
        const storm::jani::detail::ConstAssignments& transient_assignments) const;

    /**
     * @brief Update the transient variables value at a specific destination and assignment step
     * @param transient_vars The transient variable valuation that holds the current values of the transient variables
     * @param automaton_id The identifier of the automaton being processed
     * @param destination The destination edge in the JANI model that specifies the target state and assignments
     * @param assignment_level The level of assignment to be performed
     */
    void executeTransientDestinationAssignments(
        storm::generator::TransientVariableValuation<ValueType>& transient_vars, const uint64_t automaton_id,
        const storm::jani::EdgeDestination& destination, int64_t assignment_level);

    /*!
     * @brief Update the permanent variables value at a specific destination and assignment step
     * @param state The current state of the model, where to update the permanent variables values
     * @param automaton_id The identifier of the automaton being processed
     * @param action_id The action_id associated to the edge being processed
     * @param destination The destination edge in the JANI model that specifies the target state and assignments
     * @param assignment_level The level of assignment to be performed
     * @return whether the transition was performed successfully
     */
    bool executeNonTransientDestinationAssignments(
        state_properties::StateVariableData<ValueType>& state, const uint64_t automaton_id, const uint64_t action_id,
        const storm::jani::EdgeDestination& destination, int64_t assignment_level);

    /*!
     * @brief Update the provided state with the information generated from running a plugin.
     * @param state The state to update
     * @param plugin_out_data The data generated by resetting or getting the next step from a plugin
     * @param plugin_description The information about the plugin
     */
    void assignPluginResultToState(
        state_properties::StateVariableData<ValueType>& state, const smc_verifiable_plugins::DataExchange& plugin_out_data,
        const model_checker::SmcPluginInstance& plugin_description);

    std::reference_wrapper<std::default_random_engine> _random_generator;
    // A copy of the model we are going to explore
    storm::jani::Model _jani_model;
    state_properties::StateVariableInformation<ValueType> _variable_information;
    storm::generator::TransientVariableInformation<ValueType> _transient_variable_information;

    std::unique_ptr<const JaniSmcModelBuild> _model_build_ptr;

    // This bool is meant to enable additional checks on the model's expanded states. Disabled for now
    const bool _additional_checks = false;

    state_properties::StateVariableData<ValueType> _initial_state;
    state_properties::StateVariableData<ValueType> _current_state;
    // Dummy state returned in case of errors.
    const state_properties::StateVariableData<ValueType> _empty_state = state_properties::StateVariableData<ValueType>();

    std::reference_wrapper<const std::vector<model_checker::SmcPluginInstance>> _external_plugins_desc;
    std::vector<std::unique_ptr<smc_verifiable_plugins::SmcPluginBase>> _loaded_plugin_ptrs;

    // Description of a single action
    struct ActionDescription {
        const ValueType action_reward;
        const std::vector<AutomatonAndEdge> action_edges;
        inline ActionDescription(const ValueType& reward, const std::vector<AutomatonAndEdge>& automata_edges)
            : action_reward{reward}, action_edges{automata_edges} {}
    };
    std::vector<ActionDescription> _computed_actions;

    struct ComputedDestinations {
        // The action ID related to the computed destinations
        ActionId related_action;
        std::vector<std::vector<AutomatonAndDestination>> destinations;
        // Small helper to clear the content
        inline void clear() {
            destinations.clear();
        }
    };
    ComputedDestinations _computed_destinations;

    // The reward expression to evaluate
    struct {
        storm::builder::RewardModelInformation information;
        storm::expressions::Expression expression;
        ValueType current_state_reward;
    } _reward;
    // An expression evaluator
    std::unique_ptr<storm::expressions::ExpressionEvaluator<ValueType>> _evaluator_ptr;
    /// A comparator used to compare constants.
    storm::utility::ConstantsComparator<ValueType> _comparator;
};
}  // namespace smc_storm::state_generation
