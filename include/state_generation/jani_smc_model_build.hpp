/*
 * Copyright (c) 2025 Robert Bosch GmbH and its subsidiaries
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

class JaniSmcModelBuild {
  public:
    static constexpr uint64_t SILENT_ACTION_ID = std::numeric_limits<uint64_t>::max();
    static constexpr uint64_t NO_PLUGIN_ID = std::numeric_limits<uint64_t>::max();
    // A location could have multiple possible edges
    using LocationEdges = std::vector<std::reference_wrapper<const storm::jani::Edge>>;
    // Each entry in the vector relates to a location (0 to n)
    using AutomatonAction = std::vector<LocationEdges>;
    // All actions belonging to an automaton
    using AutomatonActionsSet = std::vector<AutomatonAction>;
    // A map from the automaton ID to the related action ID to take
    using AutomatonToActionId = std::pair<uint64_t, uint64_t>;
    // A composite edge is a map from action_id to a set of automata with their related action ID
    using CompositeEdge = std::pair<uint64_t, std::vector<AutomatonToActionId>>;
    // A map from the edge id to the plugin id
    using EdgeToPluginId = std::pair<uint64_t, uint64_t>;
    // All edges with plugins in an automaton
    using AutomatonEdgesWithPlugin = std::vector<EdgeToPluginId>;

    /*!
     * @brief Build an internal model from the provided JANI object.
     * @param jani_model The model we are building from.
     * @param external_plugins All plugin instances loaded in the model.
     */
    JaniSmcModelBuild(const storm::jani::Model& jani_model, const std::vector<model_checker::SmcPluginInstance>& external_plugins);

    /*!
     * @brief Get the plugin ID associated to a provided automaton and action ID pair.
     * @param automaton_id The ID of the automaton associated to the plugin.
     * @param action_id The action ID associated to the automaton, that might or not have an associated plugin ID.
     * @return The ID of the plugin to execute. If non found, it returns the NO_PLUGIN_ID constant.
     */
    uint64_t getPluginFromAutomatonAction(const uint64_t automaton_id, const uint64_t action_id) const;

    /*!
     * @brief Get all the automata instances loaded from the provided model.
     * @return An array of references to Jani Automata.
     */
    inline const std::vector<std::reference_wrapper<const storm::jani::Automaton>>& getAutomata() const {
        return _jani_automata;
    }

    /*!
     * @brief Get the amount of automata loaded from the model.
     * @return A count of the loaded automata.
     */
    inline uint64_t getAutomataCount() const {
        return _jani_automata.size();
    }

    /*!
     * @brief Get a specific automaton instance.
     * @param automaton_id The ID of the automaton to retrieve.
     * @return The retrieved automaton.
     */
    inline const storm::jani::Automaton& getAutomaton(const uint64_t automaton_id) const {
        return _jani_automata[automaton_id].get();
    }

    /*!
     * @brief Retrieve all the composite edges (aka the actions defined in the system composition).
     * @return An array of possible (composite) actions.
     */
    inline const std::vector<CompositeEdge>& getCompositeEdges() const {
        return _composite_edges;
    }

    /*!
     * @brief Given an automaton, the selected action and its location, retrieve all edges associated to it.
     * @param automaton_id The ID of the automaton we extract the edge from.
     * @param action_id The action ID associated to the edge.
     * @param location_id The location the edge leaves from.
     * @return An array of edges matching with the provided arguments (could be empty).
     */
    inline const std::vector<std::reference_wrapper<const storm::jani::Edge>>& getAutomatonActionEdgesAtLocation(
        const uint64_t automaton_id, const uint64_t action_id, const uint64_t location_id) const {
        return _automata_actions[automaton_id][action_id][location_id];
    }

  private:
    /*!
     * @brief Generate the association between a single plugin and the related automaton-action pair.
     * @param external_plugins The plugins loaded in the model.
     */
    void computePluginAssociations(const std::vector<model_checker::SmcPluginInstance>& external_plugins);

    // All automata in the system
    std::vector<std::reference_wrapper<const storm::jani::Automaton>> _jani_automata;
    // Vector of Composite Edges, each one with an ID and the related Automaton to ActionID map
    std::vector<CompositeEdge> _composite_edges;
    // Vector of automata, each one having an array of (automaton's) actions
    std::vector<AutomatonActionsSet> _automata_actions;
    // Vector of automata, each one having an array of pairs linking (automaton's) actions to plugin ids
    std::vector<AutomatonEdgesWithPlugin> _automata_plugins;
};
}  // namespace smc_storm::state_generation
