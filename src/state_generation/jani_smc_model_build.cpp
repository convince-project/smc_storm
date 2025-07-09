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

#include <storm/exceptions/InvalidModelException.h>
#include <storm/storage/jani/AutomatonComposition.h>
#include <storm/storage/jani/ParallelComposition.h>

#include "state_generation/jani_smc_model_build.hpp"

namespace smc_storm::state_generation {
template <typename ValueType>
JaniSmcModelBuild<ValueType>::JaniSmcModelBuild(
    const storm::jani::Model& jani_model, const std::vector<model_checker::SmcPluginInstance>& external_plugins) {
    const auto& jani_composition = jani_model.getSystemComposition();
    if (jani_composition.isAutomatonComposition()) {
        // We can treat it as a single automaton
        const uint64_t automaton_idx = 0u;
        _jani_automata.reserve(1u);
        _automata_actions.reserve(1u);
        _composite_edges.reserve(1u);
        STORM_LOG_THROW(
            jani_model.getAutomata().size() == 1u, storm::exceptions::InvalidModelException, "Only one automaton expected here.");
        const storm::jani::Automaton& single_automaton = jani_model.getAutomata().front();
        _jani_automata.emplace_back(std::ref(single_automaton));
        _automata_actions.emplace_back(AutomatonActionsSet({AutomatonAction(single_automaton.getNumberOfLocations(), LocationEdges())}));
        for (const storm::jani::Edge& aut_edge : single_automaton.getEdges()) {
            // Access the 0th automaton and its 0th action and append the edge to the related location
            _automata_actions.at(0u).at(0u).at(aut_edge.getSourceLocationIndex()).emplace_back(std::ref(aut_edge));
        }
        _composite_edges.emplace_back(SILENT_ACTION_ID, std::vector<AutomatonToActionId>({AutomatonToActionId(automaton_idx, 0U)}));
    } else {
        STORM_LOG_THROW(jani_composition.isParallelComposition(), storm::exceptions::InvalidModelException, "Unexpected model type found.");
        // This is the typical configuration, with the network of synched automata
        const storm::jani::ParallelComposition& parallel_composition = jani_composition.asParallelComposition();
        const uint64_t n_sub_automata = parallel_composition.getNumberOfSubcompositions();
        _jani_automata.reserve(n_sub_automata);
        _automata_actions.reserve(n_sub_automata);
        for (const auto& sub_component : parallel_composition.getSubcompositions()) {
            STORM_LOG_THROW(
                sub_component->isAutomatonComposition(), storm::exceptions::InvalidModelException, "Expected flat parallel composition.");
            const storm::jani::Automaton& sub_automaton =
                jani_model.getAutomaton(sub_component->asAutomatonComposition().getAutomatonName());
            const uint64_t sub_automaton_idx = _jani_automata.size();
            // Add an entry for the current automaton
            _jani_automata.emplace_back(std::ref(sub_automaton));
            _automata_actions.emplace_back();
            // Compute the silent actions first (only related to this automaton)
            AutomatonAction sub_aut_silent_action(sub_automaton.getNumberOfLocations(), LocationEdges());
            bool has_silent_edges = false;
            for (const storm::jani::Edge& sub_aut_edge : sub_automaton.getEdges()) {
                if (sub_aut_edge.hasSilentAction()) {
                    sub_aut_silent_action.at(sub_aut_edge.getSourceLocationIndex()).emplace_back(std::ref(sub_aut_edge));
                    has_silent_edges = true;
                }
            }
            if (has_silent_edges) {
                _automata_actions.at(sub_automaton_idx).emplace_back(std::move(sub_aut_silent_action));
                STORM_LOG_THROW(
                    _automata_actions.at(sub_automaton_idx).size() == 1u, storm::exceptions::UnexpectedException,
                    "Unexpected size of automaton actions.");
                _composite_edges.emplace_back(
                    SILENT_ACTION_ID, std::vector<AutomatonToActionId>({AutomatonToActionId(sub_automaton_idx, 0U)}));
            }
        }
        // Then compute the synched actions
        for (const auto& synched_action : parallel_composition.getSynchronizationVectors()) {
            const std::string& composed_action_name = synched_action.getOutput();
            uint64_t composed_action_index = jani_model.getActionIndex(composed_action_name);
            CompositeEdge composition_instance = {composed_action_index, {}};
            for (uint64_t sub_aut_idx = 0u; sub_aut_idx < n_sub_automata; sub_aut_idx++) {
                const std::string& sub_action_name = synched_action.getInput(sub_aut_idx);
                if (!storm::jani::SynchronizationVector::isNoActionInput(sub_action_name)) {
                    const uint64_t jani_action_id = jani_model.getActionIndex(sub_action_name);
                    const storm::jani::Automaton sub_automaton = _jani_automata.at(sub_aut_idx).get();
                    AutomatonAction sub_aut_action(sub_automaton.getNumberOfLocations(), LocationEdges());
                    const uint64_t sub_aut_action_id = _automata_actions.at(sub_aut_idx).size();
                    bool at_least_one_edge = false;
                    for (const storm::jani::Edge& edge : sub_automaton.getEdges()) {
                        if (edge.getActionIndex() == jani_action_id) {
                            at_least_one_edge = true;
                            sub_aut_action.at(edge.getSourceLocationIndex()).emplace_back(std::ref(edge));
                        }
                    }
                    STORM_LOG_THROW(at_least_one_edge, storm::exceptions::InvalidModelException, "Found always invalid edge.");
                    _automata_actions.at(sub_aut_idx).emplace_back(std::move(sub_aut_action));
                    composition_instance.second.emplace_back(sub_aut_idx, sub_aut_action_id);
                }
            }
            _composite_edges.emplace_back(std::move(composition_instance));
        }
    }
    computePluginAssociations(external_plugins);
}

template <typename ValueType>
void JaniSmcModelBuild<ValueType>::computePluginAssociations(const std::vector<model_checker::SmcPluginInstance>& external_plugins) {
    _automata_plugins = std::vector<AutomatonEdgesWithPlugin>(_jani_automata.size(), AutomatonEdgesWithPlugin());
    for (uint64_t aut_idx = 0u; aut_idx < _jani_automata.size(); aut_idx++) {
        const storm::jani::Automaton& automaton = _jani_automata.at(aut_idx).get();
        const std::string& automaton_name = automaton.getName();
        for (uint64_t plugin_idx = 0u; plugin_idx < external_plugins.size(); plugin_idx++) {
            const model_checker::SmcPluginInstance& plugin_desc = external_plugins[plugin_idx];
            if (plugin_desc.getAutomatonName() == automaton_name) {
                const uint64_t action_id = plugin_desc.getActionId();
                const std::string& action_name = plugin_desc.getActionName();
                // Ensure the automaton has the required action ID
                STORM_LOG_THROW(
                    automaton.hasEdgeLabeledWithActionIndex(action_id), storm::exceptions::InvalidModelException,
                    "Automaton " + automaton_name + " has no action named " + action_name + ".");
                // Ensure all plugin-related edges have no assignments
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
                _automata_plugins.at(aut_idx).emplace_back(action_id, plugin_idx);
                // TODO: Ensure that each automaton action has only one plugin associated to it
            }
        }
    }
}

template class JaniSmcModelBuild<double>;
}  // namespace smc_storm::state_generation
