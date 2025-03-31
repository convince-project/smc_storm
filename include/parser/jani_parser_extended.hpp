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
#include <storm-parsers/parser/JaniParser.h>
#include <storm/adapters/JsonAdapter.h>
#include <storm/storage/jani/Model.h>
#include <storm/storage/jani/Property.h>
#include <tuple>

namespace smc_storm::parser {
using JaniModelPropertiesPlugins =
    std::tuple<storm::jani::Model, std::vector<storm::jani::Property>, std::vector<model_checker::SmcPluginInstance>>;

class JaniParserExtended : public storm::parser::JaniParser<double> {
  public:
    /*!
     * @brief Load the JANI model, the properties and the plugins description from a JANI file.
     * @param path The path to the JANI file containing the model description.
     * @param plugin_paths The paths where to search the required plugins from the JANI file.
     * @return A triple, containing the loaded JANI model instance, its properties and its plugins descriptions.
     */
    static JaniModelPropertiesPlugins parseModelPropertiesAndPlugins(
        const std::filesystem::path& path, const std::vector<std::filesystem::path>& plugin_paths);

    /*!
     * @brief Constructor for the Jani file parser.
     * @param path The path to the Jani file.
     * @param plugin_paths The paths to the plugin libraries, to load at runtime.
     */
    JaniParserExtended(const std::filesystem::path& path, const std::vector<std::filesystem::path>& plugin_paths);

    ~JaniParserExtended() = default;

  protected:
    /*!
     * @brief The method parsing the JANI file from the constructor.
     * @return A triple, containing the loaded JANI model instance, its properties and its plugins descriptions.
     */
    JaniModelPropertiesPlugins parseModelPropertiesAndPlugins();

    /*!
     * @brief Generate the plugin instance descriptions from the loaded JANI model.
     * @param jani_model The JANI model the loaded plugins will refer to.
     * @return A vector of plugin instance descriptions.
     */
    std::vector<model_checker::SmcPluginInstance> loadPlugins(const storm::jani::Model& jani_model);

    /*!
     * @brief Generate the maps of global constants and variables from the loaded JANI model instance.
     * @param jani_model The JANI model to use for extracting the constants and variables.
     */
    void generateGlobalScope(const storm::jani::Model& jani_model);

    /*!
     * @brief Extract the local variables from the automaton.
     * @param plugin_automaton The JANI automaton instance to get the local variables from.
     * @return A VariablesMap instance with the automaton's local variables.
     */
    VariablesMap extractVariablesAutomatonScope(const storm::jani::Automaton& plugin_automaton) const;

    storm::json<double> _jani_json;
    std::reference_wrapper<const std::vector<std::filesystem::path>> _plugin_paths;
    // Keep track of the global scope information (since it uses just pointers)
    ConstantsMap _constants_map;
    VariablesMap _global_vars_map;
};
}  // namespace smc_storm::parser
