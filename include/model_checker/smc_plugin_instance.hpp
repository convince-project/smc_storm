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

#include <filesystem>
#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <storm/storage/expressions/Expression.h>
#include <storm/storage/expressions/Variable.h>
#include <vector>

namespace smc_storm::model_checker {
/*!
 * @brief Object to keep all information regarding a plugin instantiation in one place.
 */
class SmcPluginInstance {
  public:
    // A pair associating a plugin variable name to an expression variable
    using PluginAndModelVariable = std::pair<std::string, storm::expressions::Variable>;
    // A vector of pairs of plugin and model variables, used to describe the plugin output interface
    using PluginAndModelVariableVector = std::vector<std::pair<std::string, storm::expressions::Variable>>;
    // A mapping from the input arg name and the associated expression
    using PluginToModelExpressionMap = std::unordered_map<std::string, storm::expressions::Expression>;

    SmcPluginInstance() = delete;

    /*!
     * @brief Constructs a new SmcPluginInstance object.
     *
     * @param available_paths A vector of filesystem paths where plugins are available.
     * @param plugin_name The name of the plugin to be used.
     * @param automaton_id The ID of the automaton associated with this plugin instance.
     * @param action_name The name of the automaton's action associated with this plugin instance (for debug reasons).
     * @param action_id The ID of the automaton's action associated with this plugin instance.
     */
    SmcPluginInstance(
        const std::vector<std::filesystem::path>& available_paths, const std::string& plugin_name, const std::string& automaton_id,
        const std::string& action_name, const uint64_t action_id);

    /*!
     * @brief Get the name of the automaton associated to this plugin instance description.
     * @return The string describing the automaton name.
     */
    inline const std::string& getAutomatonName() const {
        return _automaton_id;
    }

    /*!
     * @brief Get the index associated to the action name provided in the plugin configuration.
     * @return The index related to the provided action, resulting from the JANI model.
     */
    inline const uint64_t getActionId() const {
        return _action_id;
    }

    /*!
     * @brief Get the action name provided in the plugin configuration.
     * @return The action name provided in the plugin configuration.
     */
    inline const std::string& getActionName() const {
        return _action_name;
    }

    /*!
     * @brief Appends initialization data to the plugin instance.
     *
     * @tparam T The type of the value to be appended.
     * @param ref The name of the config parameter in the plugin.
     * @param value The value to assign to the configuration parameter.
     */
    template <typename T>
    void appendInitData(const std::string& ref, const T& value);

    /*!
     * @brief Get the initialization data for the plugin.
     * @return The DataExchange object containing all initialization data for the plugin.
     */
    inline const smc_verifiable_plugins::SmcPluginBase::DataExchange& getInitData() const {
        return _init_data;
    }

    /*!
     * @brief Get the mapping between the plugin input variables and the associated model variables.
     * @return A constant reference to map relating the plugin's input data to model's variables.
     */
    inline const PluginToModelExpressionMap& getInputVariablesMap() const {
        return _input_data_to_expression;
    }

    /*!
     * @brief Get the mapping between the plugin output variables and the associated model variables.
     * @return A constant reference to map relating the plugin's output data to model's variables.
     */
    inline const PluginAndModelVariableVector& getOutputVariablesMap() const {
        return _output_data_to_variable;
    }

    /*!
     * @brief Appends input data to the plugin instance.
     *
     * @tparam T The type of the value to be appended.
     * @param ref A reference string to identify the input data in the plugin.
     * @param value The value to assign to the input data.
     */
    void appendInputData(const std::string& ref, const storm::expressions::Expression& value);

    /*!
     * @brief Appends output data to the plugin instance.
     *
     * @param ref A reference string, associated to the JANI variable storing the output value.
     * @param value The name of the output value that will be provided by the plugin.
     */
    void appendOutputData(const storm::expressions::Variable& ref, const std::string& value);

    /*!
     * @brief Sort the entries in the output data vectors based on the variable index from the Jani Model
     */
    void sortOutputData();

    /*!
     * @brief Generate a plugin instance related to the loaded plugin information.
     */
    std::unique_ptr<smc_verifiable_plugins::SmcPluginBase> generatePluginInstance() const;

  private:
    std::filesystem::path _plugin_path;
    const std::string _plugin_id;
    const std::string _automaton_id;
    const std::string _action_name;
    const uint64_t _action_id;
    smc_verifiable_plugins::SmcPluginBase::DataExchange _init_data = {};

    // Map between the input data of the plugin and the JANI expression
    PluginToModelExpressionMap _input_data_to_expression = {};

    // Map between the output data of the plugin and the JANI variable
    PluginAndModelVariableVector _output_data_to_variable = {};
    std::unordered_set<std::string> _output_keys = {};
};
}  // namespace smc_storm::model_checker
