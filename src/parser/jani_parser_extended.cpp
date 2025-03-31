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

#include "parser/jani_parser_extended.hpp"
#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/UnexpectedException.h>

namespace smc_storm::parser {
JaniModelPropertiesPlugins JaniParserExtended::parseModelPropertiesAndPlugins(
    const std::filesystem::path& path, const std::vector<std::filesystem::path>& plugin_paths) {
    JaniParserExtended parser(path, plugin_paths);
    return parser.parseModelPropertiesAndPlugins();
}

JaniParserExtended::JaniParserExtended(const std::filesystem::path& path, const std::vector<std::filesystem::path>& plugin_paths)
    : JaniParser<double>(), _plugin_paths{plugin_paths} {
    // Add a first json file to the parent class, for reading model and properties
    readFile(path.string());
    // Prepare a second copy of the json file, for reading the plugins
    _jani_json = storm::json<double>::parse(std::ifstream(path.string()));
}

JaniModelPropertiesPlugins JaniParserExtended::parseModelPropertiesAndPlugins() {
    const auto [model, properties] = this->parseModel();
    std::vector<model_checker::SmcPluginInstance> plugins = {};
    // Plugins
    return std::make_tuple(model, properties, loadPlugins(model));
}

void JaniParserExtended::generateGlobalScope(const storm::jani::Model& jani_model) {
    for (const auto& constant : jani_model.getConstants()) {
        _constants_map.emplace(constant.getName(), &constant);
    }
    for (const auto& variable : jani_model.getGlobalVariables()) {
        _global_vars_map.emplace(variable.getName(), &variable);
    }
}

JaniParserExtended::VariablesMap JaniParserExtended::extractVariablesAutomatonScope(const storm::jani::Automaton& plugin_automaton) const {
    VariablesMap ret_var;
    for (const auto& variable : plugin_automaton.getVariables()) {
        ret_var.emplace(variable.getName(), &variable);
    }
    return ret_var;
}

std::vector<model_checker::SmcPluginInstance> JaniParserExtended::loadPlugins(const storm::jani::Model& jani_model) {
    std::vector<model_checker::SmcPluginInstance> plugins = {};
    const auto& plugins_json_it = _jani_json.find("plugins");
    if (_jani_json.end() != plugins_json_it) {
        generateGlobalScope(jani_model);
        STORM_LOG_THROW(
            !_plugin_paths.get().empty(), storm::exceptions::UnexpectedException, "Using plugins, but no plugin path provided.");
        STORM_LOG_THROW(
            plugins_json_it->is_array(), storm::exceptions::InvalidModelException, "Jani: Found plugin entry, but it is not an array.");
        const auto& jani_manager = jani_model.getManager();
        plugins.reserve(plugins_json_it->size());
        for (const auto& plugin_entry : *plugins_json_it) {
            const std::string plugin_id = plugin_entry.at("plugin_id").get<std::string>();
            const std::string automaton_id = plugin_entry.at("automaton_id").get<std::string>();
            const std::string action_name = plugin_entry.at("action_name").get<std::string>();
            const uint64_t action_id = jani_model.getActionIndex(action_name);
            model_checker::SmcPluginInstance plugin_instance(_plugin_paths.get(), plugin_id, automaton_id, action_name, action_id);
            if (plugin_entry.contains("init")) {
                for (const auto& init_conf : plugin_entry.at("init")) {
                    const std::string data_name = init_conf.at("name").get<std::string>();
                    const std::string data_type = init_conf.at("type").get<std::string>();
                    // TODO: Try to load them using the Expression loader from JaniParser, making sure they are constants...
                    const auto& data_json_value = init_conf.at("value");
                    STORM_LOG_THROW(
                        data_json_value.is_boolean() || data_json_value.is_number(), storm::exceptions::InvalidModelException,
                        "Invalid init params for plugin " + plugin_id + ": expected a constant value.");
                    if (data_type == "int") {
                        plugin_instance.appendInitData(data_name, data_json_value.get<int64_t>());
                    } else if (data_type == "real") {
                        plugin_instance.appendInitData(data_name, data_json_value.get<double>());
                    } else if (data_type == "bool") {
                        plugin_instance.appendInitData(data_name, data_json_value.get<bool>());
                    } else {
                        STORM_LOG_THROW(false, storm::exceptions::InvalidModelException, "Invalid init params for plugin " + plugin_id);
                    }
                }
            }
            // Append input config
            if (plugin_entry.contains("input")) {
                for (const auto& init_conf : plugin_entry.at("input")) {
                    const std::string data_name = init_conf.at("name").get<std::string>();
                    const std::string data_type = init_conf.at("type").get<std::string>();
                    const VariablesMap automaton_vars_map = extractVariablesAutomatonScope(jani_model.getAutomaton(automaton_id));
                    Scope current_scope = Scope(automaton_id, &_constants_map, &_global_vars_map, nullptr, &automaton_vars_map);
                    const auto& data_value = parseExpression(init_conf.at("value"), current_scope);
                    // TODO: Use data_type to validate the retrieved expression
                    plugin_instance.appendInputData(data_name, data_value);
                }
            }
            // Append output config
            if (plugin_entry.contains("output")) {
                for (const auto& output_conf : plugin_entry.at("output")) {
                    const std::string ref_name = output_conf.at("ref").get<std::string>();
                    const storm::expressions::Variable& ref_var = jani_manager.getVariable(ref_name);
                    plugin_instance.appendOutputData(ref_var, output_conf.at("value").get<std::string>());
                }
            }
            plugin_instance.sortOutputData();
            // Add to loaded plugins
            plugins.emplace_back(std::move(plugin_instance));
        }
    }
    return plugins;
}
}  // namespace smc_storm::parser
