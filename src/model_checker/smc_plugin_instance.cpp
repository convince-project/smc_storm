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
#include "model_checker/smc_plugin_instance.hpp"
#include <smc_verifiable_plugins/utils.hpp>
#include <storm/exceptions/InvalidModelException.h>
#include <storm/utility/macros.h>

namespace smc_storm::model_checker {
SmcPluginInstance::SmcPluginInstance(
    const std::vector<std::filesystem::path>& available_paths, const std::string& plugin_name, const std::string& automaton_id,
    const std::string& action_name, const uint64_t action_id)
    : _plugin_id{plugin_name}, _automaton_id{automaton_id}, _action_name{action_name}, _action_id{action_id} {
    for (const auto& folder_path : available_paths) {
        const auto test_path = smc_verifiable_plugins::generatePathToPlugin(folder_path, _plugin_id);
        if (std::filesystem::exists(test_path)) {
            _plugin_path = folder_path;
            break;
        }
    }
    STORM_LOG_THROW(!_plugin_path.empty(), storm::exceptions::InvalidModelException, "Cannot find plugin library among provided paths.");
}

std::unique_ptr<smc_verifiable_plugins::SmcPluginBase> SmcPluginInstance::generatePluginInstance() const {
    return smc_verifiable_plugins::loadPlugin(_plugin_path, _plugin_id);
}

template <typename T>
void SmcPluginInstance::appendInitData(const std::string& name, const T& value) {
    const bool insertion_success = _init_data.emplace(name, value).second;
    STORM_LOG_THROW(
        insertion_success, storm::exceptions::InvalidModelException,
        "Init param " + name + " for plugin " + _plugin_id + " of automaton " + _automaton_id + " provided twice.");
}

void SmcPluginInstance::appendInputData(const std::string& name, const storm::expressions::Expression& value) {
    const bool insertion_success = _input_data_to_expression.emplace(name, value).second;
    STORM_LOG_THROW(
        insertion_success, storm::exceptions::InvalidModelException,
        "Input param " + name + " for plugin " + _plugin_id + " of automaton " + _automaton_id + " provided twice.");
}

void SmcPluginInstance::appendOutputData(const storm::expressions::Variable& ref, const std::string& value) {
    // The "ref" variable is the recipient, that in this case is the JANI variable
    const bool new_key = _output_keys.emplace(value).second;
    STORM_LOG_THROW(
        new_key, storm::exceptions::InvalidModelException,
        "Output param " + value + " for plugin " + _plugin_id + " of automaton " + _automaton_id + " provided twice.");
    _output_data_to_variable.emplace_back(value, ref);
}

void SmcPluginInstance::sortOutputData() {
    // Once here, we loaded all plugin information, so we can get rid of the _output_keys set.
    _output_keys.clear();
    std::sort(
        _output_data_to_variable.begin(), _output_data_to_variable.end(),
        [](const PluginAndModelVariable& left_pair, const PluginAndModelVariable& right_pair) {
            // expression::Variable::operator<() keeps the type into account
            return left_pair.second < right_pair.second;
        });
}

template void SmcPluginInstance::appendInitData<>(const std::string&, const bool&);
template void SmcPluginInstance::appendInitData<>(const std::string&, const int64_t&);
template void SmcPluginInstance::appendInitData<>(const std::string&, const double&);
}  // namespace smc_storm::model_checker
