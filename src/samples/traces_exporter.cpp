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

#include <filesystem>

#include <storm/exceptions/NotSupportedException.h>
#include <storm/io/file.h>
#include <storm/utility/macros.h>

#include "samples/traces_exporter.hpp"

namespace smc_storm::samples {

// TracesExporter methods
TracesExporter::TracesExporter(const std::filesystem::path& path_to_file) : _trace_counter(0) {
    STORM_LOG_THROW(!path_to_file.empty(), storm::exceptions::NotSupportedException, "The path to the file is empty!");
    const auto absolute_path = std::filesystem::absolute(path_to_file);
    STORM_LOG_THROW(
        std::filesystem::exists(absolute_path.parent_path()), storm::exceptions::NotSupportedException,
        "The parent directory does not exist!");
    STORM_LOG_THROW(!std::filesystem::exists(absolute_path), storm::exceptions::NotSupportedException, "The file already exists!");
    storm::io::openFile(absolute_path.string(), _file);
}

TracesExporter::~TracesExporter() {
    storm::io::closeFile(_file);
}

void TracesExporter::addCurrentTraceResult(const TraceInformation& result) {
    if (!_export_only_failures || result.outcome == smc_storm::samples::TraceResult::NOT_VERIFIED) {
        // In case we are not exporting only failures, this vector will have no effect
        writeCachedStates();
        _file << _trace_counter << ";;" << toString(result.outcome) << ";;";
        _trace_counter++;
        // Add one to account for the empty column between locations and remaining variables
        for (size_t i = 0; i < _n_variables + 1U; i++) {
            _file << ";";
        }
        _file << "\n\n";
    }
    clearTraces();
}

// CompressedStateTraceExporter methods
CompressedStateTraceExporter::CompressedStateTraceExporter(
    const std::filesystem::path& path_to_file, const storm::generator::VariableInformation& var_info)
    : TracesExporter(path_to_file), _var_info{var_info} {
    _n_variables = var_info.locationVariables.size() + var_info.booleanVariables.size() + var_info.integerVariables.size();
    STORM_LOG_THROW(_n_variables > 0U, storm::exceptions::NotSupportedException, "The provided VariableInformation is empty!");
    // Write the header
    _file << "Trace number;;Result;;";
    // Locations
    for (const auto& loc : var_info.locationVariables) {
        _file << loc.variable.getName() << ";";
    }
    _file << ";";
    // Booleans
    for (const auto& bool_var : var_info.booleanVariables) {
        _file << bool_var.variable.getName() << ";";
    }
    // Integers
    for (const auto& int_var : var_info.integerVariables) {
        _file << int_var.variable.getName() << ";";
    }
    // No real variables, since they can only be transient
    _file << "\n\n";
    _current_trace_states.clear();
}

void CompressedStateTraceExporter::addNextState(const storm::generator::CompressedState& state) {
    if (_export_only_failures) {
        _current_trace_states.emplace_back(state);
    } else {
        writeNextState(state);
    }
}

void CompressedStateTraceExporter::writeCachedStates() {
    for (const auto& state : _current_trace_states) {
        writeNextState(state);
    }
}

void CompressedStateTraceExporter::writeNextState(const storm::generator::CompressedState& state) {
    _file << _trace_counter << ";;;;";
    // Locations
    for (const auto& loc_info : _var_info.locationVariables) {
        const uint_fast64_t& loc_value = (loc_info.bitWidth == 0U ? 0U : state.getAsInt(loc_info.bitOffset, loc_info.bitWidth));
        _file << loc_value << ";";
    }
    _file << ";";
    // Booleans
    for (const auto& bool_info : _var_info.booleanVariables) {
        const std::string bool_value = (state.get(bool_info.bitOffset) ? "true" : "false");
        _file << bool_value << ";";
    }
    // Integers
    for (const auto& int_info : _var_info.integerVariables) {
        const int_fast64_t int_value = state.getAsInt(int_info.bitOffset, int_info.bitWidth) + int_info.lowerBound;
        _file << int_value << ";";
    }
    _file << "\n";
}

// UncompressedStateTraceExporter
UncompressedStateTraceExporter::UncompressedStateTraceExporter(
    const std::filesystem::path& path_to_file, const state_properties::StateVariableInformation<double>& var_info)
    : TracesExporter(path_to_file), _var_info{var_info} {
    _n_variables = var_info.locationVariables().size() + var_info.booleanVariables().size() + var_info.integerVariables().size() +
                   var_info.realVariables().size();
    STORM_LOG_THROW(_n_variables > 0U, storm::exceptions::NotSupportedException, "The provided VariableInformation is empty!");
    // Write the header
    _file << "Trace number;;Result;;";
    // Locations
    for (const auto& loc : var_info.locationVariables()) {
        _file << loc.variable.getName() << ";";
    }
    _file << ";";
    // Booleans
    for (const auto& bool_var : var_info.booleanVariables()) {
        _file << bool_var.variable.getName() << ";";
    }
    // Integers
    for (const auto& int_var : var_info.integerVariables()) {
        _file << int_var.variable.getName() << ";";
    }
    // Reals
    for (const auto& real_var : var_info.realVariables()) {
        _file << real_var.variable.getName() << ";";
    }
    // No real variables, since they can only be transient
    _file << "\n\n";
    _current_trace_states.clear();
}

void UncompressedStateTraceExporter::addNextState(const state_properties::StateVariableData<double>& state) {
    if (_export_only_failures) {
        _current_trace_states.emplace_back(state);
    } else {
        writeNextState(state);
    }
}

void UncompressedStateTraceExporter::writeCachedStates() {
    for (const auto& state : _current_trace_states) {
        writeNextState(state);
    }
}

void UncompressedStateTraceExporter::writeNextState(const state_properties::StateVariableData<double>& state) {
    _file << _trace_counter << ";;;;";
    // Locations
    for (const auto& loc_value : state.getLocationData()) {
        _file << loc_value << ";";
    }
    _file << ";";
    // Booleans
    for (const auto& bool_value : state.getBoolData()) {
        const std::string bool_str = bool_value ? "true" : "false";
        _file << bool_str << ";";
    }
    // Integers
    for (const auto& int_value : state.getIntData()) {
        _file << int_value << ";";
    }
    // Reals
    for (const auto& real_value : state.getRealData()) {
        _file << real_value << ";";
    }
    _file << "\n";
}

}  // namespace smc_storm::samples
