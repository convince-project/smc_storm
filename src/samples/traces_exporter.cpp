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
TracesExporter::TracesExporter(const std::filesystem::path& path_to_folder, const int thread_id)
    : _trace_counter(0), _traces_folder(std::filesystem::absolute(path_to_folder)), _thread_id(thread_id),
      _tmp_filename_path(_traces_folder / ("_tmp_trace_thread_" + std::to_string(_thread_id) + ".csv")) {
    STORM_LOG_THROW(!path_to_folder.empty(), storm::exceptions::NotSupportedException, "The path to the the traces folder is empty!");
    STORM_LOG_THROW(
        std::filesystem::exists(_traces_folder), storm::exceptions::NotSupportedException,
        "The folder " + _traces_folder.string() + " does not exist!");
}

TracesExporter::~TracesExporter() {
    if (_current_file.is_open()) {
        storm::io::closeFile(_current_file);
    }
}

void TracesExporter::createNewTraceFile() {
    STORM_LOG_THROW(
        !_current_file.is_open(), storm::exceptions::FileIoException, "Trying to open a new file before closing the previous one");
    storm::io::openFile(_tmp_filename_path, _current_file, false, true);
}

void TracesExporter::addCurrentTraceResult(const TraceInformation& result) {
    if (!_export_only_failures || result.outcome == smc_storm::samples::TraceResult::NOT_VERIFIED) {
        _current_file << _trace_counter << ";;" << toString(result.outcome) << ";;";
        // Add one to account for the empty column between locations and remaining variables
        for (size_t i = 0; i < _n_variables + 1U; i++) {
            _current_file << ";";
        }
        _current_file << "\n";
        _trace_counter++;
        storm::io::closeFile(_current_file);
        std::rename(_tmp_filename_path.c_str(), (_traces_folder / generateNewFilename(result.outcome)).c_str());
    } else {
        // Discard the trace
        storm::io::closeFile(_current_file);
        std::remove(_tmp_filename_path.c_str());
    }
}

const std::string TracesExporter::generateNewFilename(const smc_storm::samples::TraceResult& res) {
    std::string suffix;
    if (smc_storm::samples::TraceResult::VERIFIED == res) {
        suffix = "_verified";
    } else if (smc_storm::samples::TraceResult::NOT_VERIFIED == res) {
        suffix = "_not_verified";
    } else {
        suffix = "_unknown";
    }
    std::stringstream stream;
    stream << "trace_" << _thread_id << "_" << std::setw(6) << std::setfill('0') << _trace_counter << suffix;
    return stream.str();
}

// CompressedStateTraceExporter methods
CompressedStateTraceExporter::CompressedStateTraceExporter(
    const std::filesystem::path& path_to_folder, const storm::generator::VariableInformation& var_info, const int thread_id)
    : TracesExporter(path_to_folder, thread_id), _var_info{var_info} {
    _n_variables = var_info.locationVariables.size() + var_info.booleanVariables.size() + var_info.integerVariables.size();
    STORM_LOG_THROW(_n_variables > 0U, storm::exceptions::NotSupportedException, "The provided VariableInformation is empty!");
}

void CompressedStateTraceExporter::startNewTrace() {
    createNewTraceFile();

    // Write the header
    _current_file << "Trace number;;Result;;";
    // Locations
    for (const auto& loc : _var_info.locationVariables) {
        _current_file << loc.variable.getName() << ";";
    }
    _current_file << ";";
    // Booleans
    for (const auto& bool_var : _var_info.booleanVariables) {
        _current_file << bool_var.variable.getName() << ";";
    }
    // Integers
    for (const auto& int_var : _var_info.integerVariables) {
        _current_file << int_var.variable.getName() << ";";
    }
    // No real variables, since they can only be transient
    _current_file << "\n\n";
}

void CompressedStateTraceExporter::addNextState(const storm::generator::CompressedState& state) {
    _current_file << _trace_counter << ";;;;";
    // Locations
    for (const auto& loc_info : _var_info.locationVariables) {
        const uint_fast64_t& loc_value = (loc_info.bitWidth == 0U ? 0U : state.getAsInt(loc_info.bitOffset, loc_info.bitWidth));
        _current_file << loc_value << ";";
    }
    _current_file << ";";
    // Booleans
    for (const auto& bool_info : _var_info.booleanVariables) {
        const std::string bool_value = (state.get(bool_info.bitOffset) ? "true" : "false");
        _current_file << bool_value << ";";
    }
    // Integers
    for (const auto& int_info : _var_info.integerVariables) {
        const int_fast64_t int_value = state.getAsInt(int_info.bitOffset, int_info.bitWidth) + int_info.lowerBound;
        _current_file << int_value << ";";
    }
    _current_file << "\n";
}

// UncompressedStateTraceExporter
UncompressedStateTraceExporter::UncompressedStateTraceExporter(
    const std::filesystem::path& path_to_folder, const state_properties::StateVariableInformation<double>& var_info, const int thread_id)
    : TracesExporter(path_to_folder, thread_id), _var_info{var_info} {
    _n_variables = var_info.locationVariables().size() + var_info.booleanVariables().size() + var_info.integerVariables().size() +
                   var_info.realVariables().size();
    STORM_LOG_THROW(_n_variables > 0U, storm::exceptions::NotSupportedException, "The provided VariableInformation is empty!");
}

void UncompressedStateTraceExporter::startNewTrace() {
    createNewTraceFile();
    // Write the header
    _current_file << "Trace number;;Result;;";
    // Locations
    for (const auto& loc : _var_info.locationVariables()) {
        _current_file << loc.variable.getName() << ";";
    }
    _current_file << ";";
    // Booleans
    for (const auto& bool_var : _var_info.booleanVariables()) {
        _current_file << bool_var.variable.getName() << ";";
    }
    // Integers
    for (const auto& int_var : _var_info.integerVariables()) {
        _current_file << int_var.variable.getName() << ";";
    }
    // Reals
    for (const auto& real_var : _var_info.realVariables()) {
        _current_file << real_var.variable.getName() << ";";
    }
    _current_file << "\n\n";
}

void UncompressedStateTraceExporter::addNextState(const state_properties::StateVariableData<double>& state) {
    _current_file << _trace_counter << ";;;;";
    // Locations
    for (const auto& loc_value : state.getLocationData()) {
        _current_file << loc_value << ";";
    }
    _current_file << ";";
    // Booleans
    for (const auto& bool_value : state.getBoolData()) {
        const std::string bool_str = bool_value ? "true" : "false";
        _current_file << bool_str << ";";
    }
    // Integers
    for (const auto& int_value : state.getIntData()) {
        _current_file << int_value << ";";
    }
    // Reals
    for (const auto& real_value : state.getRealData()) {
        _current_file << real_value << ";";
    }
    _current_file << "\n";
}

}  // namespace smc_storm::samples
