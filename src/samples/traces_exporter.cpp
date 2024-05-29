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

#include <storm/io/file.h>
#include <storm/utility/macros.h>
#include <storm/exceptions/NotSupportedException.h>

#include "samples/traces_exporter.h"

namespace smc_storm::samples
{
TracesExporter::TracesExporter(const std::filesystem::path& path_to_file, const storm::generator::VariableInformation& var_info)
: _var_info(var_info),
  _trace_counter(0) {
    STORM_LOG_THROW(!path_to_file.empty(), storm::exceptions::NotSupportedException, "The path to the file is empty!");
    const auto absolute_path = std::filesystem::absolute(path_to_file);
    STORM_LOG_THROW(std::filesystem::exists(absolute_path.parent_path()), storm::exceptions::NotSupportedException, "The parent directory does not exist!");
    STORM_LOG_THROW(!std::filesystem::exists(absolute_path), storm::exceptions::NotSupportedException, "The file already exists!");
    _n_variables = var_info.locationVariables.size() + var_info.booleanVariables.size() + var_info.integerVariables.size();
    STORM_LOG_THROW(_n_variables > 0U, storm::exceptions::NotSupportedException, "The provided VariableInformation is empty!");
    storm::utility::openFile(absolute_path.string(), _file);
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
}

TracesExporter::~TracesExporter() {
    storm::utility::closeFile(_file);
}

void TracesExporter::addCurrentTraceResult(const TraceInformation& result) {
    _file << _trace_counter << ";;" << toString(result.outcome) << ";;";
    _trace_counter++;
    // Add one to account for the empty column between locations and remaining variables
    for (size_t i = 0; i < _n_variables + 1U; i++) {
        _file << ";";
    }
    _file << "\n\n";
}

void TracesExporter::addNextState(const storm::generator::CompressedState& state) {
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
        const uint_fast64_t int_value = state.getAsInt(int_info.bitOffset, int_info.bitWidth) + int_info.lowerBound;
        _file << int_value << ";";
    }
    _file << "\n";
}
} // namespace smc_storm::samples
