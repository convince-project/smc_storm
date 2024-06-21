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

#include "samples/trace_information.hpp"
#include <filesystem>
#include <storm/generator/CompressedState.h>
#include <storm/generator/VariableInformation.h>

namespace smc_storm::samples {
/*!
 * @brief An helper class to export the generated traces to a CSV file.
 */
class TracesExporter {
  public:
    /*!
     * @brief Constructor for the TraceExporter helper class
     * @param path_to_file Path to the CSV file to write the traces in
     * @param var_info Reference to the VariableInformation instance related to the loaded model, to expand CompressedStates
     */
    TracesExporter(const std::filesystem::path& path_to_file, const storm::generator::VariableInformation& var_info);
    ~TracesExporter();

    // TODO: Add information about the taken action and the current reward
    /*!
     * @brief Add a new line to the current trace, using the provided state
     * @param state The Compressed state containing the internal variables values.
     */
    void addNextState(const storm::generator::CompressedState& state);
    /*!
     * @brief Write the current trace's verification result and start the next trace
     * @param result The result of the current trace
     */
    void addCurrentTraceResult(const TraceInformation& result);

  private:
    std::ofstream _file;
    const storm::generator::VariableInformation& _var_info;
    size_t _trace_counter;
    size_t _n_variables;
};
}  // namespace smc_storm::samples
