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
#include "settings/smc_settings.hpp"
#include <argparse/argparse.hpp>

namespace smc_storm::settings {
/*!
 * @brief Class for generating the settings from the command line arguments
 */
class CmdSettings {
  public:
    /*!
     * @brief Constructor: it declares the available cmd arguments
     */
    CmdSettings() : _parser("smc_storm") {
        _parser.add_argument("--model").required().help("Path to the model file.");
        _parser.add_argument("--constants")
            .default_value(_loaded_settings.constants)
            .help("Optional constants for the model / properties.");
        _parser.add_argument("--property-name").required().help("Property to check.");
        _parser.add_argument("--stat-method").default_value(_loaded_settings.stat_method).help("Statistical method to use.");
        _parser.add_argument("--traces-file").default_value(_loaded_settings.traces_file).help("Path to the traces file.");
        _parser.add_argument("--confidence").scan<'g', double>().default_value(_loaded_settings.confidence).help("Confidence level.");
        _parser.add_argument("--epsilon").scan<'g', double>().default_value(_loaded_settings.epsilon).help("Maximum absolute error.");
        _parser.add_argument("--max-trace-length")
            .scan<'i', int>()
            .default_value(_loaded_settings.max_trace_length)
            .help("Maximum number of steps in a single trace (0 -> inf).");
        _parser.add_argument("--max-n-traces")
            .scan<'i', size_t>()
            .default_value(_loaded_settings.max_n_traces)
            .help("Maximum number of traces to generate (for debugging reasons, "
                  "overrides stat_method. 0 -> unset).");
        _parser.add_argument("--n-threads").scan<'i', size_t>().default_value(_loaded_settings.n_threads).help("Number of threads to use.");
        _parser.add_argument("--batch-size")
            .scan<'i', size_t>()
            .default_value(_loaded_settings.batch_size)
            .help("Batch size for the sampling.");
        _parser.add_argument("--show-statistics").default_value(false).implicit_value(true).help("Show statistics after checking.");
    }

    /*!
     * @brief Perform the actual reading from the user input
     * @param argc Number of arguments provided to the program
     * @param argv Array of arguments provided to the program
     */
    void parse(int argc, char* argv[]) {
        try {
            // Read the configs from cmd
            _parser.parse_args(argc, argv);
            // Set the entries in the loaded_settings variable
            _loaded_settings.model = _parser.get<std::string>("--model");
            _loaded_settings.constants = _parser.get<std::string>("--constants");
            _loaded_settings.property_name = _parser.get<std::string>("--property-name");
            _loaded_settings.stat_method = _parser.get<std::string>("--stat-method");
            _loaded_settings.traces_file = _parser.get<std::string>("--traces-file");
            _loaded_settings.confidence = _parser.get<double>("--confidence");
            _loaded_settings.epsilon = _parser.get<double>("--epsilon");
            _loaded_settings.max_trace_length = _parser.get<int>("--max-trace-length");
            _loaded_settings.max_n_traces = _parser.get<size_t>("--max-n-traces");
            _loaded_settings.n_threads = _parser.get<size_t>("--n-threads");
            _loaded_settings.batch_size = _parser.get<size_t>("--batch-size");
            _loaded_settings.show_statistics = _parser.get<bool>("--show-statistics");
            _parsing_done = true;
        } catch (const std::exception& err) {
            std::cerr << err.what() << std::endl;
            std::cerr << _parser;
            std::exit(1);
        }
    }

    SmcSettings getSettings() const {
        if (!_parsing_done) {
            throw std::runtime_error("Settings not parsed yet");
        }
        return _loaded_settings;
    }

  private:
    argparse::ArgumentParser _parser;
    bool _parsing_done = false;
    SmcSettings _loaded_settings;
};
}  // namespace smc_storm::settings