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
#include <iostream>
#include <random>
#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <smc_verifiable_plugins/utils.hpp>
#include <stdexcept>

namespace smc_storm_plugins {
using smc_verifiable_plugins::DataExchange;
/*!
 * @brief A plugin for testing the error handling feature of SMC plugins
 */
class ControlledErrorSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    ControlledErrorSmcPlugin() = default;

    ~ControlledErrorSmcPlugin() = default;

    std::string getPluginName() const override {
        return "controlled_error_smc_plugin";
    }

  private:
    /*!
     * @brief Load the Dice configuration: it consists of only one int telling the n. of faces
     * @param config The configuration to load: ints: [random_seed, n_faces], bool: [verbose (optional, false by default)]
     */
    void processInitParameters([[maybe_unused]] const DataExchange& config) override {}

    /*!
     * @brief Reset the plugin to the initial state (nothing to do, this plugin is stateless)
     * @return The initial state of the output variables
     */
    std::optional<DataExchange> processReset() override {
        _n_resets++;
        _n_steps = 0u;
        if (_n_resets <= 10u) {
            return std::nullopt;
        }
        return DataExchange({{"result", true}});
    }

    /*!
     * @brief Advances the plugin by one step by throwing a dice!
     * @param input_data The data used to control the evolution of the plugin. Empty here.
     * @return The outcome of the step increase, to be assigned to the model's state (a dice value).
     */
    std::optional<DataExchange> processInputParameters([[maybe_unused]] const DataExchange& input_data) override {
        _n_steps++;
        const size_t outcome_selector = _n_resets % 6;
        bool out_result = false;
        if (outcome_selector <= 1u) {
            out_result = false;
        } else if (outcome_selector <= 3U) {
            out_result = true;
        } else if (outcome_selector <= 4U) {
            // Error only if second step reached
            if (_n_steps > 1u) {
                return std::nullopt;
            } else {
                out_result = true;
            }
        } else {
            // Error at 1st step
            return std::nullopt;
        }
        return std::make_optional<DataExchange>({{"result", out_result}});
    }

    size_t _n_resets = 0u;
    size_t _n_steps = 0u;
};

}  // namespace smc_storm_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(smc_storm_plugins::ControlledErrorSmcPlugin);
}  // namespace smc_verifiable_plugins
