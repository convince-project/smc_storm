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

/*!
 * @brief Base class providing the minimum interface required by a plugin to work with SMC
 */
class DiceRollerSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    DiceRollerSmcPlugin() = default;

    ~DiceRollerSmcPlugin() {
        if (_verbose) {
            std::cout << "Destructing the plugin " << getPluginName() << " instance.\n" << std::flush;
        }
    }

    std::string getPluginName() override {
        return "uniform_random_smc_plugin";
    }

    void setRandomSeed(const uint32_t seed) override {
        _rng.seed(seed);
    }

  private:
    /*!
     * @brief Load the Dice configuration: it consists of only one int telling the n. of faces
     * @param config The configuration to load: ints: [random_seed, n_faces], bool: [verbose (optional, false by default)]
     */
    void processInitParameters(const DataExchange& config) override {
        _verbose = false;
        const auto verbose_conf_it = config.find("verbose");
        const auto faces_conf_it = config.find("faces");
        if (verbose_conf_it != config.end()) {
            _verbose = std::get<bool>(verbose_conf_it->second);
        }
        if (faces_conf_it == config.end()) {
            throw std::invalid_argument("Missing required argument \"faces\".");
        }
        _dice_faces = std::get<int64_t>(faces_conf_it->second);
        if (_dice_faces <= 0) {
            throw std::invalid_argument("Invalid n. of dice faces: " + std::to_string(_dice_faces) + " is not greater than 0.");
        }
        _int_distribution = std::uniform_int_distribution<int32_t>(1, _dice_faces);
        if (_verbose) {
            std::cout << "Initializing plugin " << getPluginName() << " with " << _dice_faces << " faces.\n";
        }
    }

    /*!
     * @brief Reset the plugin to the initial state (nothing to do, this plugin is stateless)
     * @return The initial state of the output variables
     */
    DataExchange processReset() override {
        if (_verbose) {
            std::cout << "Reset plugin " << getPluginName() << std::endl;
        }
        return DataExchange();
    }

    /*!
     * @brief Advances the plugin by one step by throwing a dice!
     * @param input_data The data used to control the evolution of the plugin. Empty here.
     * @return The outcome of the step increase, to be assigned to the model's state (a dice value).
     */
    DataExchange processInputParameters(const DataExchange& input_data) override {
        const int32_t extracted_val = _int_distribution(_rng);
        if (_verbose) {
            std::cout << "Extracted dice value is " << extracted_val << std::endl;
        }
        return {{"result", extracted_val}};
    }

    int64_t _dice_faces = -1;
    bool _verbose = false;
    std::default_random_engine _rng;
    std::uniform_int_distribution<int32_t> _int_distribution;
};

}  // namespace smc_storm_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(smc_storm_plugins::DiceRollerSmcPlugin);
}  // namespace smc_verifiable_plugins
