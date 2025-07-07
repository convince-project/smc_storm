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

#include "model_checker/model_and_settings_validation.hpp"
#include "model_checker/statistical_model_checker.hpp"
#include "parser/parsers.hpp"
#include "settings/cmd_settings.hpp"
#include "utils/storm_utilities.hpp"
#include <storm/exceptions/InvalidSettingsException.h>

int main(int argc, char* argv[]) {
    smc_storm::utils::stormSetUp();
    // Get the Cmd Arguments
    smc_storm::settings::CmdSettings cmd_settings;
    cmd_settings.parse(argc, argv);
    const auto user_settings = cmd_settings.getSettings();
    const auto model_and_properties = smc_storm::parser::parseModelAndProperties(user_settings);
    const auto mc_settings = smc_storm::settings::SmcSettings(user_settings);
    // Perform model checking
    std::cout << "Welcome to SMC Storm\n";
    std::cout << "Checking model: " << user_settings.model_file << std::endl << std::flush;
    // Validate user inputs
    STORM_LOG_THROW(
        smc_storm::model_checker::areModelAndSettingsValid(model_and_properties, mc_settings), storm::exceptions::InvalidSettingsException,
        "Provided settings are invalid. Try adjusting them and run smc_storm again.");
    for (const auto& property : model_and_properties.properties) {
        smc_storm::model_checker::StatisticalModelChecker smc(
            model_and_properties.model, property, mc_settings, model_and_properties.plugins);
        smc.printProperty();
        smc.check();
    }
    return 0;
}
