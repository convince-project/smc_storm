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
#include "settings/user_settings.hpp"
#include <argparse/argparse.hpp>

namespace smc_storm::settings {
constexpr const char* const VERSION = "0.1.1";

/*!
 * @brief Class for generating the settings from the command line arguments
 */
class CmdSettings {
  public:
    /*!
     * @brief Constructor: it declares the available cmd arguments
     */
    CmdSettings();

    /*!
     * @brief Perform the actual reading from the user input
     * @param argc Number of arguments provided to the program
     * @param argv Array of arguments provided to the program
     */
    void parse(int argc, char* argv[]);

    UserSettings getSettings() const;

  private:
    argparse::ArgumentParser _parser;
    bool _parsing_done = false;
    UserSettings _loaded_settings;
};
}  // namespace smc_storm::settings
