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

#include "utils/storm_utilities.hpp"

#include <storm/settings/SettingsManager.h>
#include <storm/utility/initialize.h>

namespace smc_storm::utils {
void stormSetUp() {
    // Ensure we do this only once
    static bool needs_init = true;
    if (needs_init) {
        // Init loggers
        storm::utility::setUp();
        storm::settings::initializeAll("smc_storm", "smc_storm");
        needs_init = false;
    }
}
}  // namespace smc_storm::utils
