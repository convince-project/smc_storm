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
#pragma once

#include "model_checker/model_and_properties.hpp"
#include "settings/smc_settings.hpp"

namespace smc_storm::model_checker {
/*!
 * @brief Check that the provided model and settings are compatible
 */
bool areModelAndSettingsValid(const ModelAndProperties& model_and_properties, const settings::SmcSettings& settings);
}  // namespace smc_storm::model_checker
