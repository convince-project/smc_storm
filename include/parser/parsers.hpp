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
#include <filesystem>

#include <storm/storage/jani/Property.h>
#include <storm/storage/SymbolicModelDescription.h>

#include "settings/smc_settings.hpp"

namespace smc_storm::parser {
/*!
 * @brief A struct that holds a symbolic model and its properties, since Jani contains both in the same file.
 */
struct SymbolicModelAndProperty {
    storm::storage::SymbolicModelDescription model;
    // Use vectors since the storm interface supposes so!
    std::vector<storm::jani::Property> property;
};

/*!
 * @brief Given the ScmSettings, generate the model and properties required by the model checking engine
 * @param settings The settings object containing the path to the model and the properties to verify
 * @return Loaded instance of the requested model and property
 */
SymbolicModelAndProperty parseModelAndProperty(const smc_storm::settings::SmcSettings& settings);

/*!
 * @brief Given the ScmSettings referring to a Jani model, generate the model and properties to be used in the model checking engine
 * @param settings The settings object containing the path to the model and the properties to verify
 * @return Loaded instance of the requested model and property
 */
SymbolicModelAndProperty parseJaniModelAndProperty(const smc_storm::settings::SmcSettings& settings);

std::vector<storm::jani::Property> filterProperties(
    const std::vector<storm::jani::Property>& properties, const std::vector<std::string>& properties_ids,
    const std::map<storm::expressions::Variable, storm::expressions::Expression>& model_constants_map);

}  // namespace smc_storm::parser
