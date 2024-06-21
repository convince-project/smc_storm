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
 * @brief Given a path to a model and the property to verify, generate the model and property to be used in the model checking engine
 * @param path_to_model The path to the model file
 * @param property_name The name of the property to verify
 * @param user_constants Value to assign to constants that might be undefined in the input model
 * @return Loaded instance of the requested model and property
 */
SymbolicModelAndProperty parseModelAndProperty(
    const std::filesystem::path& path_to_model, const std::string& property_name, const std::string& user_constants);

/*!
 * @brief Given a path to a JANI model and the property to verify, generate the model and property to be used in the model checking engine
 * @param path_to_model The path to the model file
 * @param property_name The name of the property to verify
 * @param user_constants Value to assign to constants that might be undefined in the input model
 * @return Loaded instance of the requested model and property
 */
SymbolicModelAndProperty parseJaniModelAndProperty(
    const std::filesystem::path& path_to_model, const std::string& property_name, const std::string& user_constants);

}  // namespace smc_storm::parser
