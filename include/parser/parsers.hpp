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

#include <storm/storage/SymbolicModelDescription.h>
#include <storm/storage/jani/Property.h>


namespace smc_storm::parser
{
    /*!
     * @brief A struct that holds a symbolic model and its properties, since Jani contains both in the same file.
     */
    struct SymbolicModelAndProperty {
        storm::storage::SymbolicModelDescription model;
        // Use vectors since the storm interface supposes so!
        std::vector<storm::jani::Property> property;
    };

    SymbolicModelAndProperty parseModelAndProperty(std::filesystem::path const& path_to_model, std::string const& property_name, const std::string& user_constants);

    SymbolicModelAndProperty parseJaniModelAndProperty(std::filesystem::path const& path_to_model, std::string const& property_name, const std::string& user_constants);
    
} // namespace smc_storm::parser
