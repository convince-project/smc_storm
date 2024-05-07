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

#include <storm/api/builder.h>
#include <storm-parsers/api/storm-parsers.h>
#include <storm-parsers/parser/PrismParser.h>
#include <storm-parsers/parser/JaniParser.h>
#include <storm/utility/macros.h>
#include <storm/exceptions/NotSupportedException.h>
#include <storm/exceptions/InvalidPropertyException.h>

#include "parser/parsers.hpp"

namespace smc_storm::parser
{
    SymbolicModelAndProperty parseModelAndProperty(const std::filesystem::path& path_to_model, const std::string& property_name, const std::string& user_constants)
    {
        if (path_to_model.extension() == ".jani") {
            return parseJaniModelAndProperty(path_to_model, property_name, user_constants);
        }
        STORM_LOG_THROW(false, storm::exceptions::NotSupportedException, "Only JANI models are supported for now.");
    }

    SymbolicModelAndProperty parseJaniModelAndProperty(const std::filesystem::path& path_to_model, const std::string& property_name, const std::string& user_constants)
    {
        // Load the required model and property
        const auto model_and_formulae = storm::parser::JaniParser<storm::RationalNumber>::parse(path_to_model.string(), true);
        model_and_formulae.first.checkValid();
        const auto model_constants_map = model_and_formulae.first.getConstantsSubstitution();
        // Fill the returned structure
        SymbolicModelAndProperty model_and_property;
        model_and_property.model = storm::storage::SymbolicModelDescription(model_and_formulae.first);
        model_and_property.property.clear();
        model_and_property.property.reserve(1U);
        for (auto const& property : model_and_formulae.second) {
            if (property.getName() == property_name) {
                // Do not forget to substitute Jani constants in the property, too!
                model_and_property.property.emplace_back(property.substitute(model_constants_map));
                break;
            }
        }
        STORM_LOG_THROW(!model_and_property.property.empty(), storm::exceptions::InvalidPropertyException,
            "Property not found in model.");
        // Add the user-defined constants
        const auto user_constants_map = model_and_property.model.parseConstantDefinitions(user_constants);
        model_and_property.model = model_and_property.model.preprocess(user_constants_map);
        model_and_property.property.front() = model_and_property.property.front().substitute(user_constants_map);
        STORM_LOG_THROW(model_and_property.property.front().getUndefinedConstants().empty(), storm::exceptions::InvalidPropertyException, 
            "The property uses undefined constants!!!");
        // Expand the model
        storm::jani::ModelFeatures supported_features = storm::api::getSupportedJaniFeatures(storm::builder::BuilderType::Explicit);
        storm::api::simplifyJaniModel(model_and_property.model.asJaniModel(), model_and_property.property, supported_features);
        return model_and_property;
    }
} // namespace smc_storm::helpers