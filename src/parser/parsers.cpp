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

#include <storm-parsers/api/storm-parsers.h>
#include <storm-parsers/parser/JaniParser.h>
#include <storm-parsers/parser/PrismParser.h>
#include <storm/api/builder.h>
#include <storm/api/properties.h>
#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidPropertyException.h>
#include <storm/exceptions/NotSupportedException.h>
#include <storm/utility/macros.h>

#include "parser/parsers.hpp"

namespace smc_storm::parser {
SymbolicModelAndProperty parseModelAndProperty(const smc_storm::settings::UserSettings& settings) {
    // Ensure settings validity
    STORM_LOG_THROW(settings.validModel(), storm::exceptions::InvalidModelException, "Invalid model file provided.");
    STORM_LOG_THROW(settings.validProperties(), storm::exceptions::InvalidPropertyException, "Invalid properties provided.");
    const std::filesystem::path path_to_model(settings.model_file);
    if (path_to_model.extension() == ".jani") {
        return parseJaniModelAndProperty(settings);
    }
    STORM_LOG_THROW(false, storm::exceptions::NotSupportedException, "Only JANI models are supported for now.");
}

SymbolicModelAndProperty parseJaniModelAndProperty(const smc_storm::settings::UserSettings& settings) {
    // Load the required model and property
    const auto model_and_formulae = storm::parser::JaniParser<storm::RationalNumber>::parse(settings.model_file, true);
    model_and_formulae.first.checkValid();
    const auto model_constants_map = model_and_formulae.first.getConstantsSubstitution();
    // Fill the returned structure
    SymbolicModelAndProperty model_and_property;
    model_and_property.model = storm::storage::SymbolicModelDescription(model_and_formulae.first);
    model_and_property.property.clear();
    if (!settings.custom_property.empty()) {
        // TODO
        STORM_LOG_THROW(false, storm::exceptions::NotSupportedException, "Custom properties are not supported yet.");
    } else {
        // Get the list of properties from a comma separated list
        std::vector<std::string> properties_ids = {};
        std::string properties_string = settings.properties_names;
        if (!properties_string.empty()) {
            properties_string.erase(
                std::remove_if(properties_string.begin(), properties_string.end(), [](char c) { return c == ' '; }),
                std::end(properties_string));
            std::stringstream properties_stream(properties_string);
            while (properties_stream.good()) {
                std::string property_id;
                std::getline(properties_stream, property_id, ',');
                properties_ids.emplace_back(property_id);
            }
            STORM_LOG_THROW(!properties_ids.empty(), storm::exceptions::InvalidPropertyException, "Invalid property names provided.");
        }
        model_and_property.property = filterProperties(model_and_formulae.second, properties_ids, model_constants_map);
    }
    // Add the user-defined constants
    const auto user_constants_map = model_and_property.model.parseConstantDefinitions(settings.constants);
    model_and_property.model = model_and_property.model.preprocess(user_constants_map);
    model_and_property.property = storm::api::substituteConstantsInProperties(model_and_property.property, user_constants_map);
    model_and_property.property = storm::api::substituteTranscendentalNumbersInProperties(model_and_property.property);
    // Make sure that all properties have no undefined constant
    for (const auto& property : model_and_property.property) {
        STORM_LOG_THROW(
            property.getUndefinedConstants().empty(), storm::exceptions::InvalidPropertyException,
            "The property uses undefined constants!!!");
    }
    // Expand the model
    storm::jani::ModelFeatures supported_features = storm::api::getSupportedJaniFeatures(storm::builder::BuilderType::Explicit);
    storm::api::simplifyJaniModel(model_and_property.model.asJaniModel(), model_and_property.property, supported_features);
    return model_and_property;
}

std::vector<storm::jani::Property> filterProperties(
    const std::vector<storm::jani::Property>& properties, const std::vector<std::string>& properties_ids,
    const std::map<storm::expressions::Variable, storm::expressions::Expression>& model_constants_map) {
    // If no properties are provided, return all properties
    std::vector<storm::jani::Property> filtered_properties;
    if (properties_ids.empty()) {
        filtered_properties.reserve(properties.size());
        std::transform(
            properties.begin(), properties.end(), std::back_inserter(filtered_properties),
            [&model_constants_map](const auto& property) { return property.substitute(model_constants_map); });
    } else {
        // Filter the properties based on the id
        for (const auto& property_id : properties_ids) {
            bool property_found = false;
            for (const auto& property : properties) {
                if (property.getName() == property_id) {
                    // Do not forget to substitute Jani constants in the property, too!
                    filtered_properties.emplace_back(property.substitute(model_constants_map));
                    property_found = true;
                    break;
                }
            }
            STORM_LOG_THROW(property_found, storm::exceptions::InvalidPropertyException, "Cannot find " << property_id << " in model.");
        }
    }
    return filtered_properties;
}

}  // namespace smc_storm::parser