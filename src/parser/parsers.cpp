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
#include <storm-parsers/parser/FormulaParser.h>
#include <storm/api/builder.h>
#include <storm/api/properties.h>
#include <storm/storage/expressions/ExpressionManager.h>
#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidPropertyException.h>
#include <storm/exceptions/NotSupportedException.h>
#include <storm/utility/macros.h>

#include "parser/parsers.hpp"

namespace smc_storm::parser {
SymbolicModelAndProperty parseModelAndProperties(const smc_storm::settings::UserSettings& settings) {
    // Ensure settings validity
    STORM_LOG_THROW(settings.validModel(), storm::exceptions::InvalidModelException, "Invalid model file provided.");
    STORM_LOG_THROW(settings.validProperties(), storm::exceptions::InvalidPropertyException, "Invalid properties provided.");
    const std::filesystem::path path_to_model(settings.model_file);
    if (path_to_model.extension() == ".jani") {
        return parseJaniModelAndProperties(settings);
    } else if (path_to_model.extension() == ".prism") {
        return parsePrismModelAndProperties(settings);
    }
    STORM_LOG_THROW(
        false, storm::exceptions::NotSupportedException,
        "Unrecognised file extension " << path_to_model.extension() << ": Only .jani and .prism are supported.");
}

SymbolicModelAndProperty parseJaniModelAndProperties(const smc_storm::settings::UserSettings& settings) {
    // Load the required model and property
    storm::jani::ModelFeatures supported_features = storm::api::getSupportedJaniFeatures(storm::builder::BuilderType::Explicit);
    // Removing the array feature ensures the array entries are substituted with the expanded name.
    // e.g. array[0] -> array_at_0
    supported_features.remove(storm::jani::ModelFeature::Arrays);
    auto model_and_formulae = storm::api::parseJaniModel(settings.model_file, supported_features);
    model_and_formulae.first.checkValid();
    const auto model_constants_map = model_and_formulae.first.getConstantsSubstitution();
    std::vector<storm::jani::Property> loaded_properties;
    if (!settings.custom_property.empty()) {
        const storm::parser::FormulaParser formula_parser(model_and_formulae.first.getManager().getSharedPointer());
        loaded_properties = formula_parser.parseFromString(settings.custom_property);
    } else {
        // Get the list of properties from a comma separated list
        const auto properties_ids = getRequestedProperties(settings.properties_names);
        loaded_properties = filterProperties(model_and_formulae.second, properties_ids, model_constants_map);
    }
    // Add the user-defined constants
    return substituteConstants({model_and_formulae.first, loaded_properties}, settings.constants);
}

SymbolicModelAndProperty parsePrismModelAndProperties(const smc_storm::settings::UserSettings& settings) {
    const auto prism_model = storm::api::parseProgram(settings.model_file);
    prism_model.checkValidity();
    const auto model_constants_map = prism_model.getConstantsSubstitution();
    const storm::parser::FormulaParser formula_parser(prism_model);
    std::vector<storm::jani::Property> loaded_properties;
    if (!settings.custom_property.empty()) {
        loaded_properties = formula_parser.parseFromString(settings.custom_property);
    } else {
        loaded_properties = formula_parser.parseFromFile(settings.properties_file);
        const auto properties_ids = getRequestedProperties(settings.properties_names);
        loaded_properties = filterProperties(loaded_properties, properties_ids, model_constants_map);
    }
    // Substitute constants in the model and properties
    const auto model_and_property = substituteConstants({prism_model, loaded_properties}, settings.constants);
    return model_and_property;
}

std::vector<std::string> getRequestedProperties(const std::string& properties_string) {
    std::vector<std::string> properties_ids = {};
    if (!properties_string.empty()) {
        std::string processed_properties = properties_string;
        processed_properties.erase(
            std::remove_if(processed_properties.begin(), processed_properties.end(), [](char c) { return c == ' '; }),
            std::end(processed_properties));
        std::stringstream properties_stream(processed_properties);
        while (properties_stream.good()) {
            std::string property_id;
            std::getline(properties_stream, property_id, ',');
            properties_ids.emplace_back(property_id);
        }
        STORM_LOG_THROW(!properties_ids.empty(), storm::exceptions::InvalidPropertyException, "Invalid property names provided.");
    }
    return properties_ids;
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

SymbolicModelAndProperty substituteConstants(const SymbolicModelAndProperty& model_and_properties, const std::string constants) {
    SymbolicModelAndProperty ret_model_and_properties;
    // Add the user-defined constants
    const auto user_constants_map = model_and_properties.model.parseConstantDefinitions(constants);
    ret_model_and_properties.model = model_and_properties.model.preprocess(user_constants_map);
    ret_model_and_properties.property = storm::api::substituteConstantsInProperties(model_and_properties.property, user_constants_map);
    ret_model_and_properties.property = storm::api::substituteTranscendentalNumbersInProperties(ret_model_and_properties.property);
    // Make sure that all properties have no undefined constant
    for (const auto& property : ret_model_and_properties.property) {
        STORM_LOG_THROW(
            property.getUndefinedConstants().empty(), storm::exceptions::InvalidPropertyException,
            "The property uses undefined constants!!!");
    }
    return ret_model_and_properties;
}

}  // namespace smc_storm::parser