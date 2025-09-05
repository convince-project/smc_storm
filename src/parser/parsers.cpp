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
#include <storm/exceptions/InvalidModelException.h>
#include <storm/exceptions/InvalidPropertyException.h>
#include <storm/exceptions/NotSupportedException.h>
#include <storm/storage/expressions/ExpressionManager.h>
#include <storm/utility/cli.h>
#include <storm/utility/macros.h>

#include <sstream>

#include "parser/parsers.hpp"

namespace smc_storm::parser {
model_checker::ModelAndProperties parseModelAndProperties(const smc_storm::settings::UserSettings& settings) {
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

model_checker::ModelAndProperties parseJaniModelAndProperties(const smc_storm::settings::UserSettings& settings) {
    std::vector<std::filesystem::path> plugin_paths;
    {
        std::stringstream str_stream(settings.plugin_paths);
        std::string path;
        while (std::getline(str_stream, path, ',')) {
            plugin_paths.emplace_back(path);
        }
    }
    auto model_properties_plugins = loadJaniModel(settings.model_file, plugin_paths);
    auto& jani_model = std::get<0>(model_properties_plugins);
    auto& jani_properties = std::get<1>(model_properties_plugins);
    auto& jani_plugins = std::get<2>(model_properties_plugins);
    const auto model_constants_map = jani_model.getConstantsSubstitution();
    std::vector<storm::jani::Property> loaded_properties;
    if (!settings.custom_property.empty()) {
        const storm::parser::FormulaParser formula_parser(jani_model.getManager().getSharedPointer());
        loaded_properties = formula_parser.parseFromString(settings.custom_property);
    } else {
        // Get the list of properties from a comma separated list
        const auto properties_ids = getRequestedProperties(settings.properties_names);
        loaded_properties = filterProperties(jani_properties, properties_ids, model_constants_map);
    }
    // Add the user-defined constants
    return substituteConstants({jani_model, loaded_properties, jani_plugins}, settings.constants);
}

model_checker::ModelAndProperties parsePrismModelAndProperties(const smc_storm::settings::UserSettings& settings) {
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
    return substituteConstants({prism_model, loaded_properties, {}}, settings.constants);
}

JaniModelPropertiesPlugins loadJaniModel(const std::filesystem::path& jani_file, const std::vector<std::filesystem::path>& plugin_paths) {
    // Load the required model and property
    storm::jani::ModelFeatures supported_features = storm::api::getSupportedJaniFeatures(storm::builder::BuilderType::Explicit);
    // Removing the array feature ensures the array entries are substituted with the expanded name.
    // e.g. array[0] -> array_at_0
    supported_features.remove(storm::jani::ModelFeature::Arrays);
    auto model_and_properties = JaniParserExtended::parseModelPropertiesAndPlugins(jani_file, plugin_paths);
    auto& jani_model = std::get<0>(model_and_properties);
    auto& jani_properties = std::get<1>(model_and_properties);
    jani_properties = storm::api::substituteConstantsInProperties(jani_properties, jani_model.getConstantsSubstitution());
    storm::api::simplifyJaniModel(jani_model, jani_properties, supported_features);
    jani_model.checkValid();
    return model_and_properties;
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

model_checker::ModelAndProperties substituteConstants(
    const model_checker::ModelAndProperties& model_and_properties, const std::string constants) {
    model_checker::ModelAndProperties ret_model_and_properties;
    // Add the user-defined constants
    auto user_constants_map = model_and_properties.model.parseConstantDefinitions(constants);
    std::map<storm::expressions::Variable, storm::expressions::Expression> all_defined_constants;

    // Do not use the preprocess function, since it also eliminates unused variables (required for plugins)
    if (model_and_properties.model.isJaniModel()) {
        auto jani_model = model_and_properties.model.asJaniModel().defineUndefinedConstants(user_constants_map);
        all_defined_constants = jani_model.getConstantsSubstitution();
        if (model_and_properties.plugins.empty()) {
            // This substitutes the non-changing variables with constants
            ret_model_and_properties.model = jani_model.substituteConstantsFunctionsTranscendentals();
        } else {
            // This makes sure that non-changing variables are preserved
            ret_model_and_properties.model = jani_model.substituteConstantsInPlace(true);
        }
    } else {
        const auto prism_model = model_and_properties.model.asPrismProgram().defineUndefinedConstants(user_constants_map);
        all_defined_constants = prism_model.getConstantsSubstitution();
        ret_model_and_properties.model = prism_model.substituteConstantsFormulas().substituteNonStandardPredicates();
    }
    ret_model_and_properties.properties =
        storm::api::substituteConstantsInProperties(model_and_properties.properties, all_defined_constants);
    ret_model_and_properties.properties = storm::api::substituteTranscendentalNumbersInProperties(ret_model_and_properties.properties);
    // Make sure that all properties have no undefined constant
    for (const auto& property : ret_model_and_properties.properties) {
        STORM_LOG_THROW(
            property.getUndefinedConstants().empty(), storm::exceptions::InvalidPropertyException,
            "The property uses undefined constants!!!");
    }
    // Update the info concerning constants in the various plugins amd save it in the output model
    for (const auto& loaded_plugin : model_and_properties.plugins) {
        auto plugin = loaded_plugin;
        plugin.assignConstantValues(all_defined_constants);
        ret_model_and_properties.plugins.emplace_back(std::move(plugin));
    }
    return ret_model_and_properties;
}

}  // namespace smc_storm::parser
