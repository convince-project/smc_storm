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

#include "parser/parsers.hpp"

#include "state_generation/jani_smc_states_expansion.hpp"
#include "state_properties/property_description.hpp"
#include "state_properties/state_variable_information.hpp"

#include <storm/settings/SettingsManager.h>

#include <gtest/gtest.h>
#include <iostream>
#include <utility>

const std::filesystem::path TEST_FILE{"models/counters_test.jani"};

std::map<std::string, int64_t> stateToMap(
    const smc_storm::state_properties::StateVariableInformation<double>& var_info,
    const smc_storm::state_properties::StateVariableData<double>& state) {
    std::map<std::string, int64_t> state_map;
    for (size_t loc_idx = 0u; loc_idx < var_info.locationVariables().size(); loc_idx++) {
        const std::string& loc_name = var_info.locationVariables().at(loc_idx).variable.getName();
        const bool new_entry = state_map.emplace(loc_name, static_cast<int64_t>(state.getLocationData().at(loc_idx))).second;
        STORM_LOG_THROW(new_entry, storm::exceptions::UnexpectedException, "Location is already existing in the state_map.");
    }
    // Booleans
    for (size_t bool_idx = 0u; bool_idx < var_info.booleanVariables().size(); bool_idx++) {
        const int64_t bool_value = (state.getBoolData().at(bool_idx) ? 1 : 0);
        const bool new_entry = state_map.emplace(var_info.booleanVariables().at(bool_idx).variable.getName(), bool_value).second;
        STORM_LOG_THROW(new_entry, storm::exceptions::UnexpectedException, "Boolean variable is already existing in the state map.");
    }
    // Integers
    for (size_t int_idx = 0u; int_idx < var_info.integerVariables().size(); int_idx++) {
        const std::string& int_name = var_info.integerVariables().at(int_idx).variable.getName();
        const bool new_entry = state_map.emplace(int_name, state.getIntData().at(int_idx)).second;
        STORM_LOG_THROW(new_entry, storm::exceptions::UnexpectedException, "Int variable is already existing in the state map.");
    }
    // Reals are skipped in this test
    STORM_LOG_THROW(var_info.realVariables().empty(), storm::exceptions::UnexpectedException, "Expect no real values in the model.");
    return state_map;
}

void printStateMap(const std::map<std::string, int64_t>& state_map) {
    static size_t counter = 0u;
    std::cout << "\nState Info n. " << counter++ << "\t:\n";
    for (const auto& [key, val] : state_map) {
        std::cout << "\t" << key << ":\t" << val << "\n";
    }
}

std::pair<storm::jani::Model, smc_storm::state_properties::PropertyDescription> loadModelAndProperty(
    const std::filesystem::path& jani_path) {
    const auto [jani_model, jani_properties, jani_plugins] = smc_storm::parser::loadJaniModel(TEST_FILE, {});
    STORM_LOG_THROW(jani_properties.size() == 1, storm::exceptions::UnexpectedException, "Expected a single property.");
    const auto& jani_property = jani_properties[0];
    const auto& until_expression = jani_property.getRawFormula()->asProbabilityOperatorFormula().getSubformula();
    auto property_handler = smc_storm::state_properties::PropertyDescription(until_expression);
    property_handler.generateExpressions(jani_model.getManager(), {});
    return {jani_model, property_handler};
}

/*
Test the custom, cache-less JANI states generator
*/
TEST(JaniModelStatesGeneratorTest, TestCustomGenerator) {
    // Get Model and Property
    const auto model_and_property = loadModelAndProperty(TEST_FILE);
    const auto& jani_model = model_and_property.first;
    const auto& property_handler = model_and_property.second;
    // Test it is handled correctly
    std::default_random_engine rnd(0U);
    std::vector<smc_storm::model_checker::SmcPluginInstance> empty_plugins = {};
    auto custom_generator = smc_storm::state_generation::JaniSmcStatesExpansion<double>(jani_model, std::nullopt, empty_plugins, rnd);
    const auto& var_info = custom_generator.getVariableInformation();
    {
        const auto& init_state = custom_generator.setInitialState();
        const auto& state_map = stateToMap(var_info, init_state);
        // printStateMap(state_map);
        ASSERT_EQ(state_map.at("_loc_aut_0"), 0);
        ASSERT_EQ(state_map.at("_loc_aut_1"), 0);
        ASSERT_EQ(state_map.at("counter_ones"), 0);
        ASSERT_EQ(state_map.at("counter_tens"), 0);
        ASSERT_TRUE(custom_generator.satisfies(property_handler.getConditionExpression()));
        ASSERT_FALSE(custom_generator.satisfies(property_handler.getTargetExpression()));
    }
    {
        const auto possible_actions = custom_generator.getAvailableActions();
        ASSERT_EQ(possible_actions.size(), 2U);
        // Expect the first action to be plus_one (non-synching)
        const auto possible_destinations = custom_generator.getDestinationsFromAction(0U);
        ASSERT_EQ(possible_destinations.size(), 1U);
        const auto [next_state, dest_reward] = custom_generator.setNextState(0U, 0U);
        const auto& state_map = stateToMap(var_info, next_state);
        // printStateMap(state_map);
        ASSERT_EQ(state_map.at("_loc_aut_0"), 0);
        ASSERT_EQ(state_map.at("_loc_aut_1"), 0);
        ASSERT_EQ(state_map.at("counter_ones"), 1);
        ASSERT_EQ(state_map.at("counter_tens"), 0);
    }
    {
        const auto possible_actions = custom_generator.getAvailableActions();
        ASSERT_EQ(possible_actions.size(), 2U);
        // Expect the first action to be plus_one (non-synching)
        const auto possible_destinations = custom_generator.getDestinationsFromAction(1U);
        ASSERT_EQ(possible_destinations.size(), 2U);
        ASSERT_NEAR(possible_destinations[1].second, 0.9, 0.001);
        const auto [next_state, dest_reward] = custom_generator.setNextState(1U, 1U);
        const auto& state_map = stateToMap(var_info, next_state);
        // printStateMap(state_map);
        ASSERT_EQ(state_map.at("_loc_aut_0"), 0);
        ASSERT_EQ(state_map.at("_loc_aut_1"), 1);
        ASSERT_EQ(state_map.at("counter_ones"), 1);
        ASSERT_EQ(state_map.at("counter_tens"), 1);
    }
    {
        const auto possible_actions = custom_generator.getAvailableActions();
        ASSERT_EQ(possible_actions.size(), 1U);
        // Expect the first action to be plus_one (non-synching)
        const auto possible_destinations = custom_generator.getDestinationsFromAction(0U);
        ASSERT_EQ(possible_destinations.size(), 1U);
        const auto [next_state, dest_reward] = custom_generator.setNextState(0U, 0U);
        const auto& state_map = stateToMap(var_info, next_state);
        // printStateMap(state_map);
        ASSERT_EQ(state_map.at("_loc_aut_0"), 0);
        ASSERT_EQ(state_map.at("_loc_aut_1"), 1);
        ASSERT_EQ(state_map.at("counter_ones"), 2);
        ASSERT_EQ(state_map.at("counter_tens"), 1);
        ASSERT_TRUE(custom_generator.satisfies(property_handler.getConditionExpression()));
        ASSERT_FALSE(custom_generator.satisfies(property_handler.getTargetExpression()));
    }
    for (size_t idx = 3U; idx <= 9U; idx++) {
        const auto possible_actions = custom_generator.getAvailableActions();
        ASSERT_EQ(possible_actions.size(), 1U);
        // Expect the first action to be plus_one (non-synching)
        const auto possible_destinations = custom_generator.getDestinationsFromAction(0U);
        ASSERT_EQ(possible_destinations.size(), 1U);
        const auto [next_state, dest_reward] = custom_generator.setNextState(0U, 0U);
        const auto& state_map = stateToMap(var_info, next_state);
        ASSERT_EQ(state_map.at("_loc_aut_0"), 0);
        ASSERT_EQ(state_map.at("_loc_aut_1"), 1);
        ASSERT_EQ(state_map.at("counter_ones"), idx);
        ASSERT_EQ(state_map.at("counter_tens"), 1);
    }
    ASSERT_TRUE(custom_generator.satisfies(property_handler.getConditionExpression()));
    ASSERT_TRUE(custom_generator.satisfies(property_handler.getTargetExpression()));
    // See if the reset provides the expected state
    {
        const auto& init_state = custom_generator.setInitialState();
        const auto& state_map = stateToMap(var_info, init_state);
        // printStateMap(state_map);
        ASSERT_EQ(state_map.at("_loc_aut_0"), 0);
        ASSERT_EQ(state_map.at("_loc_aut_1"), 0);
        ASSERT_EQ(state_map.at("counter_ones"), 0);
        ASSERT_EQ(state_map.at("counter_tens"), 0);
        ASSERT_TRUE(custom_generator.satisfies(property_handler.getConditionExpression()));
        ASSERT_FALSE(custom_generator.satisfies(property_handler.getTargetExpression()));
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Initialize the default STORM settings (required for comparator)
    storm::settings::initializeAll("smc_storm", "test_states_generator");
    return RUN_ALL_TESTS();
}
