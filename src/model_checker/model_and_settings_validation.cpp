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

#include "model_checker/model_and_settings_validation.hpp"

namespace smc_storm::model_checker {

bool areModelAndSettingsValid(const ModelAndProperties& model_and_properties, const settings::SmcSettings& settings) {
    bool valid_config = true;
    if (settings.cache_explored_states) {
        if (!model_and_properties.plugins.empty()) {
            STORM_LOG_ERROR("Found loaded plugins, but '--disable-explored-states-caching' is not enabled.");
            valid_config = false;
        }
        if (model_and_properties.model.isJaniModel()) {
            const auto& jani_model = model_and_properties.model.asJaniModel();
            // Check for real non-transient variables
            bool has_real_non_transient = jani_model.getGlobalVariables().containsNonTransientRealVariables();
            for (const auto& automaton : jani_model.getAutomata()) {
                if (automaton.getVariables().containsNonTransientRealVariables()) {
                    has_real_non_transient = false;
                }
            }
            if (has_real_non_transient) {
                STORM_LOG_ERROR("Found real non transient variables in JANI, but '--disable-explored-states-caching' is not enabled.");
                valid_config = false;
            }
        } else {
            // Since the parser worked before, we can expect to have a PRISM program here. No special checks here.
            valid_config = true;
        }
    } else {
        if (!model_and_properties.model.isJaniModel()) {
            STORM_LOG_ERROR("Only JANI models are supported when '--disable-explored-states-caching' is enabled.");
            return false;
        }
    }
    return valid_config;
}
}  // namespace smc_storm::model_checker
