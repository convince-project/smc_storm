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
#include <string>

namespace smc_storm::settings {
/*!
 * @brief Data structure holding all configurations required by SMC_STORM
 */
struct SmcSettings {
    std::string model_file;
    std::string properties_file = "";
    std::string properties_names = "";
    std::string custom_property = "";
    std::string constants{""};
    std::string stat_method{""};
    std::string traces_file{""};
    double confidence{0.95};
    double epsilon{0.01};
    int max_trace_length{1000000};
    size_t max_n_traces{0U};
    size_t n_threads{1U};
    size_t batch_size{100U};
    bool show_statistics{false};

    bool validModel() const {
        const bool valid_file = !model_file.empty() && std::filesystem::exists(model_file);
        const auto extension = std::filesystem::path(model_file).extension();
        return valid_file && (extension == ".jani" || extension == ".prism");
    }

    bool validProperties() const {
        const auto model_extension = std::filesystem::path(model_file).extension();
        // At least one pf the two property definition should be empty.
        // If both are empty, all available properties will be verified
        const bool valid_property_definition = properties_names.empty() || custom_property.empty();
        // Jani models
        if (model_extension == ".jani") {
            // The property file shouldn't be provided, since it is contained in the model itself
            return properties_file.empty() && valid_property_definition;
        }
        // PRISM models
        if (!properties_file.empty()) {
            const auto properties_extension = std::filesystem::path(properties_file).extension();
            const bool valid_properties_file = properties_extension == ".props" && std::filesystem::exists(properties_file);
            return valid_properties_file && custom_property.empty();
        }
        return !custom_property.empty();
    }
};
}  // namespace smc_storm::settings
