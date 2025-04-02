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
#pragma once

#include <vector>

#include "model_checker/smc_plugin_instance.hpp"
#include <storm/storage/jani/Property.h>
#include <storm/storage/SymbolicModelDescription.h>

namespace smc_storm::model_checker {
/*!
 * @brief A struct that holds a symbolic model and its properties, since Jani contains both in the same file.
 */
struct ModelAndProperties {
    // A generic model representation: can be either PRISM or JANI based.
    storm::storage::SymbolicModelDescription model;
    // A vector of (Jani) properties: default storage modality used from STORM.
    std::vector<storm::jani::Property> properties;
    // A vector of plugins, instantiated in the (JANI) model.
    std::vector<model_checker::SmcPluginInstance> plugins;
};
}  // namespace smc_storm::model_checker
