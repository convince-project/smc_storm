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

#include "settings/user_settings.hpp"

namespace smc_storm::settings {
/*!
 * @brief Data structure holding all configurations required by SMC_STORM
 */
struct SmcSettings {
    const std::string stat_method;
    const std::string traces_folder;
    const double confidence;
    const double epsilon;
    const int max_trace_length;
    const size_t max_n_traces;
    const size_t n_threads;
    const size_t batch_size;
    const bool traces_add_date;
    const bool plugins_loaded;
    const bool stop_after_failure;
    const bool store_only_not_verified;
    const bool cache_explored_states;
    const bool show_statistics;
    const bool hide_prog_bar;

    SmcSettings(const UserSettings& user_settings)
        : stat_method{user_settings.stat_method}, traces_folder{user_settings.traces_folder}, confidence{user_settings.confidence},
          epsilon{user_settings.epsilon}, max_trace_length{user_settings.max_trace_length}, max_n_traces{user_settings.max_n_traces},
          n_threads{user_settings.n_threads}, batch_size{user_settings.batch_size}, traces_add_date{user_settings.traces_add_date},
          plugins_loaded{!user_settings.plugin_paths.empty()}, stop_after_failure{user_settings.stop_after_failure},
          store_only_not_verified{user_settings.store_only_not_verified}, cache_explored_states{user_settings.cache_explored_states},
          show_statistics{user_settings.show_statistics}, hide_prog_bar(user_settings.hide_prog_bar) {}
};
}  // namespace smc_storm::settings
