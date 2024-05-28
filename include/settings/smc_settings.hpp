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
#include <string>
namespace smc_storm::settings
{
    struct SmcSettings  {
        std::string model;
        std::string property_name;
        std::string constants{""};
        std::string stat_method {""};
        std::string traces_file {""};
        double confidence {0.95};
        double epsilon {0.01};
        int max_trace_length {1000000};
        size_t max_n_traces {0U};
        size_t n_threads{1U};
        size_t batch_size{100U};
        bool show_statistics{false};
    };
} // namespace smc_storm::settings
