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

#include <stdio.h>
#include <vector>

namespace smc_storm::samples {
/*!
 * @brief Implementation of Simple statistics class + batch updates using Welford's algorithm
 */
struct BatchStatistics {
    double mean;
    double variance;
    double min_val;
    double max_val;
    size_t dim;

    /*!
     * @brief Empty constructor: sets everything to 0
     */
    BatchStatistics();

    /*!
     * @brief Constructor computing the statistics from sampled data
     * @param batch_samples The samples to use for computing the stats
     */
    BatchStatistics(const std::vector<double>& batch_samples);

    /*!
     * @brief Copy constructor
     * @param other_stats Stats to copy
     */
    BatchStatistics(const BatchStatistics& other_stats);

    /*!
     * @brief Integrate statistics from another batch in the current one
     * @param other_stats statistics we want to merge
     */
    void updateStats(const BatchStatistics& other_stats);
};
}  // namespace smc_storm::samples
