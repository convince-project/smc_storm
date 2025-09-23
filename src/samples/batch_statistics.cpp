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

#include <storm/exceptions/IllegalArgumentValueException.h>
#include <storm/utility/macros.h>

#include "samples/batch_statistics.hpp"

namespace smc_storm::samples {

BatchStatistics::BatchStatistics() : mean{0.0}, variance{0.0}, dim{0U} {}

BatchStatistics::BatchStatistics(const std::vector<double>& samples) {
    dim = samples.size();
    min_val = std::numeric_limits<double>::infinity();
    max_val = -std::numeric_limits<double>::infinity();
    STORM_LOG_THROW(dim > 0U, storm::exceptions::IllegalArgumentValueException, "No samples.");
    // Compute mean, min and max
    mean = 0.0;
    for (const auto& sample : samples) {
        mean += sample;
        min_val = std::min(min_val, sample);
        max_val = std::max(max_val, sample);
    }
    mean /= static_cast<double>(dim);
    // Compute variance from samples and mean
    variance = 0.0;
    for (const auto& sample : samples) {
        const double deviation = sample - mean;
        variance += deviation * deviation;
    }
    variance /= static_cast<double>(dim);
}

BatchStatistics::BatchStatistics(const BatchStatistics& other_stats) {
    mean = other_stats.mean;
    variance = other_stats.variance;
    dim = other_stats.dim;
    min_val = other_stats.min_val;
    max_val = other_stats.max_val;
}

void BatchStatistics::updateStats(const BatchStatistics& other_stats) {
    // If the current batch is empty, just copy the other batch
    if (dim == 0U) {
        *this = BatchStatistics(other_stats);
        return;
    }
    const double total_dim = static_cast<double>(dim + other_stats.dim);
    const double new_mean = (mean * dim + other_stats.mean * other_stats.dim) / total_dim;
    const double variance_lin_comb = (dim * variance + other_stats.dim * other_stats.variance) / total_dim;
    const double variance_correction =
        (mean - other_stats.mean) * (mean - other_stats.mean) * dim * other_stats.dim / (total_dim * total_dim);
    // Update current stats
    mean = new_mean;
    variance = variance_lin_comb + variance_correction;
    dim = dim + other_stats.dim;
    min_val = std::min(min_val, other_stats.min_val);
    max_val = std::max(max_val, other_stats.max_val);
}

}  // namespace smc_storm::samples
