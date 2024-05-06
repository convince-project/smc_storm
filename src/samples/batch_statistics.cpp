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

#include "samples/batch_statistics.h"

namespace smc_storm::samples {

BatchStatistics::BatchStatistics() : mean{0.0}, variance{0.0}, dim{0U} {}

BatchStatistics::BatchStatistics(const std::vector<double>& samples) {
    dim = samples.size();
    STORM_LOG_THROW(dim > 0U, storm::exceptions::IllegalArgumentValueException, "No samples.");
    // Compute mean
    mean = 0.0;
    for (const auto& sample : samples) {
        mean += sample;
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

BatchStatistics::BatchStatistics(const BatchStatistics& otherStats) {
    mean = otherStats.mean;
    variance = otherStats.variance;
    dim = otherStats.dim;
}

void BatchStatistics::updateStats(const BatchStatistics& otherStats) {
    // If the current batch is empty, just copy the other batch
    if (dim == 0U) {
        mean = otherStats.mean;
        variance = otherStats.variance;
        dim = otherStats.dim;
        return;
    }
    const double totalDim = static_cast<double>(dim + otherStats.dim);
    const double newMean = (mean * dim + otherStats.mean * otherStats.dim) / totalDim;
    const double varianceLinComb = (dim * variance + otherStats.dim * otherStats.variance) / totalDim;
    const double varianceCorrection = (mean - otherStats.mean) * (mean - otherStats.mean) * dim * otherStats.dim / (totalDim * totalDim);
    // Update mean and variance
    mean = newMean;
    variance = varianceLinComb + varianceCorrection;
    dim = dim + otherStats.dim;
}

}  // namespace smc_storm::samples
