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

#include <storm/exceptions/UnexpectedException.h>

#include <storm/utility/macros.h>

#include "samples/sampling_results.hpp"

namespace smc_storm::samples {
BatchResults::BatchResults(const size_t batch_size, const state_properties::PropertyType prop_type)
    : batch_size{batch_size}, property_type{prop_type} {
    reset();
}

void BatchResults::addResult(const TraceInformation& res) {
    ++count;
    switch (res.outcome) {
    case TraceResult::VERIFIED:
        n_verified++;
        break;
    case TraceResult::NOT_VERIFIED:
        n_not_verified++;
        break;
    case TraceResult::NO_INFO:
        n_no_info++;
        break;
    default:
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Unexpected Result value added");
        break;
    }
    // TODO: Consider having a structure to track min-max value and distinguish between verified and non-verified traces
    min_trace_length = std::min(min_trace_length, res.trace_length);
    max_trace_length = std::max(max_trace_length, res.trace_length);
    if (property_type == state_properties::PropertyType::R && res.outcome == TraceResult::VERIFIED) {
        rewards.emplace_back(res.reward);
        min_reward = std::min(res.reward, min_reward);
        max_reward = std::max(res.reward, max_reward);
    }
}

BatchStatistics BatchResults::getBatchStatistics() const {
    return BatchStatistics(rewards);
}

void BatchResults::reset() {
    n_verified = 0U;
    n_not_verified = 0U;
    n_no_info = 0U;
    count = 0U;
    rewards.clear();
    rewards.reserve(batch_size);
    min_reward = std::numeric_limits<double>::infinity();
    max_reward = -std::numeric_limits<double>::infinity();
    min_trace_length = std::numeric_limits<size_t>::max();
    max_trace_length = 0U;
}
}  // namespace smc_storm::samples
