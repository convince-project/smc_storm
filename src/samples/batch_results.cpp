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

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/beta.hpp>

#include <storm/exceptions/UnexpectedException.h>
#include <storm/exceptions/NotImplementedException.h>
#include <storm/exceptions/OutOfRangeException.h>

#include <storm/utility/macros.h>
#include <storm/utility/constants.h>

#include "samples/sampling_results.hpp"

namespace smc_storm::samples {
BatchResults::BatchResults(size_t const batch_size, const state_properties::PropertyType prop_type)
: _batch_size{batch_size},
  _property_type{prop_type} {
    reset();
}

void BatchResults::addResult(const TraceInformation& res) {
    ++_count;
    switch (res.outcome)
    {
        case TraceResult::VERIFIED:
            _n_verified++;
            break;
        case TraceResult::NOT_VERIFIED:
            _n_not_verified++;
            break;
        case TraceResult::NO_INFO:
            _n_no_info++;
            break;
        default:
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Unexpected Result value added");
            break;
    }
    // TODO: Consider having a structure to track min-max value and distinguish between verified and non-verified traces
    _min_trace_length = std::min(_min_trace_length, res.trace_length);
    _max_trace_length = std::max(_max_trace_length, res.trace_length);
    if (_property_type == state_properties::PropertyType::R && res.outcome == TraceResult::VERIFIED) {
        _rewards.emplace_back(res.reward);
        _min_reward = std::min(res.reward, _min_reward);
        _max_reward = std::max(res.reward, _max_reward);
    }
}

BatchStatistics BatchResults::getBatchStatistics() const {
    return BatchStatistics(_rewards);
}

void BatchResults::reset() {
    _n_verified = 0U;
    _n_not_verified = 0U;
    _n_no_info = 0U;
    _count = 0U;
    _rewards.clear();
    _rewards.reserve(_batch_size);
    _min_reward = std::numeric_limits<double>::infinity();
    _max_reward = -std::numeric_limits<double>::infinity();
    _min_trace_length = std::numeric_limits<size_t>::max();
    _max_trace_length = 0U;
}
}  // namespace smc_storm::samples
