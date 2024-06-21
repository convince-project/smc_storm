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

#include <stddef.h>

#include <functional>
#include <string>
#include <mutex>

#include "settings/smc_settings.hpp"
#include "samples/batch_statistics.hpp"
#include "samples/trace_information.hpp"
#include "state_properties/property_type.hpp"

namespace smc_storm::samples {
/*!
 * @brief Object to hold a predefined number of traces results generated from a single thread
 */
struct BatchResults {
    const size_t _batch_size;
    const state_properties::PropertyType _property_type;
    // Property verification results
    // Members counting the sampled results in a single batch
    size_t _n_verified = 0U;
    size_t _n_not_verified = 0U;
    size_t _n_no_info = 0U;
    // A counter for the total amount of sampled results
    size_t _count = 0U;
    // A vector keeping the collected rewards
    std::vector<double> _rewards;
    // Min and max trace length
    size_t _min_trace_length = std::numeric_limits<size_t>::max();
    size_t _max_trace_length = 0U;

    // Rewards accumulation results (note: it should be templated with ValueType!)
    double _min_reward = std::numeric_limits<double>::infinity();
    double _max_reward = -std::numeric_limits<double>::infinity();

    // Constructor MUST define a batch size
    BatchResults() = delete;
    /*!
     * @brief BatchResults constructor
     * @param batch_size The predefined size of each batch
     * @param prop The property type we are evaluating (probability or reward), to determine what information to store
     */
    BatchResults(size_t batch_size, const state_properties::PropertyType prop);
    
    /*!
    * @brief Check whether we need more samples according to the batch size
    * @return true if the batch contains less elements than the batch size, false otherwise
    */
    inline bool batchIncomplete() const {
        return _count < _batch_size;
    }

    /*!
    * @brief Add a new result to the batch
    * @param res Result from a single trace
    */
    void addResult(const TraceInformation& res);

    /*!
    * @brief Computes the statistics for the current batch and returns them
    * @return The stats for the current batch
    */
    BatchStatistics getBatchStatistics() const;

    /*!
    * @brief Get a count of the n. of results stored in the batch
    * @return A const reference to the aforementioned counter
    */
    inline size_t const& getCount() const {
        return _count;
    }

    /*!
    * @brief Reset all members to 0
    */
    void reset();
};
}  // namespace smc_storm::samples
