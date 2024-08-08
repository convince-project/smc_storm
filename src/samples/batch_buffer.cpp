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

#include <algorithm>
#include <storm/exceptions/IllegalArgumentException.h>
#include <storm/utility/macros.h>

#include "samples/batch_buffer.hpp"
#include "samples/batch_results.hpp"

namespace smc_storm::samples {
BatchBuffer::BatchBuffer(const size_t n_threads, const size_t n_slots) : _n_slots{n_slots}, _results_buffer(n_threads) {
    STORM_LOG_THROW(n_threads > 0U, storm::exceptions::IllegalArgumentException, "Number of threads must be positive");
    STORM_LOG_THROW(n_slots > 0U, storm::exceptions::IllegalArgumentException, "Number of slots must be positive");
}

void BatchBuffer::addResults(const BatchResults& results, const size_t thread_id) {
    STORM_LOG_THROW(thread_id < _results_buffer.size(), storm::exceptions::IllegalArgumentException, "Thread id out of bounds");
    std::lock_guard<std::mutex> lock(_buffer_mutex);
    STORM_LOG_THROW(_results_buffer[thread_id].size() < _n_slots, storm::exceptions::IllegalArgumentException, "Buffer is full");
    _results_buffer.at(thread_id).emplace_back(results);
}

std::optional<std::vector<BatchResults>> BatchBuffer::getResults() {
    std::vector<BatchResults> results;
    {
        std::lock_guard<std::mutex> lock(_buffer_mutex);
        if (std::any_of(
                _results_buffer.begin(), _results_buffer.end(), [](const auto& thread_results) { return thread_results.empty(); })) {
            return std::nullopt;
        }
        for (auto& thread_results : _results_buffer) {
            results.push_back(thread_results.front());
            thread_results.pop_front();
        }
    }
    _buffer_cv.notify_all();
    return results;
}

void BatchBuffer::waitForSlotAvailable(const size_t thread_id) const {
    STORM_LOG_THROW(thread_id < _results_buffer.size(), storm::exceptions::IllegalArgumentException, "Thread id out of bounds");
    std::unique_lock<std::mutex> lock(_buffer_mutex);
    _buffer_cv.wait(lock, [this, thread_id] { return _results_buffer[thread_id].size() < _n_slots; });
}

}  // namespace smc_storm::samples
