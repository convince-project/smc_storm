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

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace smc_storm::samples {

// Forward declaration
struct BatchResults;

/*!
 * @brief Object to synchronize batch results between threads
 */
class BatchBuffer {
  public:
    /*!
     * @brief Constructor for the BatchBuffer object
     * @param n_threads Amount of threads used in the specific run instance
     * @param n_slots How many results to store for a single thread before blocking it
     */
    BatchBuffer(const size_t n_threads, const size_t n_slots);
    ~BatchBuffer() = default;

    /*!
     * @brief Cancel the buffer, unblocking all threads and preventing new samples from being added
     */
    void cancel();

    /*!
     * @brief Add results to the buffer. Will throw if the buffer is full
     * @param results The results to add
     * @param thread_id The id of the thread that generated the results
     */
    void addResults(const BatchResults& results, const size_t thread_id);

    /*!
     * @brief Provides a BatchResult per thread, if available
     * @return A vector with n_threads BatchResults, if available. Nullopt otherwise
     */
    std::optional<std::vector<BatchResults>> getResults();

    /*!
     * @brief Wait for a slot to be available in the buffer
     * @param thread_id The id of the thread that is waiting
     */
    void waitForSlotAvailable(const size_t thread_id) const;

  private:
    const size_t _n_slots;
    // Whether the buffer is still active or it was canceled
    bool _ok;
    // TODO: Consider to use a ring-buffer instead of a deque or vector of Batch results
    std::vector<std::deque<BatchResults>> _results_buffer;
    mutable std::mutex _buffer_mutex;
    mutable std::condition_variable _buffer_cv;
};
}  // namespace smc_storm::samples
