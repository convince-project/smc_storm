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
#include <mutex>
#include <string>

#include "samples/batch_buffer.hpp"
#include "samples/batch_results.hpp"
#include "samples/batch_statistics.hpp"
#include "samples/trace_information.hpp"
#include "settings/smc_settings.hpp"
#include "state_properties/property_type.hpp"

namespace smc_storm::samples {

class SamplingResults {
  public:
    // Disable empty constructor (will use the one below)
    SamplingResults() = delete;

    /*!
     * @brief Initialize a SamplingResults object
     * @param settings The settings object containing the configuration for the MC task
     * @param prop Whether we are evaluating a probability or a reward property
     */
    explicit SamplingResults(const settings::SmcSettings& settings, const state_properties::PropertyType& prop);

    /*!
     * @brief Get the batch_size configured in the constructor
     * @return A const reference to the batch_size member variable
     */
    inline const size_t& getBatchSize() const {
        if (_settings.max_n_traces > 0) {
            return std::min(_settings.max_n_traces, _settings.batch_size);
        }
        return _settings.batch_size;
    }

    BatchResults getBatchResultInstance() const;

    /*!
     * @brief Add the results from a batch to the Buffer. If all threads reported results, process the data
     * @param res Collection of results from a single batch. It is expected to be reset afterwards!
     * @param thread_id The id of the thread that generated the results
     */
    void addBatchResults(const BatchResults& res, const size_t thread_id);

    /*!
     * @brief Get the amount of samples that returned the requested result
     * @param res The result value we are looking for
     * @return The n. of times we got the requested result.
     */
    size_t getResultCount(const TraceResult res) const;

    /*!
     * @brief Compute the estimated reward from the sampled trajectories
     * @return The estimated reward as average of sampled rewards
     */
    double getEstimatedReward() const;

    /*!
     * @brief Compute the probability that a specific property is verified
     * @return Prob. that a property is verified as n. of verified samples over the n. of samples
     */
    double getProbabilityVerifiedProperty() const;

    /*!
     * @brief Check if we need to sample another batch of results. Blocks computation if the related buffer is full
     * @param thread_id The id of the thread that is checking if a new batch is needed
     * @return Whether we need to sample a new batch or not.
     */
    bool newBatchNeeded(const size_t thread_id) const;

    /*!
     * @brief Print the Sampling results to std::out stream
     */
    void printResults() const;

  private:
    /*!
     * @brief Process the results of a batch to update the internal state of the SamplingResults object
     * @param res The results of a batch to process
     */
    void processBatchResults(const BatchResults& res);

    /*!
     * @brief Check if we can continue the sampling process (not yet converged to a result and valid samples). Update the internal variable
     */
    void updateSamplingStatus();

    /*!
     * @brief Check if we have reached the minimum n. of iterations
     * @return Whether we have reached the minimum n. of iterations
     */
    inline bool minIterationsReached() const {
        return _n_verified + _n_not_verified >= _min_iterations;
    }

    /*!
     * @brief Calculate the variance of the collected samples, distinguishing between P and E properties
     * @return The compute variance
     */
    inline double calculateVariance() const {
        // Probability properties
        if (_property_type == state_properties::PropertyType::P) {
            // For Bernoulli distribution, he variance is computed as p(1-p) where p is the probability of success
            const double successes = static_cast<double>(_n_verified);
            const double failures = static_cast<double>(_n_not_verified);
            const double n_samples = successes + failures;
            return (n_samples * successes - successes * successes) / (n_samples * n_samples);
        }
        // Reward properties
        return _reward_stats.variance;
    }

    /*!
     * @brief Calculate the quantile associated to a specific confidence level (assuming normal distribution)
     * @param confidence The confidence value to use
     * @return The quantile value that will be used to evaluate sampling bounds
     */
    static double calculateQuantile(const double& confidence);

    /*!
     * @brief Get the n. of samples we need to collect to ensure result within bounds
     * @return The max n. of required samples according to the cmd params `confidence` and `maxabserror`
     */
    void updateChernoffBound();

    /*!
     * @brief Generates the bound function depending on the cmd configuration
     */
    void initBoundFunction();

    // Collection of bound functions to use. Return value is whether more batches are needed.
    // The following iteration bounds work only on P properties
    bool evaluateWaldBound();
    bool evaluateAgrestiBound();
    bool evaluateWilsonBound();
    bool evaluateWilsonCorrectedBound();
    bool evaluateClopperPearsonBound();
    bool evaluateAdaptiveBound();
    bool evaluateArcsineBound();
    // The following iteration bounds work R properties, too
    bool evaluateChernoffBound();
    bool evaluateChowRobbinsBound();

    mutable std::mutex _mtx;
    // The kind of property we need to evaluate
    const state_properties::PropertyType _property_type;
    // Whether we should continue sampling or not (since convergence is reached or other conditions fired)
    bool _keep_sampling;
    // Variables to keep track of the sampled traces results (Used for P properties)
    size_t _n_verified;
    size_t _n_not_verified;
    size_t _n_no_info;
    // Variables to keep track of rewards (WIP, will need to be extended for different bound methods)
    double _min_reward;
    double _max_reward;
    BatchStatistics _reward_stats;

    // Variables to keep track of the trace lengths
    size_t _min_trace_length;
    size_t _max_trace_length;

    // Upper bound for n. of samples, coming from Chernoff bounds
    size_t _required_samples;

    const settings::SmcSettings& _settings;
    BatchBuffer _results_buffer;

    // Constants needed for computing whether a new batch is needed
    const double _quantile;
    const size_t _min_iterations;
    // Evaluator of choice to define whether we need more samples or not
    std::function<bool()> _bound_function;
};

}  // namespace smc_storm::samples
