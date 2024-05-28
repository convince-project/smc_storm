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
#include "samples/batch_statistics.h"
#include "samples/trace_information.hpp"

namespace smc_storm::samples {

class SamplingResults {
   public:
    /*!
     * @brief The kind of property we are evaluating
     */
    enum class PropertyType {
        P,
        R
    };

    /*!
     * @brief A structure to keep results from a single batch. They will be added to the SamplingResults object at once.
     */
    struct BatchResults {
        const size_t _batch_size;
        const PropertyType _property_type;
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
        BatchResults(size_t batch_size, const PropertyType prop);
        
        /*!
         * @brief Check whether we need more samples according to the batch size
         * @return true if the have less elements than the batch size. False otherwise
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

    // Disable empty constructor (will use the one below)
    SamplingResults() = delete;

    /*!
     * @brief Initialize a SamplingResults object
     * @param settings The settings object containing the configuration for the MC task
     * @param prop Whether we are evaluating a probability or a reward property
     */
    explicit SamplingResults(const settings::SmcSettings& settings, PropertyType const& prop);

    /*!
     * @brief Get the batch_size configured in the constructor
     * @return A const reference to the batch_size member variable
     */
    inline size_t const& getBatchSize() const {
        if (_settings.max_n_traces > 0) {
            return std::min(_settings.max_n_traces, _settings.batch_size);
        }
        return _settings.batch_size;
    }

    BatchResults getBatchResultInstance() const;

    /*!
     * @brief Add the results from a batch to the SampledResults.
     * @param res Collection of results from a single batch. It is expected to be reset afterwards!
     */
    void addBatchResults(BatchResults const& res);

    /*!
     * @brief Get the amount of samples that returned the requested result
     * @param res The result value we are looking for
     * @return The n. of times we got the requested result.
     */
    size_t getResultCount(TraceResult const res) const;

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
     * @brief Check  if we need to sample another batch of results
     * @return Whether we need to sample a new batch or not.
     */
    bool newBatchNeeded() const;

    /*!
     * @brief Print the Sampling results to std::out stream
     */
    void printResults() const;

   private:
    /*!
     * @brief Check if we have reached the minimum n. of iterations
     * @return Whether we have reached the minimum n. of iterations
     */
    inline bool minIterationsReached() const {
        return _n_verified + _n_not_verified >= _min_iterations;
    }

    /*!
     * @brief Calculate the quantile associated to a specific confidence level (assuming normal distribution)
     * @param confidence The confidence value to use
     * @return The quantile value that will be used to evaluate sampling bounds
     */
    static double calculateQuantile(double const& confidence);

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
    // Chernoff bounds work both for P and R properties
    bool evaluateChernoffBound();
    // The following iteration bounds work only on proportions (P properties)
    bool evaluateWaldBound();
    bool evaluateAgrestiBound();
    bool evaluateWilsonBound();
    bool evaluateWilsonCorrectedBound();
    bool evaluateClopperPearsonBound();
    bool evaluateAdaptiveBound();
    bool evaluateArcsineBound();
    // The following iteration bounds work for R properties
    bool evaluateZInterval();
    bool evaluateChowRobbinsBound();

    mutable std::mutex _mtx;
    // The kind of property we need to evaluate
    const PropertyType _property_type;
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

    // Constants needed for computing whether a new batch is needed
    const settings::SmcSettings& _settings;
    const double _quantile;
    // Evaluator of choice to define whether we need more samples or not
    std::function<bool()> _bound_function;
    const size_t _min_iterations;
};

}  // namespace smc_storm::samples
