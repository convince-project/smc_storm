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

#include "samples/sampling_results.h"

namespace smc_storm::samples {
SamplingResults::SamplingResults(const settings::SmcSettings& settings, state_properties::PropertyType const& prop)
: _settings{settings},
  _property_type{prop},
  _quantile{calculateQuantile(_settings.confidence)},
  _min_iterations{50U} {
    _n_verified = 0U;
    _n_not_verified = 0U;
    _n_no_info = 0U;
    _min_reward = std::numeric_limits<double>::infinity();
    _max_reward = -std::numeric_limits<double>::infinity();
    _min_trace_length = std::numeric_limits<size_t>::max();
    _max_trace_length = 0U;
    initBoundFunction();
}

BatchResults SamplingResults::getBatchResultInstance() const {
    return BatchResults(getBatchSize(), _property_type);
}

void SamplingResults::initBoundFunction() {
    _bound_function = nullptr;
    // Set the default method to evaluate the n. of iterations
    std::string stat_method = _settings.stat_method;
    const bool is_reward_prop = state_properties::PropertyType::R == _property_type;
    if (stat_method.empty()) {
        stat_method = is_reward_prop ? "chow_robbins" : "adaptive";
    }
    // Assign the correct bound function
    if (is_reward_prop) {
        // All iteration bounds related to Reward properties
        if ("chernoff" == stat_method) {
            updateChernoffBound();
            _bound_function = std::bind(&SamplingResults::evaluateChernoffBound, this);
        } else if ("z_interval" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateZInterval, this);
        } else if ("chow_robbins" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateChowRobbinsBound, this);
        }
    } else {
        if ("chernoff" == stat_method) {
            updateChernoffBound();
            _bound_function = std::bind(&SamplingResults::evaluateChernoffBound, this);
        } else if ("wald" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateWaldBound, this);
        } else if ("agresti" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateAgrestiBound, this);
        } else if ("wilson" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateWilsonBound, this);
        } else if ("wilson_corrected" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateWilsonCorrectedBound, this);
        } else if ("clopper_pearson" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateClopperPearsonBound, this);
        } else if ("adaptive" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateAdaptiveBound, this);
        } else if ("arcsine" == stat_method) {
            _bound_function = std::bind(&SamplingResults::evaluateArcsineBound, this);
        }
    }
    STORM_LOG_THROW(_bound_function, storm::exceptions::NotImplementedException, "Could not find bounds method " << stat_method);
}

bool SamplingResults::evaluateChernoffBound()
{
    return (_n_verified + _n_not_verified) < _required_samples;
}

bool SamplingResults::evaluateWaldBound()
{
    const double successes = static_cast<double>(_n_verified);
    const double failures = static_cast<double>(_n_not_verified);
    const double iterations = successes + failures;
    const double success_proportion = successes / iterations;
    double ci_half_width = _quantile * sqrt(success_proportion * ( 1 - success_proportion) / iterations);
    // Boolean to make sure the certainty bound computed with the Wald bound is inside the desired interval
    return ci_half_width > _settings.epsilon; 
}

bool SamplingResults::evaluateAgrestiBound()
{
    const double successes = static_cast<double>(_n_verified);
    const double failures = static_cast<double>(_n_not_verified);
    const double iterations = successes + failures;
    const double adjusted_success_proportion=(successes + 0.5 * _quantile) / (iterations + _quantile * _quantile);
    // Boolean to make sure the certainty bound computed with the Agresti bound is inside the desired interval
    return _quantile * sqrt(adjusted_success_proportion * (1.0 - adjusted_success_proportion) / (iterations + _quantile * _quantile)) > _settings.epsilon;
}

bool SamplingResults::evaluateWilsonBound()
{
    const double successes = static_cast<double>(_n_verified);
    const double failures = static_cast<double>(_n_not_verified);
    const double iterations = successes + failures;
    const double z = _quantile;
    const double zSq = z * z;
    const double ci_half_width = (z / (iterations + zSq)) * sqrt(successes * failures / iterations + zSq / 4.0);
    // Boolean to make sure the certainty bound computed with the Wilson bound is inside the desired interval
    return ci_half_width > _settings.epsilon;
}

bool SamplingResults::evaluateWilsonCorrectedBound()
{
    const double successes = static_cast<double>(_n_verified);
    const double failures = static_cast<double>(_n_not_verified);
    const double iterations = successes + failures;
    const double p = (successes / iterations) - (1.0 / (2.0 * iterations));
    const double z = _quantile;
    const double zSq = z * z;
    const double ci_half_width = (z / (2.0 * (iterations + zSq))) * sqrt((2.0 * successes - 1.0) * (2.0 * failures + 1.0) * (1.0 / iterations) + zSq);
    // Boolean to make sure the certainty bound computed with the Wilson bound is inside the desired interval
    return ci_half_width > _settings.epsilon;
}

bool SamplingResults::evaluateClopperPearsonBound()
{
    const double alpha_half = (1 - _settings.confidence) * 0.5;
    const double successes = static_cast<double>(_n_verified);
    const double failures = static_cast<double>(_n_not_verified);
    const double iterations = successes + failures;
    double lower_bound = 0.0;
    double upper_bound = 1.0;
    if (_n_verified == 0) {
        upper_bound = 1.0 - pow(alpha_half, 1.0 / iterations);
    } else if (_n_not_verified == 0) {
        lower_bound = pow(alpha_half, 1.0 / iterations);
    } else {
        boost::math::beta_distribution lower_dist(successes, failures + 1.0);
        boost::math::beta_distribution upper_dist(successes + 1.0, failures);
        lower_bound = boost::math::quantile(lower_dist, alpha_half);
        upper_bound = boost::math::quantile(upper_dist, 1.0 - alpha_half);
    }
    const double ci_half_width = std::abs(upper_bound - lower_bound) * 0.5;
    return ci_half_width > _settings.epsilon;
}

bool SamplingResults::evaluateAdaptiveBound()
{
    // Based on "A New Adaptive Sampling Method for Scalable Learning" by Chen and Xu (2013)
    const double alpha = (1 - _settings.confidence);
    const double successes = static_cast<double>(_n_verified);
    const double iterations = static_cast<double>(_n_verified + _n_not_verified);
    const double successes_proportion = successes / iterations;
    const double corrected_success_prob = abs(successes_proportion - 0.5) - (_settings.epsilon * 2.0 / 3.0);
    const double min_expected_iterations = (2.0 * log(2.0 / alpha) / (_settings.epsilon * _settings.epsilon)) * (0.25 - corrected_success_prob * corrected_success_prob);
    return iterations < min_expected_iterations;
}

bool SamplingResults::evaluateArcsineBound()
{
    const double successes = static_cast<double>(_n_verified);
    const double failures = static_cast<double>(_n_not_verified);
    const double iterations = successes + failures;
    const double interval_modifier=_quantile / (2.0 * sqrt(iterations));
    const double adjusted_success_proportion = (successes + 0.5 * _quantile) / (iterations + _quantile * _quantile);
    const double sqrt_lower_bound = sin(asin(sqrt(adjusted_success_proportion)) - interval_modifier);
    const double sqrt_upper_bound = sin(asin(sqrt(adjusted_success_proportion)) + interval_modifier);
    // Boolean to make sure the certainty bound computed with the Arcsine bound is inside the desired interval
    return abs(sqrt_lower_bound * sqrt_lower_bound - sqrt_upper_bound * sqrt_upper_bound) > _settings.epsilon * 2.0;
}

bool SamplingResults::evaluateZInterval()
{
    // Following https://www.statisticshowto.com/probability-and-statistics/confidence-interval/#CIZ2
    if (_reward_stats.dim == 0U) {
        // No sample yet! Keep getting samples!
        return true;
    }
    const double ci_half_width = _quantile * _reward_stats.variance / sqrt(_reward_stats.dim);
    return ci_half_width > _settings.epsilon;
}

bool SamplingResults::evaluateChowRobbinsBound()
{
    // Based on "On the Asymptotic Theory of Fixed-Width Sequential Confidence Intervals for the Mean" paper (1965).
    const double ci_half_width_squared = _reward_stats.variance * _quantile * _quantile / static_cast<double>(_reward_stats.dim);
    return (_settings.epsilon * _settings.epsilon) < ci_half_width_squared;
}

void SamplingResults::addBatchResults(BatchResults const& res) {
    std::scoped_lock<std::mutex> lock(_mtx);
    _n_verified += res._n_verified;
    _n_not_verified += res._n_not_verified;
    _n_no_info += res._n_no_info;
    _min_trace_length = std::min(_min_trace_length, res._min_trace_length);
    _max_trace_length = std::max(_max_trace_length, res._max_trace_length);
    if (_property_type == state_properties::PropertyType::R) {
        _reward_stats.updateStats(res.getBatchStatistics());
        bool is_interval_changed = false;
        if (_min_reward > res._min_reward) {
            is_interval_changed = true;
            _min_reward = res._min_reward;
        }
        if (_max_reward < res._max_reward) {
            is_interval_changed = true;
            _max_reward = res._max_reward;
        }
        if (is_interval_changed) {
            // Recalculate the Chernoff bounds, since the interval got larger
            updateChernoffBound();
        }
    }
}

size_t SamplingResults::getResultCount(TraceResult const res) const {
    std::scoped_lock<std::mutex> lock(_mtx);
    switch (res)
    {
    case TraceResult::VERIFIED:
        return _n_verified;
    case TraceResult::NOT_VERIFIED:
        return _n_not_verified;
    case TraceResult::NO_INFO:
        return _n_no_info;
    }
    STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Unexpected Result value added");
}

double SamplingResults::getEstimatedReward() const {
    if (_n_no_info  > 0U || _n_not_verified > 0U) {
        STORM_LOG_WARN("Found samples not verifying the property. This results in infinite reward (cost).");
        return std::numeric_limits<double>::infinity();
    }
    STORM_LOG_THROW(_n_verified > 0U, storm::exceptions::OutOfRangeException, "Computing rewards without samples!");    
    return _reward_stats.mean;
}

double SamplingResults::getProbabilityVerifiedProperty() const
{
    const size_t n_samples = _n_verified + _n_not_verified;
    STORM_LOG_THROW(n_samples > 0U, storm::exceptions::OutOfRangeException, "No samples to use for evaluation.");
    return static_cast<double>(_n_verified) / static_cast<double>(n_samples);
}

bool SamplingResults::newBatchNeeded() const {
    // Reward properties require always reaching the target state
    if (_property_type == state_properties::PropertyType::R && (_n_no_info  > 0U || _n_not_verified > 0U)) {
        return false;
    }
    const size_t n_samples = _n_no_info + _n_not_verified + _n_verified;
    if (_settings.max_n_traces > 0U && n_samples >= _settings.max_n_traces) {
        return false;
    }
    if (!minIterationsReached()) {
        return true;
    }
    // Check if we never reached a terminal states
    if (n_samples > _min_iterations && _n_no_info > n_samples * 0.5) {
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "More than half the generated traces do not reach the terminal state. Aborting.");
    }
    return _bound_function();
}

double SamplingResults::calculateQuantile(double const& confidence)
{
    boost::math::normal distribution_normal(0.0, 1.0);
    return boost::math::quantile(distribution_normal, 1.0 - ((1.0-confidence) * 0.5));
}

void SamplingResults::updateChernoffBound()
{
    // Use Chernoff bound for Bernoullian distribution
    double result_interval_width = 1.0;
    if (_property_type == state_properties::PropertyType::R && !(std::isinf(_min_reward) && std::isinf(_max_reward))) {
        result_interval_width = std::max(1.0, _max_reward - _min_reward);
    }
    _required_samples = storm::utility::ceil(storm::utility::log(2.0 / (1.0 - _settings.confidence)) * storm::utility::pow(result_interval_width, 2) / (2.0 * storm::utility::pow(_settings.epsilon, 2)));
}

void SamplingResults::printResults() const
{
    const size_t n_samples = _n_no_info + _n_not_verified + _n_verified;
    STORM_PRINT("\n============= SMC Results =============\n");
    STORM_PRINT("\tN. of times target reached:\t" << _n_verified << "\n");
    STORM_PRINT("\tN. of times no termination:\t" << _n_no_info << "\n");
    STORM_PRINT("\tTot. n. of tries (samples):\t" << n_samples << "\n");
    if (_property_type == state_properties::PropertyType::P) {
        STORM_PRINT("\tEstimated success prob.:\t" << getProbabilityVerifiedProperty() << "\n");
    } else {
        STORM_PRINT("\tEstimated average reward.:\t" << getEstimatedReward() << "\n");
        STORM_PRINT("\tRewards interval: [" << _min_reward << ", " << _max_reward << "]\n");
    }
    STORM_PRINT("\tMin trace length:\t" << _min_trace_length << "\n");
    STORM_PRINT("\tMax trace length:\t" << _max_trace_length << "\n");
    STORM_PRINT("=========================================\n");
    STORM_LOG_ASSERT(_n_no_info < n_samples, "All sampled paths do not terminate. Check your input model.");
}

}  // namespace smc_storm::samples
