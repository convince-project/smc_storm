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
SamplingResults::BatchResults::BatchResults(size_t const batchSize, const PropertyType prop_type) :
        _batchSize{batchSize},
        _property_type{prop_type} {
    reset();
}

void SamplingResults::BatchResults::addResult(const TraceInformation& res) {
    ++_count;
    switch (res.outcome)
    {
        case TraceResult::VERIFIED:
            _nVerified++;
            break;
        case TraceResult::NOT_VERIFIED:
            _nNotVerified++;
            break;
        case TraceResult::NO_INFO:
            _nNoInfo++;
            break;
        default:
            STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Unexpected Result value added");
            break;
    }
    // TODO: Consider having a structure to track min-max value and distinguish between verified and non-verified traces
    _min_trace_length = std::min(_min_trace_length, res.traceLength);
    _max_trace_length = std::max(_max_trace_length, res.traceLength);
    if (_property_type == PropertyType::R && res.outcome == TraceResult::VERIFIED) {
        _rewards.emplace_back(res.reward);
        _minReward = std::min(res.reward, _minReward);
        _maxReward = std::max(res.reward, _maxReward);
    }
}

BatchStatistics SamplingResults::BatchResults::getBatchStatistics() const {
    return BatchStatistics(_rewards);
}

void SamplingResults::BatchResults::reset() {
    _nVerified = 0U;
    _nNotVerified = 0U;
    _nNoInfo = 0U;
    _count = 0U;
    _rewards.clear();
    _rewards.reserve(_batchSize);
    _minReward = std::numeric_limits<double>::infinity();
    _maxReward = -std::numeric_limits<double>::infinity();
    _min_trace_length = std::numeric_limits<size_t>::max();
    _max_trace_length = 0U;
}

SamplingResults::SamplingResults(size_t const batch_size, PropertyType const& prop,  const double epsilon, const double confidence, const std::string& stat_method)
: _propertyType{prop},
  _stat_method{stat_method},
  _maxAbsErr{epsilon},
  _confidence{confidence},
  _quantile{calculateQuantile(_confidence)},
  _batchSize{batch_size},
  _minIterations{50U} {
    _nVerified = 0U;
    _nNotVerified = 0U;
    _nNoInfo = 0U;
    _minReward = std::numeric_limits<double>::infinity();
    _maxReward = -std::numeric_limits<double>::infinity();
    _min_trace_length = std::numeric_limits<size_t>::max();
    _max_trace_length = 0U;
    initBoundFunction();
}

void SamplingResults::initBoundFunction() {
    _boundFunction = nullptr;
    // Set the default method to evaluate the n. of iterations
    std::string stat_method = _stat_method;
    if (stat_method.empty()) {
        stat_method = (PropertyType::R == _propertyType) ? "chow_robbins" : "adaptive";
    }
    // Assign the correct bound function
    if (_propertyType == PropertyType::R) {
        // All iteration bounds related to Reward properties
        if ("chernoff" == stat_method) {
            updateChernoffBound();
            _boundFunction = std::bind(&SamplingResults::evaluateChernoffBound, this);
        } else if ("z_interval" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateZInterval, this);
        } else if ("chow_robbins" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateChowRobbinsBound, this);
        }
    } else {
        if ("chernoff" == stat_method) {
            updateChernoffBound();
            _boundFunction = std::bind(&SamplingResults::evaluateChernoffBound, this);
        } else if ("wald" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateWaldBound, this);
        } else if ("agresti" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateAgrestiBound, this);
        } else if ("wilson" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateWilsonBound, this);
        } else if ("wilson_corrected" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateWilsonCorrectedBound, this);
        } else if ("clopper_pearson" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateClopperPearsonBound, this);
        } else if ("adaptive" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateAdaptiveBound, this);
        } else if ("arcsine" == stat_method) {
            _boundFunction = std::bind(&SamplingResults::evaluateArcsineBound, this);
        }
    }
    STORM_LOG_THROW(_boundFunction, storm::exceptions::NotImplementedException, "Could not find bounds method " << stat_method);
}

bool SamplingResults::evaluateChernoffBound()
{
    return (_nVerified + _nNotVerified) < _requiredSamples;
}

bool SamplingResults::evaluateWaldBound()
{
    if (!minIterationsReached()) {
        return true;
    }
    const double successes = static_cast<double>(_nVerified);
    const double failures = static_cast<double>(_nNotVerified);
    const double iterations = successes + failures;
    const double successProportion = successes / iterations;
    double ciHalfWidth = _quantile * sqrt(successProportion * ( 1 - successProportion) / iterations);
    // Boolean to make sure the certainty bound computed with the Wald bound is inside the desired interval
    return ciHalfWidth > _maxAbsErr; 
}

bool SamplingResults::evaluateAgrestiBound()
{
    if (!minIterationsReached()) {
        return true;
    }
    const double successes = static_cast<double>(_nVerified);
    const double failures = static_cast<double>(_nNotVerified);
    const double iterations = successes + failures;
    const double adjustedSuccessProportion=(successes + 0.5 * _quantile) / (iterations + _quantile * _quantile);
    // Boolean to make sure the certainty bound computed with the Agresti bound is inside the desired interval
    return _quantile * sqrt(adjustedSuccessProportion * (1.0 - adjustedSuccessProportion) / (iterations + _quantile * _quantile))>_maxAbsErr;
}

bool SamplingResults::evaluateWilsonBound()
{
    if (!minIterationsReached()) {
        return true;
    }
    const double successes = static_cast<double>(_nVerified);
    const double failures = static_cast<double>(_nNotVerified);
    const double iterations = successes + failures;
    const double z = _quantile;
    const double zSq = z * z;
    const double ciHalfWidth = (z / (iterations + zSq)) * sqrt(successes * failures / iterations + zSq / 4.0);
    // Boolean to make sure the certainty bound computed with the Wilson bound is inside the desired interval
    return ciHalfWidth > _maxAbsErr;
}

bool SamplingResults::evaluateWilsonCorrectedBound()
{
    if (!minIterationsReached()) {
        return true;
    }
    const double successes = static_cast<double>(_nVerified);
    const double failures = static_cast<double>(_nNotVerified);
    const double iterations = successes + failures;
    const double p = (successes / iterations) - (1.0 / (2.0 * iterations));
    const double z = _quantile;
    const double zSq = z * z;
    const double ciHalfWidth = (z / (2.0 * (iterations + zSq))) * sqrt((2.0 * successes - 1.0) * (2.0 * failures + 1.0) * (1.0 / iterations) + zSq);
    // Boolean to make sure the certainty bound computed with the Wilson bound is inside the desired interval
    return ciHalfWidth > _maxAbsErr;
}

bool SamplingResults::evaluateClopperPearsonBound()
{
    if (!minIterationsReached()) {
        return true;
    }
    const double alpha_half = (1 - _confidence) * 0.5;
    const double successes = static_cast<double>(_nVerified);
    const double failures = static_cast<double>(_nNotVerified);
    const double iterations = successes + failures;
    double lowerBound = 0.0;
    double upperBound = 1.0;
    if (_nVerified == 0) {
        upperBound = 1.0 - pow(alpha_half, 1.0 / iterations);
    } else if (_nNotVerified == 0) {
        lowerBound = pow(alpha_half, 1.0 / iterations);
    } else {
        boost::math::beta_distribution lowerDist(successes, failures + 1.0);
        boost::math::beta_distribution upperDist(successes + 1.0, failures);
        lowerBound = boost::math::quantile(lowerDist, alpha_half);
        upperBound = boost::math::quantile(upperDist, 1.0 - alpha_half);
    }
    const double ciHalfWidth = std::abs(upperBound - lowerBound) * 0.5;
    return ciHalfWidth > _maxAbsErr;
}

bool SamplingResults::evaluateAdaptiveBound()
{
    // Based on "A New Adaptive Sampling Method for Scalable Learning" by Chen and Xu (2013)
    if (!minIterationsReached()) {
        return true;
    }
    const double alpha = (1 - _confidence);
    const double successes = static_cast<double>(_nVerified);
    const double iterations = static_cast<double>(_nVerified + _nNotVerified);
    const double successesProportion = successes / iterations;
    const double correctedSuccessProb = abs(successesProportion - 0.5) - (_maxAbsErr * 2.0 / 3.0);
    const double minExpectedIterations = (2.0 * log(2.0 / alpha) / (_maxAbsErr * _maxAbsErr)) * (0.25 - correctedSuccessProb * correctedSuccessProb);
    return iterations < minExpectedIterations;
}

bool SamplingResults::evaluateArcsineBound()
{
    if (!minIterationsReached()) {
        return true;
    }
    const double successes = static_cast<double>(_nVerified);
    const double failures = static_cast<double>(_nNotVerified);
    const double iterations = successes + failures;
    const double intervalModifier=_quantile / (2.0 * sqrt(iterations));
    const double adjustedSuccessProportion = (successes + 0.5 * _quantile) / (iterations + _quantile * _quantile);
    const double sqrtLowerBound = sin(asin(sqrt(adjustedSuccessProportion)) - intervalModifier);
    const double sqrtUpperBound = sin(asin(sqrt(adjustedSuccessProportion)) + intervalModifier);
    // Boolean to make sure the certainty bound computed with the Arcsine bound is inside the desired interval
    return abs(sqrtLowerBound * sqrtLowerBound - sqrtUpperBound * sqrtUpperBound) > _maxAbsErr * 2.0;
}

bool SamplingResults::evaluateZInterval()
{
    // Following https://www.statisticshowto.com/probability-and-statistics/confidence-interval/#CIZ2
    if (_rewardStats.dim == 0U) {
        // No sample yet! Keep getting samples!
        return true;
    }
    const double ciHalfWidth = _quantile * _rewardStats.variance / sqrt(_rewardStats.dim);
    return ciHalfWidth > _maxAbsErr;
}

bool SamplingResults::evaluateChowRobbinsBound()
{
    // Based on "On the Asymptotic Theory of Fixed-Width Sequential Confidence Intervals for the Mean" paper (1965).
    if (!minIterationsReached()) {
        return true;
    }
    const double ciHalfWidthSquared = _rewardStats.variance * _quantile * _quantile / static_cast<double>(_rewardStats.dim);
    return (_maxAbsErr * _maxAbsErr) < ciHalfWidthSquared;
}

void SamplingResults::addBatchResults(BatchResults const& res) {
    const BatchStatistics batchStats = ((PropertyType::R == _propertyType) ? res.getBatchStatistics() : BatchStatistics());
    std::scoped_lock<std::mutex> lock(_mtx);
    _nVerified += res._nVerified;
    _nNotVerified += res._nNotVerified;
    _nNoInfo += res._nNoInfo;
    _min_trace_length = std::min(_min_trace_length, res._min_trace_length);
    _max_trace_length = std::max(_max_trace_length, res._max_trace_length);
    if (_propertyType == PropertyType::R) {
        _rewardStats.updateStats(batchStats);
        bool intervalChanged = false;
        if (_minReward > res._minReward) {
            intervalChanged = true;
            _minReward = res._minReward;
        }
        if (_maxReward < res._maxReward) {
            intervalChanged = true;
            _maxReward = res._maxReward;
        }
        if (intervalChanged) {
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
        return _nVerified;
    case TraceResult::NOT_VERIFIED:
        return _nNotVerified;
    case TraceResult::NO_INFO:
        return _nNoInfo;
    }
    STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Unexpected Result value added");
}

double SamplingResults::getEstimatedReward() const {
    if (_nNoInfo  > 0U || _nNotVerified > 0U) {
        STORM_LOG_WARN("Found samples not verifying the property. This results in infinite reward (cost).");
        return std::numeric_limits<double>::infinity();
    }
    STORM_LOG_THROW(_nVerified > 0U, storm::exceptions::OutOfRangeException, "Computing rewards without samples!");    
    return _rewardStats.mean;
}

double SamplingResults::getProbabilityVerifiedProperty() const
{
    const size_t nSamples = _nVerified + _nNotVerified;
    STORM_LOG_THROW(nSamples > 0U, storm::exceptions::OutOfRangeException, "No samples to use for evaluation.");
    return static_cast<double>(_nVerified) / static_cast<double>(nSamples);
}

bool SamplingResults::newBatchNeeded() const {
    // Reward properties require always reaching the target state
    if (_propertyType == PropertyType::R && (_nNoInfo  > 0U || _nNotVerified > 0U)) {
        return false;
    }
    // Check if we never reached a terminal states
    const size_t n_samples = _nNoInfo + _nNotVerified + _nVerified;
    if (n_samples > _minIterations && _nNoInfo > n_samples * 0.5) {
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "More than half the generated traces do not reach the terminal state. Aborting.");
    }
    return _boundFunction();
}

double SamplingResults::calculateQuantile(double const& confidence)
{
    boost::math::normal distributionNormal(0.0, 1.0);
    return boost::math::quantile(distributionNormal, 1.0 - ((1.0-confidence) * 0.5));
}

void SamplingResults::updateChernoffBound()
{
    // Use Chernoff bound for Bernoullian distribution
    double resultIntervalWidth = 1.0;
    if (_propertyType == PropertyType::R && !(std::isinf(_minReward) && std::isinf(_maxReward))) {
        resultIntervalWidth = std::max(1.0, _maxReward - _minReward);
    }
    _requiredSamples = storm::utility::ceil(storm::utility::log(2.0 / (1.0 - _confidence)) * storm::utility::pow(resultIntervalWidth, 2) / (2.0 * storm::utility::pow(_maxAbsErr, 2)));
}

void SamplingResults::printResults() const
{
    const size_t nSamples = _nNoInfo + _nNotVerified + _nVerified;
    STORM_PRINT("\n============= SMC Results =============\n");
    STORM_PRINT("\tN. of times target reached:\t" << _nVerified << "\n");
    STORM_PRINT("\tN. of times no termination:\t" << _nNoInfo << "\n");
    STORM_PRINT("\tTot. n. of tries (samples):\t" << nSamples << "\n");
    if (_propertyType == PropertyType::P) {
        STORM_PRINT("\tEstimated success prob.:\t" << getProbabilityVerifiedProperty() << "\n");
    } else {
        STORM_PRINT("\tEstimated average reward.:\t" << getEstimatedReward() << "\n");
        STORM_PRINT("\tRewards interval: [" << _minReward << ", " << _maxReward << "]\n");
    }
    STORM_PRINT("\tMin trace length:\t" << _min_trace_length << "\n");
    STORM_PRINT("\tMax trace length:\t" << _max_trace_length << "\n");
    STORM_PRINT("=========================================\n");
    STORM_LOG_ASSERT(_nNoInfo < nSamples, "All sampled paths do not terminate. Check your input model.");
}

}  // namespace smc_storm::samples
