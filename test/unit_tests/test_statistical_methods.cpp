/*
 * Copyright (c) 2025 Robert Bosch GmbH and its subsidiaries
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

#include <gtest/gtest.h>

#include <storm/settings/SettingsManager.h>

#include "samples/sampling_results.hpp"
#include "settings/smc_settings.hpp"
#include "settings/user_settings.hpp"
#include "state_properties/property_type.hpp"

smc_storm::settings::SmcSettings getRequiredSettings(const std::string& stat_method) {
    smc_storm::settings::UserSettings settings;
    settings.stat_method = stat_method;
    settings.n_threads = 1u;
    settings.hide_prog_bar = true;
    return smc_storm::settings::SmcSettings(settings);
}

smc_storm::samples::BatchResults getSamplesBatch(
    const smc_storm::state_properties::PropertyType p_type, const size_t n_successes, const size_t n_failures,
    const size_t n_not_terminated) {
    const size_t batch_size = n_successes + n_failures + n_not_terminated;
    if (batch_size <= 0u) {
        throw std::invalid_argument("Batch size must be greater than 0.");
    }
    smc_storm::samples::BatchResults single_batch(batch_size, p_type);
    single_batch.count = batch_size;
    single_batch.n_verified = n_successes;
    single_batch.n_not_verified = n_failures;
    single_batch.n_no_info = n_not_terminated;
    return single_batch;
}

void runTest(const smc_storm::state_properties::PropertyType p_type, const std::string& stat_method, const double success_prob) {
    auto settings = getRequiredSettings(stat_method);
    smc_storm::samples::SamplingResults samples_holder(settings, p_type);
    bool always_fail = success_prob < FLT_EPSILON;
    size_t n_succ = 0u;
    size_t n_fail = 0u;
    size_t inverse_prob = always_fail ? std::numeric_limits<size_t>::max() : static_cast<size_t>(std::round(1.0 / success_prob));
    size_t n_batches = 0;
    while (samples_holder.newBatchNeeded(0u)) {
        if (always_fail || n_batches % inverse_prob != 0u) {
            n_succ = 0u;
            n_fail = 1u;
        } else {
            n_succ = 1u;
            n_fail = 0u;
        }
        samples_holder.addBatchResults(getSamplesBatch(p_type, n_succ, n_fail, 0u), 0u);
        n_batches++;
    }
    EXPECT_GT(n_batches, 100u);
    if (!(always_fail || inverse_prob == 1U)) {
        // In this case, we expect more samples
        EXPECT_GT(n_batches, 600u);
    }
    EXPECT_NEAR(samples_holder.getProbabilityVerifiedProperty(), success_prob, 0.001);
}

TEST(SamplingResultsTest, ChernoffBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    auto settings = getRequiredSettings("chernoff");
    smc_storm::samples::SamplingResults samples_holder(settings, p_type);
    // Simulate some verified and not verified samples
    for (size_t i = 0u; i < 20u; i++) {
        auto batch = getSamplesBatch(p_type, 500u, 500u, 10u);
        samples_holder.addBatchResults(batch, 0u);
        ASSERT_EQ(samples_holder.newBatchNeeded(0u), i < 18u);
    }
    ASSERT_NEAR(samples_holder.getProbabilityVerifiedProperty(), 0.5, 0.001);
}

TEST(SamplingResultsTest, WaldBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "wald";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

TEST(SamplingResultsTest, WilsonBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "wilson";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

TEST(SamplingResultsTest, WilsonCorrectedBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "wilson_corrected";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

TEST(SamplingResultsTest, ClopperPearsonBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "clopper_pearson";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

TEST(SamplingResultsTest, AdaptiveBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "adaptive";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

TEST(SamplingResultsTest, ArcsineBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "arcsine";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

TEST(SamplingResultsTest, ChowRobbinsBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    const std::string stat_method = "chow_robbins";
    runTest(p_type, stat_method, 1.0);
    runTest(p_type, stat_method, 0.0);
    runTest(p_type, stat_method, 0.25);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Initialize the default STORM settings (required for comparator)
    storm::settings::initializeAll("smc_storm", "test_batch_buffer");
    return RUN_ALL_TESTS();
}
