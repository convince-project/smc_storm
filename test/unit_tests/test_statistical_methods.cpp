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

smc_storm::samples::SamplingResults getSamplingResult(
    const std::string& stat_method, const smc_storm::state_properties::PropertyType p_type) {
    smc_storm::settings::UserSettings settings;
    settings.stat_method = stat_method;
    settings.n_threads = 1u;
    return smc_storm::samples::SamplingResults(smc_storm::settings::SmcSettings(settings), p_type);
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

TEST(SamplingResultsTest, ChernoffBoundProgress) {
    const auto p_type = smc_storm::state_properties::PropertyType::P;
    auto samples_holder = getSamplingResult("chernoff", p_type);
    // Simulate some verified and not verified samples
    for (size_t i = 0u; i < 19; i++) {
        auto batch = getSamplesBatch(p_type, 500u, 500u, 10u);
        samples_holder.addBatchResults(batch, 0u);
        ASSERT_EQ(samples_holder.newBatchNeeded(0u), i < 18u);
    }
    // (You may need to expose or mock internal state for more detailed tests)
    ASSERT_NEAR(samples_holder.getProbabilityVerifiedProperty(), 0.5, 0.001);
}

// TEST(SamplingResultsTest, WaldBoundProgress) {
//     settings.stat_method = "wald";
//     SamplingResults waldResults(settings, PropertyType::P);
//     bool moreNeeded = waldResults.evaluateWaldBound();
//     ASSERT_TRUE(moreNeeded);
// }

// Add similar tests for Wilson, WilsonCorrected, ClopperPearson, Adaptive, Arcsine, ChowRobbins

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Initialize the default STORM settings (required for comparator)
    storm::settings::initializeAll("smc_storm", "test_batch_buffer");
    return RUN_ALL_TESTS();
}
