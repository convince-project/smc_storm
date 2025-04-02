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

#include <storm/settings/SettingsManager.h>

#include "samples/batch_buffer.hpp"
#include "samples/batch_results.hpp"

#include <gtest/gtest.h>

smc_storm::samples::BatchResults createBatchResults(const smc_storm::samples::TraceInformation& res) {
    smc_storm::samples::BatchResults result(1, smc_storm::state_properties::PropertyType::P);
    result.addResult(res);
    return result;
}

void testBatchResults(
    const smc_storm::samples::BatchResults& res, size_t n_verified, const size_t n_not_verified, const size_t n_no_info,
    const size_t max_trace_length) {
    EXPECT_EQ(res.n_verified, n_verified);
    EXPECT_EQ(res.n_not_verified, n_not_verified);
    EXPECT_EQ(res.n_no_info, n_no_info);
    EXPECT_EQ(res.count, n_verified + n_not_verified + n_no_info);
    EXPECT_EQ(res.max_trace_length, max_trace_length);
}

TEST(BatchBufferTest, AddAndGetResults) {
    // Create a BatchBuffer with 2 threads and 3 slots
    smc_storm::samples::BatchBuffer buffer(2, 3);
    buffer.addResults(createBatchResults({10, smc_storm::samples::TraceResult::VERIFIED, 0.0}), 0);
    EXPECT_EQ(buffer.getResults(), std::nullopt);
    buffer.addResults(createBatchResults({20, smc_storm::samples::TraceResult::NOT_VERIFIED, 0.0}), 0);
    EXPECT_EQ(buffer.getResults(), std::nullopt);
    buffer.addResults(createBatchResults({30, smc_storm::samples::TraceResult::NO_INFO, 0.0}), 0);
    EXPECT_EQ(buffer.getResults(), std::nullopt);
    buffer.addResults(createBatchResults({40, smc_storm::samples::TraceResult::VERIFIED, 0.0}), 1);
    auto buffer_results = buffer.getResults();
    EXPECT_NE(buffer_results, std::nullopt);
    EXPECT_EQ(buffer_results->size(), 2);
    testBatchResults(buffer_results->at(0), 1, 0, 0, 10);
    testBatchResults(buffer_results->at(1), 1, 0, 0, 40);
    EXPECT_EQ(buffer.getResults(), std::nullopt);
    buffer.addResults(createBatchResults({50, smc_storm::samples::TraceResult::NOT_VERIFIED, 0.0}), 1);
    buffer.addResults(createBatchResults({60, smc_storm::samples::TraceResult::NOT_VERIFIED, 0.0}), 1);
    buffer_results = buffer.getResults();
    EXPECT_NE(buffer_results, std::nullopt);
    EXPECT_EQ(buffer_results->size(), 2);
    testBatchResults(buffer_results->at(0), 0, 1, 0, 20);
    testBatchResults(buffer_results->at(1), 0, 1, 0, 50);
    buffer_results = buffer.getResults();
    EXPECT_NE(buffer_results, std::nullopt);
    EXPECT_EQ(buffer_results->size(), 2);
    testBatchResults(buffer_results->at(0), 0, 0, 1, 30);
    testBatchResults(buffer_results->at(1), 0, 1, 0, 60);
    buffer.addResults(createBatchResults({70, smc_storm::samples::TraceResult::VERIFIED, 0.0}), 0);
    buffer.addResults(createBatchResults({80, smc_storm::samples::TraceResult::NO_INFO, 0.0}), 1);
    buffer_results = buffer.getResults();
    EXPECT_NE(buffer_results, std::nullopt);
    EXPECT_EQ(buffer_results->size(), 2);
    testBatchResults(buffer_results->at(0), 1, 0, 0, 70);
    testBatchResults(buffer_results->at(1), 0, 0, 1, 80);
}

// Add more tests here if needed

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Initialize the default STORM settings (required for comparator)
    storm::settings::initializeAll("smc_storm", "test_batch_buffer");
    return RUN_ALL_TESTS();
}
