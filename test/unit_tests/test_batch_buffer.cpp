#include <gtest/gtest.h>
#include "samples/batch_buffer.hpp"

TEST(BatchBufferTest, AddAndGetResults) {
    // Create a BatchBuffer with 2 threads and 3 slots
    smc_storm::samples::BatchBuffer buffer(2, 3);

    // Create some BatchResults
    smc_storm::samples::BatchResults results1;
    results1.addData(10);
    results1.addData(20);

    smc_storm::samples::BatchResults results2;
    results2.addData(30);
    results2.addData(40);

    // Add the BatchResults to the buffer
    buffer.addResults(results1, 0);
    buffer.addResults(results2, 1);

    // Get the results from the buffer
    std::optional<std::vector<smc_storm::samples::BatchResults>> results = buffer.getResults();

    // Check that the results are correct
    ASSERT_TRUE(results.has_value());
    ASSERT_EQ(results.value().size(), 2);

    ASSERT_EQ(results.value()[0].getData().size(), 2);
    ASSERT_EQ(results.value()[0].getData()[0], 10);
    ASSERT_EQ(results.value()[0].getData()[1], 20);

    ASSERT_EQ(results.value()[1].getData().size(), 2);
    ASSERT_EQ(results.value()[1].getData()[0], 30);
    ASSERT_EQ(results.value()[1].getData()[1], 40);
}

TEST(BatchBufferTest, WaitForSlotAvailable) {
    // Create a BatchBuffer with 2 threads and 3 slots
    smc_storm::samples::BatchBuffer buffer(2, 3);

    // Call waitForSlotAvailable on thread 0
    buffer.waitForSlotAvailable(0);

    // Call waitForSlotAvailable on thread 1
    buffer.waitForSlotAvailable(1);

    // No assertion is needed as this test is to verify that the function does not throw any exceptions
}

// Add more tests here if needed

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}