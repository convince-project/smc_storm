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

#include <gtest/gtest.h>

#include <filesystem>

#include "settings/smc_settings.hpp"
#include "model_checker/statistical_model_checker.hpp"

const std::filesystem::path test_path{"test_files"};

smc_storm::settings::SmcSettings getSmcSettings(const std::filesystem::path& jani_file, const std::string& property, const std::string& constants = "") {
    smc_storm::settings::SmcSettings settings;
    settings.model = jani_file.string();
    settings.property_name = property;
    settings.constants = constants;
    // Set Chernoff to default method for better stability in tests
    settings.stat_method = "chernoff";
    return settings;
}

template <typename ResultType>
ResultType getVerificationResult(const smc_storm::settings::SmcSettings& settings) {
    smc_storm::model_checker::StatisticalModelChecker smc(settings);
    smc.check();
    return smc.getResult<ResultType>();
}

TEST(StatisticalModelCheckerTest, TestLeaderSync) {
    const std::filesystem::path jani_file = test_path / "leader_sync.3-2.v1.jani";
    const auto smc_settings = getSmcSettings(jani_file, "time");
    const double result = getVerificationResult<double>(smc_settings);
    EXPECT_NEAR(result, 1.3333333, smc_settings.epsilon);
}

TEST(StatisticalModelCheckerTest, TestNand) {
    const std::filesystem::path jani_file = test_path / "nand.v1.jani";
    const auto smc_settings = getSmcSettings(jani_file, "reliable", "N=20,K=2");
    const double result = getVerificationResult<double>(smc_settings);
    EXPECT_NEAR(result, 0.4128626, smc_settings.epsilon);
}

TEST(StatisticalModelCheckerTest, TestTrigonometry) {
    const std::filesystem::path jani_file = test_path / "trigonometry_test.jani";
    {
        const auto smc_settings = getSmcSettings(jani_file, "destination_reached_cos");
        const double result = getVerificationResult<double>(smc_settings);
        EXPECT_NEAR(result, 1.0, smc_settings.epsilon);
    }
    {
        const auto smc_settings = getSmcSettings(jani_file, "destination_reached_cos_bool");
        const double result = getVerificationResult<double>(smc_settings);
        EXPECT_NEAR(result, 1.0, smc_settings.epsilon);
    }
    {
        const auto smc_settings = getSmcSettings(jani_file, "destination_reached_sin");
        const double result = getVerificationResult<double>(smc_settings);
        EXPECT_NEAR(result, 1.0, smc_settings.epsilon);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
