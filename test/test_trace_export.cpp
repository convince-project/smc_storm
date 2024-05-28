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

#include <storm/exceptions/NotSupportedException.h>

#include "settings/smc_settings.hpp"
#include "model_checker/statistical_model_checker.hpp"

const std::filesystem::path test_path{"test_files"};
const std::filesystem::path test_traces_path{"test_traces"};

smc_storm::settings::SmcSettings getSmcSettings(const std::string& traces_file, const std::filesystem::path& jani_file, const std::string& property, const std::string& constants = "") {
    smc_storm::settings::SmcSettings settings;
    settings.model = jani_file.string();
    settings.property_name = property;
    settings.constants = constants;
    // Set Chernoff to default method for better stability in tests
    settings.stat_method = "chernoff";
    settings.max_n_traces = 10;
    settings.traces_file = (test_traces_path / traces_file).string();
    return settings;
}

template <typename ResultType>
ResultType getVerificationResult(const smc_storm::settings::SmcSettings& settings) {
    smc_storm::model_checker::StatisticalModelChecker smc(settings);
    smc.check();
    return smc.getResult<ResultType>();
}

TEST(ExportTracesCommonCaseTest, TestLeaderSync) {
    const std::filesystem::path jani_file = test_path / "leader_sync.3-2.v1.jani";
    const auto smc_settings = getSmcSettings("leader_sync.csv", jani_file, "time");
    const double result = getVerificationResult<double>(smc_settings);
    EXPECT_TRUE(std::filesystem::exists(smc_settings.traces_file));
    EXPECT_GT(result, FLT_EPSILON);
    std::ifstream traces_file(smc_settings.traces_file, std::ios::in);

    std::string line;
    while (true) {
        std::getline(traces_file, line);
        if (traces_file.peek() != EOF) {
            traces_file.seekg(1, traces_file.cur);
            if (traces_file.peek() == EOF) {
                break;
            }
            traces_file.seekg(-1, traces_file.cur);
        }
    }
    EXPECT_EQ(line[0], '9');
    traces_file.close();
}

TEST(ExportTracesDeathTest, TestLeaderSync) {
    const std::filesystem::path jani_file = test_path / "leader_sync.3-2.v1.jani";
    const auto smc_settings = getSmcSettings("leader_sync_death.csv", jani_file, "time");
    // Make sure the file already exists
    std::ofstream ofs(smc_settings.traces_file);
    ofs << "\n"; 
    ofs.close();
    // Ensure the code dies in such case
    EXPECT_DEATH(getVerificationResult<double>(smc_settings), "");
}

int main(int argc, char** argv) {
    // Make sure the initial folder status is as expected
    std::filesystem::remove_all(test_traces_path);
    std::filesystem::create_directory(test_traces_path);
    ::testing::InitGoogleTest(&argc, argv);
    (void)(::testing::GTEST_FLAG(death_test_style) = "threadsafe");
    return RUN_ALL_TESTS();
}
