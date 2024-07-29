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

#include "model_checker/statistical_model_checker.hpp"
#include "parser/parsers.hpp"
#include "settings/user_settings.hpp"

const std::filesystem::path TEST_PATH{"test_files"};
const std::filesystem::path TEST_TRACES_PATH{"test_traces"};
constexpr char SEPARATOR = ';';
constexpr char FORBIDDEN_CHAR = ',';

smc_storm::settings::UserSettings getSmcSettings(
    const std::string& traces_file, const std::filesystem::path& jani_file, const std::string& property,
    const std::string& constants = "") {
    smc_storm::settings::UserSettings settings;
    settings.model_file = jani_file.string();
    settings.properties_names = property;
    settings.constants = constants;
    // Set Chernoff to default method for better stability in tests
    settings.stat_method = "chernoff";
    settings.max_n_traces = 10;
    settings.traces_file = (TEST_TRACES_PATH / traces_file).string();
    return settings;
}

template <typename ResultType>
ResultType getVerificationResult(const smc_storm::settings::UserSettings& settings) {
    const smc_storm::settings::SmcSettings smc_settings(settings);
    const auto model_and_properties = smc_storm::parser::parseModelAndProperty(settings);
    smc_storm::model_checker::StatisticalModelChecker smc(model_and_properties.model, model_and_properties.property[0], smc_settings);
    smc.check();
    return smc.getResult<ResultType>();
}

bool lineValid(const std::string& line, size_t expected_separators) {
    if (line.empty()) {
        return true;
    }
    const size_t n_separators = std::count(line.begin(), line.end(), SEPARATOR);
    const size_t n_forbidden = std::count(line.begin(), line.end(), FORBIDDEN_CHAR);
    return n_separators == expected_separators && n_forbidden == 0U;
}

TEST(ExportTracesCommonCaseTest, TestLeaderSync) {
    const std::filesystem::path jani_file = TEST_PATH / "leader_sync.3-2.v1.jani";
    const auto smc_settings = getSmcSettings("leader_sync.csv", jani_file, "time");
    const double result = getVerificationResult<double>(smc_settings);
    EXPECT_TRUE(std::filesystem::exists(smc_settings.traces_file));
    EXPECT_GT(result, FLT_EPSILON);
    std::ifstream traces_file(smc_settings.traces_file, std::ios::in);

    std::string line;
    // Use the first line to check the amount of semicolons in the header
    std::getline(traces_file, line);
    const size_t n_separators = std::count(line.begin(), line.end(), SEPARATOR);
    const size_t n_forbidden = std::count(line.begin(), line.end(), FORBIDDEN_CHAR);
    EXPECT_GT(n_separators, 0U);
    EXPECT_EQ(n_forbidden, 0U);
    while (true) {
        std::getline(traces_file, line);
        EXPECT_TRUE(lineValid(line, n_separators));
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
    const std::filesystem::path jani_file = TEST_PATH / "leader_sync.3-2.v1.jani";
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
    std::filesystem::remove_all(TEST_TRACES_PATH);
    std::filesystem::create_directory(TEST_TRACES_PATH);
    ::testing::InitGoogleTest(&argc, argv);
    (void)(::testing::GTEST_FLAG(death_test_style) = "threadsafe");
    return RUN_ALL_TESTS();
}
