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
#include <regex>

#include <storm/exceptions/NotSupportedException.h>

#include "model_checker/statistical_model_checker.hpp"
#include "parser/parsers.hpp"
#include "settings/user_settings.hpp"

const std::filesystem::path TEST_PATH{"models"};
const std::filesystem::path TEST_TRACES_PATH{"test_traces"};
constexpr char SEPARATOR = ';';
constexpr char FORBIDDEN_CHAR = ',';

smc_storm::settings::UserSettings getSmcSettings(
    const std::string& traces_folder, const std::filesystem::path& jani_file, const std::string& property,
    const std::string& constants = "", const bool add_date = false) {
    smc_storm::settings::UserSettings settings;
    settings.model_file = jani_file.string();
    settings.properties_names = property;
    settings.constants = constants;
    // Set Chernoff to default method for better stability in tests
    settings.stat_method = "chernoff";
    settings.max_n_traces = 10;
    settings.traces_folder = (TEST_TRACES_PATH / traces_folder).string();
    settings.traces_add_date = add_date;
    return settings;
}

template <typename ResultType>
ResultType getVerificationResult(const smc_storm::settings::UserSettings& settings) {
    const smc_storm::settings::SmcSettings smc_settings(settings);
    const auto model_and_properties = smc_storm::parser::parseModelAndProperties(settings);
    smc_storm::model_checker::StatisticalModelChecker smc(model_and_properties.model, model_and_properties.properties[0], smc_settings);
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
    const auto smc_settings = getSmcSettings("leader_sync", jani_file, "time");
    const double result = getVerificationResult<double>(smc_settings);
    EXPECT_TRUE(std::filesystem::exists(smc_settings.traces_folder));
    EXPECT_GT(result, FLT_EPSILON);
    size_t n_files = 0;
    for (const auto& single_trace : std::filesystem::directory_iterator(smc_settings.traces_folder)) {
        n_files++;
        std::ifstream traces_file(single_trace.path(), std::ios::in);
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
            if (traces_file.peek() == EOF) {
                break;
            }
        }
        traces_file.close();
    }
    EXPECT_EQ(n_files, 10u);
}

TEST(ExportTracesDeathTest, TestLeaderSync) {
    const std::filesystem::path jani_file = TEST_PATH / "leader_sync.3-2.v1.jani";
    const auto smc_settings = getSmcSettings("leader_sync_death", jani_file, "time", "");
    // Make sure the file already exists
    std::filesystem::create_directory(smc_settings.traces_folder);
    // Ensure the code dies in such case
    ASSERT_THROW(getVerificationResult<double>(smc_settings), storm::exceptions::FileIoException);
}

TEST(ExportTracesWithDate, TestLeaderSync) {
    const std::string trace_dir_name = "folder_w_date";
    const std::filesystem::path jani_file = TEST_PATH / "leader_sync.3-2.v1.jani";
    const auto smc_settings = getSmcSettings(trace_dir_name, jani_file, "time", "", true);
    const double result = getVerificationResult<double>(smc_settings);
    bool dir_found = false;
    for (const auto& folder_name : std::filesystem::directory_iterator(TEST_TRACES_PATH)) {
        if (folder_name.is_directory()) {
            std::string name_str = folder_name.path().filename().string();
            if (name_str.starts_with(trace_dir_name)) {
                std::regex date_regex(R"(^.+_\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$)");
                dir_found = std::regex_match(name_str, date_regex);
                break;
            }
        }
    }
    EXPECT_TRUE(dir_found);
}

int main(int argc, char** argv) {
    // Make sure the initial folder status is as expected
    std::filesystem::remove_all(TEST_TRACES_PATH);
    std::filesystem::create_directory(TEST_TRACES_PATH);
    ::testing::InitGoogleTest(&argc, argv);
    (void)(::testing::GTEST_FLAG(death_test_style) = "threadsafe");
    return RUN_ALL_TESTS();
}
