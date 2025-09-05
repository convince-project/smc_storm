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
#include <iostream>
#include <random>
#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <smc_verifiable_plugins/utils.hpp>
#include <stdexcept>

namespace smc_storm_plugins {
using smc_verifiable_plugins::DataExchange;

struct RobotPose {
    double x;
    double y;
    double theta;
};

inline std::string printRobotPose(const RobotPose& p) {
    std::stringstream s;
    s << "(" << p.x << ", " << p.y << ", " << p.theta << ")";
    return s.str();
}

/*!
 * @brief A plugin simulating a 2-D robot moving based on provided linear and angular velocity.
 */
class UnicycleRobotSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    UnicycleRobotSmcPlugin() = default;

    ~UnicycleRobotSmcPlugin() {
        if (_verbose) {
            std::cout << "Destructing the plugin " << getPluginName() << " instance.\n" << std::flush;
        }
    }

    std::string getPluginName() const override {
        return "uniform_random_smc_plugin";
    }

  private:
    /*!
     * @brief Load the Dice configuration: it consists of only one int telling the n. of faces
     * @param config The configuration to load: ints: [random_seed, n_faces], bool: [verbose (optional, false by default)]
     */
    void processInitParameters(const DataExchange& config) override {
        _verbose = false;
        if (config.contains("verbose")) {
            _verbose = std::get<bool>(config.at("verbose"));
        }
        _time_step = std::get<double>(config.at("time_step"));
        _start_pose.x = std::get<double>(config.at("start_x"));
        _start_pose.y = std::get<double>(config.at("start_y"));
        _start_pose.theta = std::get<double>(config.at("start_theta"));
        _start_pose.theta = fmod(_start_pose.theta, 2.0 * M_PI);
        if (_verbose) {
            std::cout << "[" << getPluginName() << "]: Init pose " << printRobotPose(_start_pose) << ".\n";
            std::cout << "[" << getPluginName() << "]: Time step " << _time_step << ".\n";
        }
    }

    /*!
     * @brief Reset the plugin to the initial state (move the robot back to its start pose)
     * @return The robot pose.
     */
    std::optional<DataExchange> processReset() override {
        _current_pose = _start_pose;
        if (_verbose) {
            std::cout << "[" << getPluginName() << "]: reset at pose " << printRobotPose(_current_pose) << std::endl;
        }
        return generateOutputData();
    }

    /*!
     * @brief Advances the plugin by one step by applying the provided linear and angular velocity!
     * @param input_data linear and angular velocity.
     * @return The robot pose after applying the requested motion.
     */
    std::optional<DataExchange> processInputParameters(const DataExchange& input_data) override {
        const double lin_vel = std::get<double>(input_data.at("lin_vel"));
        const double ang_vel = std::get<double>(input_data.at("ang_vel"));
        const double c = cos(_current_pose.theta);
        const double s = sin(_current_pose.theta);
        const double lin_dist = lin_vel * _time_step;
        const double ang_dist = ang_vel * _time_step;
        _current_pose.x += c * lin_dist;
        _current_pose.y += s * lin_dist;
        _current_pose.theta += ang_dist;
        _current_pose.theta = fmod(_current_pose.theta, 2.0 * M_PI);
        if (_verbose) {
            std::cout << "[" << getPluginName() << "]: drive with v, w (" << lin_vel << ", " << ang_vel << ")\n";
            std::cout << "[" << getPluginName() << "]: stepped to new pose " << printRobotPose(_current_pose) << std::endl;
        }
        return generateOutputData();
    }

    /*!
     * @brief Convenience function to generate the output DataExchange variable
     * @return The current state of the plugin, ready to be used by the model checker.
     */
    inline DataExchange generateOutputData() {
        return DataExchange({{"x", _current_pose.x}, {"y", _current_pose.y}, {"theta", _current_pose.theta}});
    }

    bool _verbose = false;
    double _time_step;
    struct RobotPose _start_pose;
    struct RobotPose _current_pose;
};

}  // namespace smc_storm_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(smc_storm_plugins::UnicycleRobotSmcPlugin);
}  // namespace smc_verifiable_plugins
