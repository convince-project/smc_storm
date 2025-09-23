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

#pragma once

#include <storm/exceptions/InternalException.h>
#include <storm/storage/jani/Model.h>

#include <functional>
#include <optional>
#include <vector>

namespace storm::expressions {
template <typename ValueType>
class ExpressionEvaluator;
}  // namespace storm::expressions

namespace smc_storm::state_properties {

template <typename ExprType>
ExprType evaluateExpression(const storm::expressions::Expression& expr) {
    if constexpr (std::is_same_v<ExprType, int64_t>) {
        return expr.evaluateAsInt();
    } else if constexpr (std::is_same_v<ExprType, double>) {
        return expr.evaluateAsDouble();
    } else {
        STORM_LOG_THROW(false, storm::exceptions::InternalException, "Unexpected function call!");
    }
}
/*!
 * @brief Class holding the current value of each non-transient variable in the model.
 * @tparam RealValueType The type associated to a real value. Normally it is simply a double.
 */
template <typename RealValueType>
class StateVariableData {
  public:
    StateVariableData() = default;

    StateVariableData(const size_t bool_count, const size_t int_count, const size_t real_count, const size_t locations_count)
        : _bool_data{std::vector<bool>(bool_count)}, _int_data{std::vector<int64_t>(int_count)},
          _real_data{std::vector<RealValueType>(real_count)}, _location_data{std::vector<uint64_t>(locations_count)} {}

    StateVariableData(const StateVariableData<RealValueType>& other)
        : _bool_data{other._bool_data}, _int_data{other._int_data}, _real_data{other._real_data}, _location_data{other._location_data} {}

    inline bool empty() const {
        return _bool_data.empty() && _int_data.empty() && _real_data.empty() && _location_data.empty();
    }

    inline StateVariableData& operator=(const StateVariableData<RealValueType>& other) {
        _bool_data = other._bool_data;
        _int_data = other._int_data;
        _real_data = other._real_data;
        _location_data = other._location_data;
        return *this;
    }

    inline bool operator==(const StateVariableData<RealValueType>& other) const {
        return _location_data == other._location_data && _bool_data == other._bool_data && _int_data == other._int_data &&
               _real_data == other._real_data;
    }

    inline void setBool(const size_t bool_idx, bool value) {
        _bool_data.at(bool_idx) = value;
    }

    inline void setInt(const size_t int_idx, int64_t value) {
        _int_data.at(int_idx) = value;
    }

    inline void setReal(const size_t real_idx, RealValueType value) {
        _real_data.at(real_idx) = value;
    }

    inline void setLocation(const size_t location_idx, uint64_t value) {
        _location_data.at(location_idx) = value;
    }

    inline const std::vector<bool>& getBoolData() const {
        return _bool_data;
    }

    inline const std::vector<int64_t>& getIntData() const {
        return _int_data;
    }

    inline const std::vector<RealValueType>& getRealData() const {
        return _real_data;
    }

    inline const std::vector<uint64_t>& getLocationData() const {
        return _location_data;
    }

  private:
    std::vector<bool> _bool_data;
    std::vector<int64_t> _int_data;
    std::vector<RealValueType> _real_data;
    std::vector<uint64_t> _location_data;
};

template <typename VariableType>
struct VariableInformation {
    /*!
     * @brief Make the information on the type of stored variable accessible.
     */
    using value_type = VariableType;

    VariableInformation(
        const storm::expressions::Variable& in_variable, bool is_global, const std::optional<VariableType>& lower_b = std::nullopt,
        const std::optional<VariableType>& upper_b = std::nullopt)
        : variable(in_variable), lower_bound{lower_b}, upper_bound{upper_b}, global{is_global} {}
    // The integer variable.
    storm::expressions::Variable variable;

    // The lower bound of its range.
    std::optional<VariableType> lower_bound;

    // The upper bound of its range.
    std::optional<VariableType> upper_bound;

    // A flag indicating whether the variable is a global one.
    bool global;
};

template <typename RealValueType>
class StateVariableInformation {
  public:
    StateVariableInformation() = default;
    StateVariableInformation(
        const storm::jani::Model& model, const std::vector<std::reference_wrapper<const storm::jani::Automaton>>& parallel_automata,
        bool out_of_bounds_state);

    inline bool checkVariableBounds() const {
        return _check_bounds;
    }

    inline StateVariableData<RealValueType> generateVariableData() const {
        return StateVariableData<RealValueType>(_bool_data.size(), _int_data.size(), _real_data.size(), _location_data.size());
    }

    inline const std::vector<VariableInformation<bool>>& booleanVariables() const {
        return _bool_data;
    }

    inline const std::vector<VariableInformation<int64_t>>& integerVariables() const {
        return _int_data;
    }

    inline const std::vector<VariableInformation<RealValueType>>& realVariables() const {
        return _real_data;
    }

    inline const std::vector<VariableInformation<uint64_t>>& locationVariables() const {
        return _location_data;
    }

    void setInEvaluator(
        storm::expressions::ExpressionEvaluator<RealValueType>& evaluator, const StateVariableData<RealValueType>& var_data) const;

  private:
    void processVariableSet(const storm::jani::VariableSet& vars, const bool is_global);

    template <typename VarType>
    static void sortVariableVector(std::vector<VariableInformation<VarType>>& var_vector) {
        std::sort(
            var_vector.begin(), var_vector.end(),
            [](const VariableInformation<VarType>& var_entry_l, const VariableInformation<VarType>& var_entry_r) {
                return var_entry_l.variable < var_entry_r.variable;
            });
    }
    std::vector<VariableInformation<bool>> _bool_data;
    std::vector<VariableInformation<int64_t>> _int_data;
    std::vector<VariableInformation<RealValueType>> _real_data;
    std::vector<VariableInformation<uint64_t>> _location_data;
    bool _check_bounds = false;
};
}  // namespace smc_storm::state_properties
