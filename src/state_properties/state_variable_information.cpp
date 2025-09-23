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

#include "state_properties/state_variable_information.hpp"
#include <storm/exceptions/InternalException.h>
#include <storm/storage/expressions/ExpressionEvaluator.h>
#include <storm/storage/jani/Model.h>

#include <algorithm>
#include <functional>
#include <optional>
#include <vector>

namespace smc_storm::state_properties {

template <typename RealValueType>
StateVariableInformation<RealValueType>::StateVariableInformation(
    const storm::jani::Model& model, const std::vector<std::reference_wrapper<const storm::jani::Automaton>>& parallel_automata,
    bool out_of_bounds_state)
    : _check_bounds{out_of_bounds_state} {
    processVariableSet(model.getGlobalVariables(), true);
    for (const auto& automaton_ref : parallel_automata) {
        const auto& automaton = automaton_ref.get();
        processVariableSet(automaton.getVariables(), false);
        _location_data.emplace_back(automaton.getLocationExpressionVariable(), false, 0U, automaton.getNumberOfLocations() - 1);
    }
    sortVariableVector(_bool_data);
    sortVariableVector(_int_data);
    sortVariableVector(_real_data);
    sortVariableVector(_location_data);
}

template <typename RealValueType>
void StateVariableInformation<RealValueType>::processVariableSet(const storm::jani::VariableSet& vars, const bool is_global) {
    STORM_LOG_THROW(!vars.containsArrayVariables(), storm::exceptions::InternalException, "Found array var. Unexpected here!");
    // Function to load the variable information
    auto var_processor = [this, is_global](const storm::jani::detail::ConstVariables<storm::jani::Variable>& var_set, auto& target_vector) {
        for (const auto& variable : var_set) {
            if (variable.isTransient()) {
                continue;
            }
            if (variable.getType().isBoundedType()) {
                const auto& bounded_variable = variable.getType().asBoundedType();
                typedef typename std::remove_reference_t<decltype((target_vector))>::value_type::value_type var_type;
                std::optional<var_type> lower_bound = std::nullopt;
                std::optional<var_type> upper_bound = std::nullopt;
                if (bounded_variable.hasLowerBound()) {
                    lower_bound = evaluateExpression<var_type>(bounded_variable.getLowerBound());
                }
                if (bounded_variable.hasUpperBound()) {
                    upper_bound = evaluateExpression<var_type>(bounded_variable.getUpperBound());
                }
                target_vector.emplace_back(variable.getExpressionVariable(), is_global, lower_bound, upper_bound);
            } else {
                target_vector.emplace_back(variable.getExpressionVariable(), is_global);
            }
        }
    };
    var_processor(vars.getBooleanVariables(), _bool_data);
    var_processor(vars.getUnboundedIntegerVariables(), _int_data);
    var_processor(vars.getBoundedIntegerVariables(), _int_data);
    var_processor(vars.getRealVariables(), _real_data);
}

template <typename RealValueType>
void StateVariableInformation<RealValueType>::setInEvaluator(
    storm::expressions::ExpressionEvaluator<RealValueType>& evaluator, const StateVariableData<RealValueType>& var_data) const {
    // Location data
    for (size_t loc_idx = 0U; loc_idx < _location_data.size(); loc_idx++) {
        const auto& loc_info = _location_data.at(loc_idx);
        const auto& loc_value = var_data.getLocationData().at(loc_idx);
        evaluator.setIntegerValue(loc_info.variable, loc_value);
    }
    // Bool data
    for (size_t bool_idx = 0U; bool_idx < _bool_data.size(); bool_idx++) {
        const auto& bool_info = _bool_data.at(bool_idx);
        const auto& bool_value = var_data.getBoolData().at(bool_idx);
        evaluator.setBooleanValue(bool_info.variable, bool_value);
    }
    // Integer data
    for (size_t int_idx = 0U; int_idx < _int_data.size(); int_idx++) {
        const auto& int_info = _int_data.at(int_idx);
        const auto& int_value = var_data.getIntData().at(int_idx);
        evaluator.setIntegerValue(int_info.variable, int_value);
    }
    // Real data
    for (size_t real_idx = 0U; real_idx < _real_data.size(); real_idx++) {
        const auto& real_info = _real_data.at(real_idx);
        const auto& real_value = var_data.getRealData().at(real_idx);
        evaluator.setRationalValue(real_info.variable, real_value);
    }
}

template class StateVariableInformation<double>;

}  // namespace smc_storm::state_properties
