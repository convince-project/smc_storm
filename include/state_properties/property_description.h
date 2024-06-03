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

#pragma once

#include <limits>
#include <map>
#include <functional>
#include <optional>

#include <storm/logic/Formula.h>
#include <storm/logic/BooleanLiteralFormula.h>

namespace smc_storm::state_properties {

/*!
 * @brief Class defining an input property in terms of Condition and Target formula (plus some flags).
 */
class PropertyDescription {
public:
    using LabelsMap = std::map<std::string, storm::expressions::Expression>;
    /*!
     * @brief Input property under verification.
     * @param formula The property to verify as a storm Formula.
     */
    PropertyDescription(storm::logic::Formula const& formula);

    /*!
     * @brief Property already epxressed in terms of condition and target expressions, for compatibility
     * @param condition_expr An expression
     * @param target_expr An expression
     */
    PropertyDescription(storm::expressions::Expression const& condition_expr, storm::expressions::Expression const& target_expr);

    /*!
     * @brief Convert the  loaded formulae to Expressions, that can be evaluated.
     * @param manager Expression Manager
     * @param label_to_expression_mapping Mapping from variable names to actual expressions
     */
    void generateExpressions(storm::expressions::ExpressionManager const& manager, LabelsMap const& label_to_expression_mapping);

    // Various getters
    inline const storm::expressions::Expression& getConditionExpression() const
    {
        return _condition_expression;
    }

    inline const storm::expressions::Expression& getTargetExpression() const
    {
        return _target_expression;
    }

    bool getIsTerminalVerified() const
    {
        return _is_terminal_verified;
    }

    inline size_t getLowerBound() const
    {
        return _lower_bound_value;
    }

    inline size_t getUpperBound() const
    {
        return _upper_bound_value;
    }

private:
    // Helper vars
    const storm::logic::BooleanLiteralFormula TrueFormula = storm::logic::BooleanLiteralFormula(true);
    const storm::logic::BooleanLiteralFormula FalseFormula = storm::logic::BooleanLiteralFormula(false);

    // Private methods to precess formulae
    void processEventuallyFormula(storm::logic::EventuallyFormula const& formula);
    void processUntilFormula(storm::logic::UntilFormula const& formula);
    void processBoundedUntilFormula(storm::logic::BoundedUntilFormula const& formula);
    void processGloballyFormula(storm::logic::GloballyFormula const& formula);
    void processNextFormula(storm::logic::NextFormula const& formula);
    void processUnaryBooleanPathFormula(storm::logic::UnaryBooleanPathFormula const& formula);

    bool _is_terminal_verified = false;

    // Lower and upper bounds (Non strict)
    size_t _lower_bound_value = 0U;
    size_t _upper_bound_value = std::numeric_limits<size_t>::max();
    
    // Reference to the extracted formulae
    std::reference_wrapper<const storm::logic::Formula> _condition_formula_ref{FalseFormula};
    std::reference_wrapper<const storm::logic::Formula> _target_formula_ref{FalseFormula};
    // Flag telling wether we need to negate the expression at generation time.
    bool _negate_condition = false;
    bool _negate_target = false;

    // Converted expressions
    storm::expressions::Expression _condition_expression;
    storm::expressions::Expression _target_expression;

};
}  // namespace smc_storm::state_properties
