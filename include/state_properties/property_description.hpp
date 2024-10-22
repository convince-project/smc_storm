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

#include <functional>
#include <limits>
#include <map>
#include <optional>

#include <storm/logic/BooleanLiteralFormula.h>
#include <storm/logic/Formula.h>
#include <storm/storage/expressions/Expression.h>

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
    PropertyDescription(const storm::logic::Formula& formula);

    /*!
     * @brief Property already expressed in terms of condition and target expressions, for compatibility
     * @param condition_expr An expression
     * @param target_expr An expression
     */
    PropertyDescription(const storm::expressions::Expression& condition_expr, const storm::expressions::Expression& target_expr);

    /*!
     * @brief Convert the  loaded formulae to Expressions, that can be evaluated.
     * @param manager Expression Manager
     * @param label_to_expression_mapping Mapping from variable names to actual expressions
     */
    void generateExpressions(const storm::expressions::ExpressionManager& manager, const LabelsMap& label_to_expression_mapping);

    // Various getters
    inline const storm::expressions::Expression& getConditionExpression() const {
        return _condition_expression;
    }

    inline const storm::expressions::Expression& getTargetExpression() const {
        return _target_expression;
    }

    bool getIsTerminalVerified() const {
        return _is_terminal_verified;
    }

    inline size_t getLowerBound() const {
        return _lower_bound_value;
    }

    inline size_t getUpperBound() const {
        return _upper_bound_value;
    }

  private:
    // Helper vars
    const storm::logic::BooleanLiteralFormula _true_formula = storm::logic::BooleanLiteralFormula(true);
    const storm::logic::BooleanLiteralFormula _false_formula = storm::logic::BooleanLiteralFormula(false);

    // Private methods to precess formulae
    void processEventuallyFormula(const storm::logic::EventuallyFormula& formula);
    void processUntilFormula(const storm::logic::UntilFormula& formula);
    void processBoundedUntilFormula(const storm::logic::BoundedUntilFormula& formula);
    void processGloballyFormula(const storm::logic::GloballyFormula& formula);
    void processNextFormula(const storm::logic::NextFormula& formula);
    void processUnaryBooleanPathFormula(const storm::logic::UnaryBooleanPathFormula& formula);

    bool _is_terminal_verified = false;

    // Lower and upper bounds (Non strict)
    size_t _lower_bound_value = 0U;
    size_t _upper_bound_value = std::numeric_limits<size_t>::max();

    // Reference to the extracted formulae
    std::reference_wrapper<const storm::logic::Formula> _condition_formula_ref{_false_formula};
    std::reference_wrapper<const storm::logic::Formula> _target_formula_ref{_false_formula};
    // Flag telling wether we need to negate the expression at generation time.
    bool _negate_condition = false;
    bool _negate_target = false;

    // Converted expressions
    storm::expressions::Expression _condition_expression;
    storm::expressions::Expression _target_expression;
};
}  // namespace smc_storm::state_properties
