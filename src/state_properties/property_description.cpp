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

#include <storm/exceptions/IllegalArgumentValueException.h>
#include <storm/exceptions/NotImplementedException.h>
// clang-format off
// Don't reorder those includes, due to storm includes
#include <storm/storage/expressions/Expression.h>
#include <storm/storage/expressions/ExpressionManager.h>
// clang-format on
#include <storm/logic/BooleanLiteralFormula.h>
#include <storm/logic/BoundedUntilFormula.h>
#include <storm/logic/EventuallyFormula.h>
#include <storm/logic/GloballyFormula.h>
#include <storm/logic/NextFormula.h>
#include <storm/logic/UnaryBooleanPathFormula.h>
#include <storm/logic/UntilFormula.h>
#include <storm/utility/macros.h>

#include "state_properties/property_description.hpp"

namespace smc_storm::state_properties {
PropertyDescription::PropertyDescription(const storm::logic::Formula& formula) {
    // For now we support only a limited set of simpler formulas
    if (formula.isBoundedUntilFormula()) {
        processBoundedUntilFormula(formula.asBoundedUntilFormula());
        return;
    }
    if (formula.isEventuallyFormula()) {
        processEventuallyFormula(formula.asEventuallyFormula());
        return;
    }
    if (formula.isUntilFormula()) {
        processUntilFormula(formula.asUntilFormula());
        return;
    }
    if (formula.isGloballyFormula()) {
        processGloballyFormula(formula.asGloballyFormula());
        return;
    }
    if (formula.isNextFormula()) {
        processNextFormula(formula.asNextFormula());
        return;
    }
    if (formula.isUnaryBooleanPathFormula()) {
        processUnaryBooleanPathFormula(dynamic_cast<const storm::logic::UnaryBooleanPathFormula&>(formula));
        return;
    }
    STORM_LOG_THROW(false, storm::exceptions::NotImplementedException, "The provided formula is not supported");
}

PropertyDescription::PropertyDescription(
    const storm::expressions::Expression& condition_expr, const storm::expressions::Expression& target_expr)
    : _condition_expression(condition_expr), _target_expression(target_expr) {
    // Supposed to be empty
}

void PropertyDescription::generateExpressions(
    const storm::expressions::ExpressionManager& manager, const LabelsMap& label_to_expression_mapping) {
    _condition_expression = _condition_formula_ref.get().toExpression(manager, label_to_expression_mapping);
    _target_expression = _target_formula_ref.get().toExpression(manager, label_to_expression_mapping);
    if (_negate_condition) {
        _condition_expression = !_condition_expression;
    }
    if (_negate_target) {
        _target_expression = !_target_expression;
    }
}

void PropertyDescription::processEventuallyFormula(const storm::logic::EventuallyFormula& formula) {
    _condition_formula_ref = TrueFormula;
    _target_formula_ref = formula.getSubformula();
}

void PropertyDescription::processUntilFormula(const storm::logic::UntilFormula& formula) {
    _condition_formula_ref = formula.getLeftSubformula();
    _target_formula_ref = formula.getRightSubformula();
}

void PropertyDescription::processBoundedUntilFormula(const storm::logic::BoundedUntilFormula& formula) {
    STORM_LOG_THROW(!formula.isMultiDimensional(), storm::exceptions::NotImplementedException, "We support only bounds in one dimension.");
    _condition_formula_ref = formula.getLeftSubformula();
    _target_formula_ref = formula.getRightSubformula();
    // TODO: Get the actual bounds
    if (formula.hasLowerBound()) {
        _lower_bound_value = formula.getNonStrictLowerBound<size_t>();
    }
    if (formula.hasUpperBound()) {
        _upper_bound_value = formula.getNonStrictUpperBound<size_t>();
    }
}

void PropertyDescription::processGloballyFormula(const storm::logic::GloballyFormula& formula) {
    // For global formulae we want to ensure we reach a terminal state without breaking our condition
    _is_terminal_verified = true;
    _condition_formula_ref = formula.getSubformula();
    _target_formula_ref = FalseFormula;
}

void PropertyDescription::processNextFormula(const storm::logic::NextFormula& formula) {
    _condition_formula_ref = TrueFormula;
    _target_formula_ref = formula.getSubformula();
    // neXt is basically a Final with two bounds to make sure we check EXACTLY the second element on the path!
    _lower_bound_value = 1U;
    _upper_bound_value = 1U;
}

void PropertyDescription::processUnaryBooleanPathFormula(const storm::logic::UnaryBooleanPathFormula& formula) {
    // This would be basically a negate formula...
    STORM_LOG_THROW(formula.isNot(), storm::exceptions::IllegalArgumentValueException, "Expected the Unary Boolean formula to be a NOT.");
    const auto& subformula = formula.getSubformula();
    if (subformula.isNextFormula()) {
        processNextFormula(subformula.asNextFormula());
        _negate_target = true;
        return;
    }
    if (subformula.isEventuallyFormula()) {
        // Eventually formulae can be negated converting them to Global formulae
        // TODO: Unify this block with the processGloballyFormula method
        _is_terminal_verified = true;
        _target_formula_ref = FalseFormula;
        _condition_formula_ref = subformula.asEventuallyFormula().getSubformula();
        _negate_condition = true;
        return;
    }
    if (subformula.isGloballyFormula()) {
        // Globally formulae can be negated converting them to Eventually formulae
        // TODO: Unify this block with the processEventuallyFormula method
        _target_formula_ref = subformula.asGloballyFormula().getSubformula();
        _condition_formula_ref = TrueFormula;
        _negate_target = true;
        return;
    }
    // TODO: Bounded Eventually Formulae
    STORM_LOG_THROW(false, storm::exceptions::NotImplementedException, "The provided negated input formula is not supported yet!");
}

}  // namespace smc_storm::state_properties
