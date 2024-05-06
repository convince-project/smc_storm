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

#include <storm/storage/expressions/Expression.h>
#include <storm/storage/expressions/ExpressionManager.h>
#include <storm/logic/BooleanLiteralFormula.h>
#include <storm/logic/EventuallyFormula.h>
#include <storm/logic/UntilFormula.h>
#include <storm/logic/BoundedUntilFormula.h>
#include <storm/logic/GloballyFormula.h>
#include <storm/logic/NextFormula.h>
#include <storm/logic/UnaryBooleanPathFormula.h>
#include <storm/utility/macros.h>
#include <storm/exceptions/IllegalArgumentValueException.h>
#include <storm/exceptions/NotImplementedException.h>

#include "properties/property_description.h"

namespace smc_storm::properties {
PropertyDescription::PropertyDescription(storm::logic::Formula const& formula) {
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
        processUnaryBooleanPathFormula(dynamic_cast<storm::logic::UnaryBooleanPathFormula const&>(formula));
        return;
    }
    STORM_LOG_THROW(false, storm::exceptions::NotImplementedException, "The provided formula is not supported");
}

PropertyDescription::PropertyDescription(storm::expressions::Expression const& conditionExpr, storm::expressions::Expression const& targetExpr)
: _conditionExpression(conditionExpr), _targetExpression(targetExpr)
{
    // Supposed to be empty
}

void PropertyDescription::generateExpressions(storm::expressions::ExpressionManager const& manager, LabelsMap const& labelToExpressionMapping)
{
    _conditionExpression = _conditionFormulaRef.get().toExpression(manager, labelToExpressionMapping);
    _targetExpression = _targetFormulaRef.get().toExpression(manager, labelToExpressionMapping);
    if (_negateCondition) {
        _conditionExpression = !_conditionExpression;
    }
    if (_negateTarget) {
        _targetExpression = !_targetExpression;
    }
}

void PropertyDescription::processEventuallyFormula(storm::logic::EventuallyFormula const& formula) {
    _conditionFormulaRef = TrueFormula;
    _targetFormulaRef = formula.getSubformula();
}

void PropertyDescription::processUntilFormula(storm::logic::UntilFormula const& formula) {
    _conditionFormulaRef = formula.getLeftSubformula();
    _targetFormulaRef = formula.getRightSubformula();
}

void PropertyDescription::processBoundedUntilFormula(storm::logic::BoundedUntilFormula const& formula)
{
    STORM_LOG_THROW(!formula.isMultiDimensional(), storm::exceptions::NotImplementedException, "We support only bounds in one dimension.");
    _conditionFormulaRef = formula.getLeftSubformula();
    _targetFormulaRef = formula.getRightSubformula();
    // TODO: Get the actual bounds
    if (formula.hasLowerBound()) {
        _lowerBoundValue = formula.getNonStrictLowerBound<size_t>();
    }
    if (formula.hasUpperBound()) {
        _upperBoundValue = formula.getNonStrictUpperBound<size_t>();
    }
}

void PropertyDescription::processGloballyFormula(storm::logic::GloballyFormula const& formula) {
    // For global formulae we want to ensure we reach a terminal state without breaking our condition
    _isTerminalVerified = true;
    _conditionFormulaRef = formula.getSubformula();
    _targetFormulaRef = FalseFormula;
}

void PropertyDescription::processNextFormula(storm::logic::NextFormula const& formula) {
    _conditionFormulaRef = TrueFormula;
    _targetFormulaRef = formula.getSubformula();
    // neXt is basically a Final with two bounds to make sure we check EXACTLY the second element on the path!
    _lowerBoundValue = 1U;
    _upperBoundValue = 1U;
}

void PropertyDescription::processUnaryBooleanPathFormula(storm::logic::UnaryBooleanPathFormula const& formula) {
    // This would be basically a negate formula...
    STORM_LOG_THROW(formula.isNot(), storm::exceptions::IllegalArgumentValueException, "Expected the Unary Boolean formula to be a NOT.");
    const auto& subformula = formula.getSubformula();
    if (subformula.isNextFormula()) {
        processNextFormula(subformula.asNextFormula());
        _negateTarget  = true;
        return;
    }
    if (subformula.isEventuallyFormula()) {
        // Eventually formulae can be negated converting them to Global formulae
        // TODO: Unify this block with the processGloballyFormula method
        _isTerminalVerified = true;
        _targetFormulaRef = FalseFormula;
        _conditionFormulaRef = subformula.asEventuallyFormula().getSubformula();
        _negateCondition = true;
        return;
    }
    if (subformula.isGloballyFormula()) {
        // Globally formulae can be negated converting them to Eventually formulae
        // TODO: Unify this block with the processEventuallyFormula method
        _targetFormulaRef = subformula.asGloballyFormula().getSubformula();
        _conditionFormulaRef = TrueFormula;
        _negateTarget = true;
        return;
    }
    // TODO: Bounded Eventually Formulae
    STORM_LOG_THROW(false, storm::exceptions::NotImplementedException, "The provided negated input formula is not supported yet!");
}

}  // namespace smc_storm::properties
