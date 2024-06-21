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

#include <storm/api/builder.h>
#include <storm/api/storm.h>
#include <storm/exceptions/InvalidPropertyException.h>
#include <storm/exceptions/UnexpectedException.h>
#include <storm/modelchecker/CheckTask.h>
#include <storm/modelchecker/results/CheckResult.h>
#include <storm/modelchecker/results/ExplicitQualitativeCheckResult.h>
#include <storm/modelchecker/results/ExplicitQuantitativeCheckResult.h>
#include <storm/models/ModelType.h>
#include <storm/settings/modules/GeneralSettings.h>
#include <storm/settings/SettingsManager.h>
#include <storm/utility/initialize.h>
#include <storm/utility/macros.h>

#include <storm/models/sparse/Dtmc.h>
#include <storm/models/sparse/Mdp.h>

#include <storm-parsers/api/storm-parsers.h>

#include "model_checker/statistical_model_checker.hpp"
#include "model_checker/statistical_model_checking_engine.hpp"
#include "parser/parsers.hpp"

namespace smc_storm::model_checker {
StatisticalModelChecker::StatisticalModelChecker(const settings::SmcSettings& settings) : _settings(settings) {
    // Init loggers
    storm::utility::setUp();
    // Check if settings have been already initialized
    if (!storm::settings::hasModule<storm::settings::modules::GeneralSettings>()) {
        // Set some settings objects.
        storm::settings::initializeAll("smc_storm", "smc_storm");
    }
    const auto model_and_properties = parser::parseModelAndProperty(settings.model, settings.property_name, settings.constants);
    STORM_LOG_THROW(model_and_properties.property.size() == 1, storm::exceptions::UnexpectedException, "Only one property is expected.");
    _model = model_and_properties.model;
    _property = model_and_properties.property;
}

StatisticalModelChecker::~StatisticalModelChecker() = default;

void StatisticalModelChecker::printProperty() const {
    STORM_PRINT("Property " << _property.front().asPrismSyntax() << "\n");
}

void StatisticalModelChecker::check() {
    STORM_PRINT("CONVINCE Statistical Model Checker\n");
    STORM_PRINT("Checking model: " << _settings.model << std::endl);
    const storm::modelchecker::CheckTask<> check_task(*(_property.front().getRawFormula()), true);
    if (_model.getModelType() == storm::storage::SymbolicModelDescription::ModelType::DTMC) {
        StatisticalModelCheckingEngine<storm::models::sparse::Dtmc<double>> checker(_model, _settings);
        _result = checker.check(check_task);
    } else if (_model.getModelType() == storm::storage::SymbolicModelDescription::ModelType::MDP) {
        StatisticalModelCheckingEngine<storm::models::sparse::Mdp<double>> checker(_model, _settings);
        _result = checker.check(check_task);
    } else {
        STORM_LOG_THROW(false, storm::exceptions::NotSupportedException, "Only DTMC and MDP models are supported.");
    }
    STORM_PRINT("Result: " << *_result << std::endl);
}

template <>
double StatisticalModelChecker::getResult() const {
    STORM_LOG_THROW(
        _result->isExplicitQuantitativeCheckResult(), storm::exceptions::UnexpectedException,
        "Numeric results must be explicit quantitative ones.");
    return _result->asExplicitQuantitativeCheckResult<double>().getMin();
}

template <>
bool StatisticalModelChecker::getResult() const {
    STORM_LOG_THROW(
        _result->isExplicitQualitativeCheckResult(), storm::exceptions::UnexpectedException,
        "Boolean results must be explicit qualitative ones.");
    return _result->asExplicitQualitativeCheckResult().forallTrue();
}

}  // namespace smc_storm::model_checker
