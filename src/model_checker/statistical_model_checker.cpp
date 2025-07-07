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
#include <storm/utility/macros.h>

#include <storm/models/sparse/Dtmc.h>
#include <storm/models/sparse/Mdp.h>

#include <storm-parsers/api/storm-parsers.h>

#include "model_checker/statistical_model_checker.hpp"
#include "model_checker/statistical_model_checking_engine.hpp"
#include "utils/storm_utilities.hpp"

namespace smc_storm::model_checker {
StatisticalModelChecker::StatisticalModelChecker(
    const storm::storage::SymbolicModelDescription& model, const storm::jani::Property& property, const settings::SmcSettings& settings,
    const std::vector<SmcPluginInstance>& loaded_plugins)
    : _settings{settings}, _model{model}, _property{property}, _loaded_plugins{loaded_plugins} {
    utils::stormSetUp();
}

StatisticalModelChecker::~StatisticalModelChecker() = default;

void StatisticalModelChecker::printProperty() const {
    std::cout << "Property " << _property.get().asPrismSyntax() << std::endl << std::flush;
}

void StatisticalModelChecker::check() {
    const storm::modelchecker::CheckTask<> check_task(*(_property.get().getRawFormula()), true);
    const bool check_explored_states = _settings.get().cache_explored_states;
    if (_settings.get().cache_explored_states) {
        if (_model.get().getModelType() == storm::storage::SymbolicModelDescription::ModelType::DTMC) {
            StatisticalModelCheckingEngine<storm::models::sparse::Dtmc<double>, true> checker(
                _model.get(), _settings.get(), _loaded_plugins.get());
            _result = checker.check(check_task);
        } else if (_model.get().getModelType() == storm::storage::SymbolicModelDescription::ModelType::MDP) {
            StatisticalModelCheckingEngine<storm::models::sparse::Mdp<double>, true> checker(
                _model.get(), _settings.get(), _loaded_plugins.get());
            _result = checker.check(check_task);
        } else {
            STORM_LOG_THROW(false, storm::exceptions::NotSupportedException, "Only DTMC and MDP models are supported.");
        }
    } else {
        if (_model.get().getModelType() == storm::storage::SymbolicModelDescription::ModelType::DTMC) {
            StatisticalModelCheckingEngine<storm::models::sparse::Dtmc<double>, false> checker(
                _model.get(), _settings.get(), _loaded_plugins.get());
            _result = checker.check(check_task);
        } else if (_model.get().getModelType() == storm::storage::SymbolicModelDescription::ModelType::MDP) {
            StatisticalModelCheckingEngine<storm::models::sparse::Mdp<double>, false> checker(
                _model.get(), _settings.get(), _loaded_plugins.get());
            _result = checker.check(check_task);
        } else {
            STORM_LOG_THROW(false, storm::exceptions::NotSupportedException, "Only DTMC and MDP models are supported.");
        }
    }
    std::cout << "Result: " << *_result << std::endl;
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
