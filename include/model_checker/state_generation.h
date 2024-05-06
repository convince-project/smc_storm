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

#include <memory>
#include <functional>

#include <storm/generator/CompressedState.h>
#include <storm/generator/NextStateGenerator.h>
#include <storm/storage/SymbolicModelDescription.h>
#include <storm/storage/sparse/StateStorage.h>

#include "properties/property_description.h"

namespace smc_storm {
namespace samples {
template<typename StateType, typename ValueType>
class ExplorationInformation;
}  // namespace samples

namespace model_checker {
template<typename StateType, typename ValueType>
class StateGeneration {
   public:
    /*!
     * @brief Base constructor for StateGeneration class
     * @param model The model to generate the next states from
     * @param formula The formula to evaluate on the generated states
     * @param explorationInformation Structure that keeps track of the already explored states
     * @param buildRewards Whether to embed the rewards in the model we are building up
     */
    StateGeneration(storm::storage::SymbolicModelDescription const& model, storm::logic::Formula const& formula,
                    samples::ExplorationInformation<StateType, ValueType>& explorationInformation,
                    bool const buildRewards = false);

    /*!
     * @brief Extension of the constructor above, in case we evaluate Rewards
     * @param model The model to generate the next states from
     * @param formula The formula to evaluate on the generated states
     * @param rewardModel Name of the reward model to use for assigning costs on states and actions
     * @param explorationInformation Structure that keeps track of the already explored states
     */
    StateGeneration(storm::storage::SymbolicModelDescription const& model, storm::logic::Formula const& formula,
                    std::string const& rewardModel, samples::ExplorationInformation<StateType, ValueType>& explorationInformation);

    /*!
     * @brief Check if a reward model is loaded and therefore it needs to be evaluated
     * @return true if we loaded a reward model, false otherwise 
     */
    inline bool rewardLoaded() const
    {
        return std::numeric_limits<size_t>::max() != rewardModelIndex;
    }

    /*!
     * @brief Get the index of the reward model of our interest
     * @return Index matching to the reward model we are evaluating
     */
    inline size_t getRewardIndex() const
    {
        return rewardModelIndex;
    }

    void load(storm::generator::CompressedState const& state);

    std::vector<StateType> getInitialStates();

    storm::generator::StateBehavior<ValueType, StateType> expand();

    void computeInitialStates();

    StateType getFirstInitialState() const;

    std::size_t getNumberOfInitialStates() const;

    // Check whether the loaded state satisfies the condition formula
    bool isConditionState() const;

    // Check whether the loaded state satisfies the target formula
    bool isTargetState() const;

    inline size_t getLowerBound() const
    {
        return propertyDescription.getLowerBound();
    }

    inline size_t getUpperBound() const
    {
        return propertyDescription.getUpperBound();
    }

    inline bool getIsTerminalVerified() const
    {
        return propertyDescription.getIsTerminalVerified();
    }

   private:
    void initStateToIdCallback(samples::ExplorationInformation<StateType, ValueType>& explorationInformation);

    std::unique_ptr<storm::generator::NextStateGenerator<ValueType, StateType>> generatorPtr;
    std::unique_ptr<storm::storage::sparse::StateStorage<StateType>> stateStoragePtr;

    properties::PropertyDescription propertyDescription;
    size_t rewardModelIndex = std::numeric_limits<size_t>::max();

    std::function<StateType(storm::generator::CompressedState const&)> stateToIdCallback;
};

}  // namespace model_checker
}  // namespace smc_storm
