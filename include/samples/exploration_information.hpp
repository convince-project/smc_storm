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
#include <optional>
#include <unordered_map>
#include <vector>

#include <storm/solver/OptimizationDirection.h>

#include <storm/generator/CompressedState.h>

#include <storm/storage/BoostTypes.h>
#include <storm/storage/SparseMatrix.h>

#include "state_properties/state_description.hpp"
#include "state_properties/state_info.hpp"

namespace smc_storm::samples {
/*!
 * @brief Class holding information about the explored states
 * @tparam StateType variable type for state and action identifiers
 * @tparam ValueType variable type for the results (e.g. Rewards)
 */
template <typename StateType, typename ValueType>
class ExplorationInformation {
  public:
    typedef std::unordered_map<StateType, storm::generator::CompressedState> IdToStateMap;
    typedef typename IdToStateMap::const_iterator const_iterator;

    /*!
     * @brief Constructor for the ExplorationInformation class
     * @param store_rewards If true, the class will store the computed reward
     * @param store_expanded_states If true, the class will store all expanded raw states in memory
     */
    ExplorationInformation();

    /*!
     * @brief Return the state description associated to the input state id
     * @param state_id The ID of the requested state
     * @return The state description associated to the input state id
     */
    const state_properties::StateDescription<StateType, ValueType>& getStateDescription(const StateType& state_id) const;

    /*!
     * @brief Add a state description to the explored states
     * @param state_id The ID of the state to add
     * @param state_description_ptr The state description to add (as a unique_ptr reference)
     */
    void addStateDescription(
        const StateType& state_id, std::unique_ptr<state_properties::StateDescription<StateType, ValueType>> state_description_ptr);

    const_iterator findUnexploredState(const StateType& state) const;

    const_iterator unexploredStatesEnd() const;

    void removeUnexploredState(const_iterator it);

    void addUnexploredState(const StateType& state_id, const storm::generator::CompressedState& compressed_state);

    inline bool isUnexplored(const StateType& state_id) {
        return _state_to_vect_idx.at(state_id) == UNEXPLORED_MARKER;
    }

  private:
    // The value used to mark unexplored states
    static constexpr StateType UNEXPLORED_MARKER{std::numeric_limits<StateType>::max()};

    // Map each StateId to the vector ID containing the state description (or Unexplored ID)
    std::vector<StateType> _state_to_vect_idx;
    // Vector containing the explored states (attention, the vector id is not the state ID)
    std::vector<std::unique_ptr<state_properties::StateDescription<StateType, ValueType>>> _state_description_ptrs;

    IdToStateMap _unexplored_states;
};
}  // namespace smc_storm::samples
