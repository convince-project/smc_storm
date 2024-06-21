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

#include <storm/exceptions/UnexpectedException.h>
#include <storm/utility/macros.h>

#include "samples/trace_result.hpp"

namespace smc_storm::samples {
const std::string toString(const TraceResult& result) {
    switch (result) {
    case TraceResult::VERIFIED:
        return "Verified";
    case TraceResult::NOT_VERIFIED:
        return "Not verified";
    case TraceResult::NO_INFO:
        return "No information";
    default:
        STORM_LOG_THROW(false, storm::exceptions::UnexpectedException, "Unknown TraceResult");
    }
}

}  // namespace smc_storm::samples