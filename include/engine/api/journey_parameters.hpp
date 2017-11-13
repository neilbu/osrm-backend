/*

Copyright (c) 2016, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ENGINE_API_JOURNEY_PARAMETERS_HPP
#define ENGINE_API_JOURNEY_PARAMETERS_HPP

#include "engine/api/base_parameters.hpp"

#include <cstddef>

#include <algorithm>
#include <iterator>
#include <vector>

namespace osrm
{
namespace engine
{
namespace api
{

/**
 * Parameters specific to the OSRM Journey service.
 *
 * Holds member attributes:
 *  - sources: indices into coordinates indicating sources for the Table service, no sources means
 *             use all coordinates as sources
 *  - destinations: indices into coordinates indicating destinations for the Table service, no
 *                  destinations means use all coordinates as destinations
 *
 * \see OSRM, Coordinate, Hint, Bearing, RouteParame, RouteParameters, TableParameters,
 *      NearestParameters, TripParameters, MatchParameters and TileParameters
 */
struct JourneyParameters : public BaseParameters
{
    JourneyParameters() = default;
    template <typename... Args>
    JourneyParameters(Args... args_)
        : BaseParameters{std::forward<Args>(args_)...}
    {
    }

    bool IsValid() const
    {
        if (!BaseParameters::IsValid())
            return false;

        // Distance Table makes only sense with 2+ coodinates
        if (coordinates.size() < 2)
            return false;

        // 1/ The user is able to specify duplicates in srcs and dsts, in that case it's their fault

        return true;
    }
};
}
}
}

#endif // ENGINE_API_JOURNEY_PARAMETERS_HPP
