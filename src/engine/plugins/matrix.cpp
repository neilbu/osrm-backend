#include "engine/plugins/matrix.hpp"

#include "engine/api/matrix_api.hpp"
#include "engine/api/matrix_parameters.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/search_engine_data.hpp"
#include "util/json_container.hpp"
#include "util/string_util.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/assert.hpp>

namespace osrm
{
namespace engine
{
namespace plugins
{

MatrixPlugin::MatrixPlugin(const int max_locations_distance_table)
    : max_locations_distance_table(max_locations_distance_table)
{
}

Status MatrixPlugin::HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                  const RoutingAlgorithmsInterface &algorithms,
                                  const api::MatrixParameters &params,
                                  util::json::Object &result) const
{
    BOOST_ASSERT(route_parameters.IsValid());

    if (!algorithms.HasDirectShortestPathSearch() && !algorithms.HasShortestPathSearch())
    {
        return Error(
            "NotImplemented",
            "Direct shortest path search used in matrix generation is not implemented for the chosen search algorithm.",
            result);
    }

    if (!CheckAllCoordinates(params.coordinates))
    {
        return Error("InvalidOptions", "Coordinates are invalid", result);
    }

    if (params.bearings.size() > 0 && params.coordinates.size() != params.bearings.size())
    {
        return Error(
            "InvalidOptions", "Number of bearings does not match number of coordinates", result);
    }

    // Empty sources or destinations means the user wants all of them included, respectively
    // The ManyToMany routing algorithm we dispatch to below already handles this perfectly.
    const auto num_coordinates = params.coordinates.size();

    if (max_locations_distance_table > 0 &&
        ((num_coordinates * num_coordinates) >
         static_cast<std::size_t>(max_locations_distance_table * max_locations_distance_table)))
    {
        return Error("TooBig", "Too many table coordinates", result);
    }

    auto snapped_phantoms = SnapPhantomNodes(GetPhantomNodes(facade, params));
    std::vector<EdgeWeight> result_table(num_coordinates);
    
    if (result_table.empty())
    {
        return Error("NoMatrix", "No matrix found", result);
    }

    api::MatrixAPI matrix_api{facade, params};
    matrix_api.MakeResponse(result_table, snapped_phantoms, result);

    return Status::Ok;
}
}
}
}
