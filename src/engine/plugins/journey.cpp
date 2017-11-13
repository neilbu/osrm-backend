#include "engine/plugins/journey.hpp"

#include "engine/api/journey_api.hpp"
#include "engine/api/journey_parameters.hpp"
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

JourneyPlugin::JourneyPlugin(const int max_locations_distance_table)
    : max_locations_distance_table(max_locations_distance_table)
{
}

Status JourneyPlugin::HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                  const RoutingAlgorithmsInterface &algorithms,
                                  const api::JourneyParameters &params,
                                  util::json::Object &result) const
{
    BOOST_ASSERT(params.IsValid());

    if (!algorithms.HasDirectShortestPathSearch() && !algorithms.HasShortestPathSearch())
    {
        return Error(
            "NotImplemented",
            "Direct shortest path search used in journey generation is not implemented for the chosen search algorithm.",
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

    const bool continue_straight_at_waypoint = facade.GetContinueStraightDefault();

    std::vector<std::pair<EdgeWeight, double>> result_table;
    
    for (unsigned sourceIndex = 0; sourceIndex < num_coordinates; ++sourceIndex)
    {
        const auto sourceNode = snapped_phantoms[sourceIndex];
	for (unsigned targetIndex = 0; targetIndex < num_coordinates; ++targetIndex) 
	{
	    if (sourceIndex == targetIndex) 
	    {
		result_table.emplace_back(std::make_pair(0, 0));
	    }
	    else
	    {
		const auto targetNode = snapped_phantoms[targetIndex];
		std::vector<PhantomNodes> start_end_nodes;
		start_end_nodes.push_back(PhantomNodes{sourceNode, targetNode});
		auto &last_inserted = start_end_nodes.back();
		// enable forward direction if possible
		if (last_inserted.source_phantom.forward_segment_id.id != SPECIAL_SEGMENTID)
		{
		    last_inserted.source_phantom.forward_segment_id.enabled |=
			!continue_straight_at_waypoint;
		}
		// enable reverse direction if possible
		if (last_inserted.source_phantom.reverse_segment_id.id != SPECIAL_SEGMENTID)
		{
		    last_inserted.source_phantom.reverse_segment_id.enabled |=
			!continue_straight_at_waypoint;
		}
		InternalRouteResult raw_route;
		if (1 == start_end_nodes.size() && algorithms.HasDirectShortestPathSearch())
		{
		    raw_route = algorithms.DirectShortestPathSearch(start_end_nodes.front());
		}
		else
		{
		    raw_route =
			algorithms.ShortestPathSearch(start_end_nodes, continue_straight_at_waypoint);
		}

		if (raw_route.is_valid()) 
		{
		    // Calculate distance and time from the legs
		    std::vector<guidance::RouteLeg> legs;
		    std::vector<guidance::LegGeometry> leg_geometries;
		    auto number_of_legs = raw_route.segment_end_coordinates.size();
		    
		    double route_distance = 0.0;
		    double route_duration = 0.0;

		    for (auto idx : util::irange<std::size_t>(0UL, number_of_legs))
		    {
			const auto &phantoms = raw_route.segment_end_coordinates[idx];
			const auto &path_data = raw_route.unpacked_path_segments[idx];

			const bool reversed_source = raw_route.source_traversed_in_reverse[idx];
			const bool reversed_target = raw_route.target_traversed_in_reverse[idx];

			auto leg_geometry = guidance::assembleGeometry(facade,
								    path_data,
								    phantoms.source_phantom,
								    phantoms.target_phantom,
								    reversed_source,
								    reversed_target);
			auto leg = guidance::assembleLeg(facade,
							path_data,
							leg_geometry,
							phantoms.source_phantom,
							phantoms.target_phantom,
							reversed_target,
							false);
			

			route_distance += leg.distance;
			route_duration += leg.duration;
		    }

		    result_table.emplace_back(std::make_pair(route_distance, route_duration));
		    
		}
		else
		{
		    // We don't have a route, so cannot provide an answer
		    result_table.emplace_back(std::make_pair(-1, -1));
		}

	    }
	  
	}
    }
    
    if (result_table.empty())
    {
        return Error("NoJourney", "No journeys found", result);
    }

    api::JourneyAPI journey_api{facade, params};
    journey_api.MakeResponse(result_table, snapped_phantoms, result);

    return Status::Ok;
}
}
}
}
