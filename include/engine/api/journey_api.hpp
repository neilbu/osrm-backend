#ifndef ENGINE_API_JOURNEY_HPP
#define ENGINE_API_JOURNEY_HPP

#include "engine/api/base_api.hpp"
#include "engine/api/json_factory.hpp"
#include "engine/api/journey_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_steps.hpp"

#include "engine/internal_route_result.hpp"

#include "util/integer_range.hpp"

#include <boost/range/algorithm/transform.hpp>

#include <iterator>

namespace osrm
{
namespace engine
{
namespace api
{

class JourneyAPI final : public BaseAPI
{
  public:
    JourneyAPI(const datafacade::BaseDataFacade &facade_, const JourneyParameters &parameters_)
        : BaseAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    virtual void MakeResponse(const std::vector<std::pair<EdgeWeight, double>> &durations,
                              const std::vector<PhantomNode> &phantoms,
                              util::json::Object &response) const
    {
        auto number_of_coordinates = phantoms.size();
        response.values["sources"] = MakeWaypoints(phantoms);
        response.values["destinations"] = MakeWaypoints(phantoms);
        response.values["durations"] = MakeMatrix(durations, number_of_coordinates);
        response.values["code"] = "Ok";
    }

  protected:
    virtual util::json::Array MakeWaypoints(const std::vector<PhantomNode> &phantoms) const
    {
        util::json::Array json_waypoints;
        json_waypoints.values.reserve(phantoms.size());
        BOOST_ASSERT(phantoms.size() == parameters.coordinates.size());

        boost::range::transform(
            phantoms,
            std::back_inserter(json_waypoints.values),
            [this](const PhantomNode &phantom) { 
		util::json::Array array;
		array.values.push_back(static_cast<double>(util::toFloating(phantom.location.lon)));
		array.values.push_back(static_cast<double>(util::toFloating(phantom.location.lat)));
		return array;
	    });
        return json_waypoints;
    }

    virtual util::json::Array MakeMatrix(const std::vector<std::pair<EdgeWeight, double>> &values,
                                        std::size_t matrix_size) const
    {
        util::json::Array json_table;
        for (const auto row : util::irange<std::size_t>(0UL, matrix_size))
        {
            util::json::Array json_row;
            auto row_begin_iterator = values.begin() + (row * matrix_size);
            auto row_end_iterator = values.begin() + ((row + 1) * matrix_size);
            json_row.values.resize(matrix_size);
            std::transform(row_begin_iterator,
                           row_end_iterator,
                           json_row.values.begin(),
                           [](const std::pair<EdgeWeight, double> duration) {
                               util::json::Object result = util::json::Object();
                               if (duration.first == MAXIMAL_EDGE_DURATION)
                               {
				    result.values["distance"] = util::json::Null();
				    result.values["time"] = util::json::Null();			       
                               }
                               else
			       {
				    result.values["distance"] = duration.first;
				    result.values["time"] = duration.second;			       
			       }
			       return result;
                           });
            json_table.values.push_back(std::move(json_row));
        }
        return json_table;
    }

    const JourneyParameters &parameters;
};

} // ns api
} // ns engine
} // ns osrm

#endif
