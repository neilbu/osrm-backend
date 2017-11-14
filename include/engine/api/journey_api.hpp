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
        response.values["journeys"] = MakeResultTable(phantoms, durations);
        response.values["code"] = "Ok";
    }

  protected:
    virtual util::json::Array MakeResultTable(const std::vector<PhantomNode> &phantoms,
                                              const std::vector<std::pair<EdgeWeight, double>> &values) const
    {
        util::json::Array json_table;
        auto number_of_results = values.size();
        for (unsigned resultIndex = 0; resultIndex < number_of_results; ++resultIndex)
        {
            util::json::Object result = util::json::Object();
            const auto sourceNode = phantoms[resultIndex * 2];
            util::json::Array sourceArray;
            sourceArray.values.push_back(static_cast<double>(util::toFloating(sourceNode.location.lon)));
            sourceArray.values.push_back(static_cast<double>(util::toFloating(sourceNode.location.lat)));
            result.values["from"] = sourceArray;
            const auto targetNode = phantoms[(resultIndex * 2) + 1];
            util::json::Array targetArray;
            targetArray.values.push_back(static_cast<double>(util::toFloating(targetNode.location.lon)));
            targetArray.values.push_back(static_cast<double>(util::toFloating(targetNode.location.lat)));
            result.values["to"] = targetArray;
            if (values[resultIndex].first == MAXIMAL_EDGE_DURATION)
            {
                result.values["distance"] = util::json::Null();
                result.values["time"] = util::json::Null();			       
            }
            else
            {
                result.values["distance"] = values[resultIndex].first;
                result.values["time"] = values[resultIndex].second;			       
            }
            json_table.values.push_back(result);
        }
        
        return json_table;
    }

    const JourneyParameters &parameters;
};

} // ns api
} // ns engine
} // ns osrm

#endif
