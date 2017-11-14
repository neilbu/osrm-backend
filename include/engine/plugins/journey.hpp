#ifndef JOURNEY_HPP
#define JOURNEY_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/journey_parameters.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/search_engine_data.hpp"
#include "util/json_container.hpp"

namespace osrm
{
namespace engine
{
namespace plugins
{

class JourneyPlugin final : public BasePlugin
{
  public:
    explicit JourneyPlugin(const int max_locations_distance_table);

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::JourneyParameters &params,
                         util::json::Object &result) const;

  private:
    const int max_locations_distance_table;
};
}
}
}

#endif // JOURNEY_HPP
