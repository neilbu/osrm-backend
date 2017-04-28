#ifndef MATRIX_HPP
#define MATRIX_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/matrix_parameters.hpp"
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

class MatrixPlugin final : public BasePlugin
{
  public:
    explicit MatrixPlugin(const int max_locations_distance_table);

    Status HandleRequest(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                         const RoutingAlgorithmsInterface &algorithms,
                         const api::MatrixParameters &params,
                         util::json::Object &result) const;

  private:
    const int max_locations_distance_table;
};
}
}
}

#endif // TABLE_HPP
