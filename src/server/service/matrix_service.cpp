#include "server/service/matrix_service.hpp"

#include "server/api/parameters_parser.hpp"
#include "engine/api/matrix_parameters.hpp"

#include "util/json_container.hpp"

#include <boost/format.hpp>

namespace osrm
{
namespace server
{
namespace service
{

engine::Status
MatrixService::RunQuery(std::size_t prefix_length, std::string &query, ResultT &result)
{
    result = util::json::Object();
    auto &json_result = result.get<util::json::Object>();

    auto query_iterator = query.begin();
    auto parameters =
        api::parseParameters<engine::api::MatrixParameters>(query_iterator, query.end());
    if (!parameters || query_iterator != query.end())
    {
        const auto position = std::distance(query.begin(), query_iterator);
        json_result.values["code"] = "InvalidQuery";
        json_result.values["message"] =
            "Query string malformed close to position " + std::to_string(prefix_length + position);
        return engine::Status::Error;
    }
    BOOST_ASSERT(parameters);

    if (!parameters->IsValid())
    {
        json_result.values["code"] = "InvalidOptions";
        json_result.values["message"] = "At least two coordinates required";
        return engine::Status::Error;
    }
    BOOST_ASSERT(parameters->IsValid());

    return BaseService::routing_machine.Matrix(*parameters, json_result);
}
}
}
}
