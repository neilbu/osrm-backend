/*

Copyright (c) 2015, Project OSRM, Dennis Luxen, others
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

#ifndef DISTANCE_MATRIX_HPP
#define DISTANCE_MATRIX_HPP

#include "plugin_base.hpp"

#include "../algorithms/object_encoder.hpp"
#include "../data_structures/query_edge.hpp"
#include "../data_structures/search_engine.hpp"
#include "../descriptors/descriptor_base.hpp"
#include "../util/json_renderer.hpp"
#include "../util/make_unique.hpp"
#include "../util/string_util.hpp"
#include "../util/timing_util.hpp"

#include <osrm/json_container.hpp>

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

template <class DataFacadeT> class DistanceMatrixPlugin final : public BasePlugin
{
  private:
    std::unique_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;

  public:
    explicit DistanceMatrixPlugin(DataFacadeT *facade)
        : descriptor_string("matrix"),
          facade(facade)
    {
        search_engine_ptr = osrm::make_unique<SearchEngine<DataFacadeT>>(facade);
    }

    virtual ~DistanceMatrixPlugin() {}

    const std::string GetDescriptor() const override final { return descriptor_string; }

    std::shared_ptr<std::vector<std::pair<EdgeWeight, double>>>
    BuildMatrix(const RouteParameters &route_parameters,
                  unsigned &calctime_in_us)
    {
        // check number of parameters
        if (2 > route_parameters.coordinates.size())
        {
            return nullptr;
        }

        if (std::any_of(begin(route_parameters.coordinates), end(route_parameters.coordinates),
                        [&](FixedPointCoordinate coordinate)
                        {
                return !coordinate.is_valid();
            }))
        {
            return nullptr;
        }

        const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());
        PhantomNodeArray phantom_node_vector(route_parameters.coordinates.size());
        for (unsigned i = 0; i < route_parameters.coordinates.size(); ++i)
        {
            if (checksum_OK && i < route_parameters.hints.size() &&
                !route_parameters.hints[i].empty())
            {
                PhantomNode current_phantom_node;
                ObjectEncoder::DecodeFromBase64(route_parameters.hints[i], current_phantom_node);
                if (current_phantom_node.is_valid(facade->GetNumberOfNodes()))
                {
                    phantom_node_vector[i].emplace_back(std::move(current_phantom_node));
                    continue;
                }
            }
            facade->IncrementalFindPhantomNodeForCoordinate(route_parameters.coordinates[i],
                                                            phantom_node_vector[i],
                                                            1);

            BOOST_ASSERT(phantom_node_vector[i].front().is_valid(facade->GetNumberOfNodes()));
        }

        TIMER_START(multi_target);

        unsigned nodeCount = phantom_node_vector.size();
        std::shared_ptr<std::vector<std::pair<EdgeWeight, double>>> results = std::make_shared<std::vector<std::pair<EdgeWeight, double>>>();
        results->reserve(nodeCount * nodeCount);
        for (unsigned sourceIndex = 0; sourceIndex < nodeCount; ++sourceIndex)
        {
            const auto &source = phantom_node_vector[sourceIndex];
            const std::vector<PhantomNode> &phantom_node_source = source;
            for (unsigned targetIndex = 0; targetIndex < nodeCount; ++targetIndex)
            {
                if (sourceIndex == targetIndex)
                {
                    results->emplace_back(std::make_pair(0, 0));
                }
                else
                {
                    const auto &target = phantom_node_vector[targetIndex];
                    const std::vector<PhantomNode> &phantom_node_target = target;

                    PhantomNodes coordItem{phantom_node_source.front(), phantom_node_target.front()};

                    // Swap the front/back phantom nodes if they're not from the tiny CC
                    if (0 != coordItem.source_phantom.component_id)
                    {
                        coordItem.source_phantom = phantom_node_source.back();
                    }
                    if (0 != coordItem.target_phantom.component_id)
                    {
                        coordItem.target_phantom = phantom_node_target.back();
                    }

                    InternalRouteResult raw_route;
                    raw_route.segment_end_coordinates.emplace_back(coordItem);

                    search_engine_ptr->shortest_path(raw_route.segment_end_coordinates,
                                                     route_parameters.uturns, raw_route);
                    if (raw_route.shortest_path_length == INVALID_EDGE_WEIGHT)
                    {
                        search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(), raw_route);
                    }

                    std::vector<FixedPointCoordinate> coords;
                    coords.emplace_back(raw_route.segment_end_coordinates.front().source_phantom.location);
                    for (const auto i : osrm::irange<std::size_t>(0, raw_route.unpacked_path_segments.size()))
                    {
                        FixedPointCoordinate current_coordinate;
                        for (const PathData &path_data : raw_route.unpacked_path_segments[i])
                        {
                            current_coordinate = facade->GetCoordinateOfNode(path_data.node);
                            coords.emplace_back(current_coordinate);
                        }
                        coords.emplace_back(raw_route.segment_end_coordinates[i].target_phantom.location);
                    }
                    float routeDistance = 0;
                    for (unsigned i = 1; i < coords.size(); ++i)
                    {
                        routeDistance += coordinate_calculation::euclidean_distance(
                            coords[i - 1], coords[i]);
                    }

                    std::pair<EdgeWeight, double> dist =
                            std::make_pair(raw_route.shortest_path_length, (int)routeDistance);
                    results->emplace_back(std::move(dist));
                }
            }
        }

        TIMER_STOP(multi_target);
        calctime_in_us = TIMER_USEC(multi_target);

        return results;
    }

    int HandleRequest(const RouteParameters &route_parameters,
                      osrm::json::Object &json_result) override final
    {
        unsigned calctime_in_ms = 0;
        std::shared_ptr<std::vector<std::pair<EdgeWeight, double>>> result_table = BuildMatrix(route_parameters, calctime_in_ms);

        if (!result_table)
        {
            json_result.values["status"] = 500;
            json_result.values["status_message"] = "Failed to generate matrix";
            json_result.values["request_size"] = route_parameters.coordinates.size();
            return 500;
        }

        osrm::json::Array json_array;
        unsigned coord_size = route_parameters.coordinates.size();

        for (unsigned row = 0; row < coord_size; ++row)
        {
            osrm::json::Array json_row;
            for (unsigned column = 0; column < coord_size; ++column)
            {
                unsigned i = (coord_size * row) + column;
                auto routing_result = result_table->operator[](i);

                osrm::json::Object result;
                result.values["time_cost"] = routing_result.first;
                result.values["distance"] = routing_result.second;
                json_row.values.emplace_back(result);
            }
            json_array.values.emplace_back(json_row)                   ;
        }
        json_result.values["distances"] = json_array;
        return 200;
    }

  private:
    std::string descriptor_string;
    DataFacadeT *facade;
};

#endif // DISTANCE_MATRIX_HPP
