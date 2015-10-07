/*

Copyright (c) 2015, Project OSRM contributors
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

#ifndef TRIP_FARTHEST_INSERTION_HPP
#define TRIP_FARTHEST_INSERTION_HPP

#include "../data_structures/search_engine.hpp"
#include "../util/dist_table_wrapper.hpp"

#include <osrm/json_container.hpp>
#include <boost/assert.hpp>

#include <cstdlib>
#include <algorithm>
#include <string>
#include <vector>
#include <limits>

namespace osrm
{
namespace trip
{

// given a route and a new location, find the best place of insertion and
// check the distance of roundtrip when the new location is additionally visited
using NodeIDIter = std::vector<NodeID>::iterator;
std::pair<EdgeWeight, NodeIDIter>
GetShortestRoundTrip(const NodeID new_loc,
                     const DistTableWrapper<EdgeWeight> &dist_table,
                     const std::size_t number_of_locations,
                     std::vector<NodeID> &route)
{

    auto min_trip_distance = INVALID_EDGE_WEIGHT;
    NodeIDIter next_insert_point_candidate;

    // for all nodes in the current trip find the best insertion resulting in the shortest path
    // assert min 2 nodes in route
    const auto start = std::begin(route);
    const auto end = std::end(route);
    for (auto from_node = start; from_node != end; ++from_node)
    {
        auto to_node = std::next(from_node);
        if (to_node == end)
        {
            to_node = start;
        }

        const auto dist_from = dist_table(*from_node, new_loc);
        const auto dist_to = dist_table(new_loc, *to_node);
        const auto trip_dist = dist_from + dist_to - dist_table(*from_node, *to_node);

        BOOST_ASSERT_MSG(dist_from != INVALID_EDGE_WEIGHT, "distance has invalid edge weight");
        BOOST_ASSERT_MSG(dist_to != INVALID_EDGE_WEIGHT, "distance has invalid edge weight");
        BOOST_ASSERT_MSG(trip_dist >= 0, "previous trip was not minimal. something's wrong");

        // from all possible insertions to the current trip, choose the shortest of all insertions
        if (trip_dist < min_trip_distance)
        {
            min_trip_distance = trip_dist;
            next_insert_point_candidate = to_node;
        }
    }
    BOOST_ASSERT_MSG(min_trip_distance != INVALID_EDGE_WEIGHT, "trip has invalid edge weight");

    return std::make_pair(min_trip_distance, next_insert_point_candidate);
}

template <typename NodeIDIterator>
// given two initial start nodes, find a roundtrip route using the farthest insertion algorithm
std::vector<NodeID> FindRoute(const std::size_t &number_of_locations,
                              const std::size_t &component_size,
                              const NodeIDIterator &start,
                              const NodeIDIterator &end,
                              const DistTableWrapper<EdgeWeight> &dist_table,
                              const NodeID &start1,
                              const NodeID &start2)
{
    BOOST_ASSERT_MSG(number_of_locations >= component_size,
                     "component size bigger than total number of locations");

    std::vector<NodeID> route;
    route.reserve(number_of_locations);

    // tracks which nodes have been already visited
    std::vector<bool> visited(number_of_locations, false);

    visited[start1] = true;
    visited[start2] = true;
    route.push_back(start1);
    route.push_back(start2);

    // add all other nodes missing (two nodes are already in the initial start trip)
    for (std::size_t j = 2; j < component_size; ++j)
    {

        auto farthest_distance = 0;
        auto next_node = -1;
        NodeIDIter next_insert_point;

        // find unvisited loc i that is the farthest away from all other visited locs
        for (auto i = start; i != end; ++i)
        {
            // find the shortest distance from i to all visited nodes
            if (!visited[*i])
            {
                const auto insert_candidate =
                    GetShortestRoundTrip(*i, dist_table, number_of_locations, route);

                BOOST_ASSERT_MSG(insert_candidate.first != INVALID_EDGE_WEIGHT,
                                 "shortest round trip is invalid");

                // add the location to the current trip such that it results in the shortest total
                // tour
                if (insert_candidate.first >= farthest_distance)
                {
                    farthest_distance = insert_candidate.first;
                    next_node = *i;
                    next_insert_point = insert_candidate.second;
                }
            }
        }

        BOOST_ASSERT_MSG(next_node >= 0, "next node to visit is invalid");

        // mark as visited and insert node
        visited[next_node] = true;
        route.insert(next_insert_point, next_node);
    }
    return route;
}

template <typename NodeIDIterator>
std::vector<NodeID> FarthestInsertionTrip(const NodeIDIterator &start,
                                          const NodeIDIterator &end,
                                          const std::size_t number_of_locations,
                                          const DistTableWrapper<EdgeWeight> &dist_table)
{
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // START FARTHEST INSERTION HERE
    // 1. start at a random round trip of 2 locations
    // 2. find the location that is the farthest away from the visited locations and whose insertion
    // will make the round trip the longest
    // 3. add the found location to the current round trip such that round trip is the shortest
    // 4. repeat 2-3 until all locations are visited
    // 5. DONE!
    //////////////////////////////////////////////////////////////////////////////////////////////////

    const auto component_size = std::distance(start, end);
    BOOST_ASSERT(component_size >= 0);

    auto max_from = -1;
    auto max_to = -1;

    if (static_cast<std::size_t>(component_size) == number_of_locations)
    {
        // find the pair of location with the biggest distance and make the pair the initial start
        // trip
        const auto index = std::distance(
            std::begin(dist_table), std::max_element(std::begin(dist_table), std::end(dist_table)));
        max_from = index / number_of_locations;
        max_to = index % number_of_locations;
    }
    else
    {
        auto max_dist = 0;
        for (auto x = start; x != end; ++x)
        {
            for (auto y = start; y != end; ++y)
            {
                const auto xy_dist = dist_table(*x, *y);
                if (xy_dist > max_dist)
                {
                    max_dist = xy_dist;
                    max_from = *x;
                    max_to = *y;
                }
            }
        }
    }
    BOOST_ASSERT(max_from >= 0);
    BOOST_ASSERT(max_to >= 0);
    BOOST_ASSERT_MSG(static_cast<std::size_t>(max_from) < number_of_locations, "start node");
    BOOST_ASSERT_MSG(static_cast<std::size_t>(max_to) < number_of_locations, "start node");
    return FindRoute(number_of_locations, component_size, start, end, dist_table, max_from, max_to);
}

} // end namespace trip
} // end namespace osrm

#endif // TRIP_FARTHEST_INSERTION_HPP
