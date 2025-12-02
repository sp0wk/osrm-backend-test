#include "engine/plugins/route_inspection.hpp"

#include "engine/algorithm.hpp"
#include "engine/api/route_inspection_api.hpp"
#include "engine/api/route_inspection_parameters.hpp"
#include "engine/route_inspection/route_inspection.hpp"

#include "util/coordinate_calculation.hpp"
#include "util/integer_range.hpp"
#include "util/polygon.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace osrm::engine::plugins
{

// given the node order in which to visit, compute the actual route (with geometry, travel time and
// so on) and return the result
// NOTE: Final route may freely go outside polygon if it's more optimal to do so
InternalRouteResult RouteInspectionPlugin::ComputeRoute(
    const RoutingAlgorithmsInterface &algorithms,
    const std::vector<PhantomNodeCandidates> &waypoint_candidates) const
{
    auto min_route = algorithms.ShortestPathSearch(waypoint_candidates, {false});
    BOOST_ASSERT_MSG(min_route.shortest_path_weight < INVALID_EDGE_WEIGHT, "unroutable route");
    return min_route;
}

template <typename AlgorithmT>
Status RouteInspectionPlugin::HandleRequest(const DataFacade<AlgorithmT> &facade,
                                            const RoutingAlgorithmsInterface &algorithms,
                                            const api::RouteInspectionParameters &parameters,
                                            osrm::engine::api::ResultT &result) const
{
    if (!CheckAlgorithms(parameters, algorithms, result))
    {
        return Status::Error;
    }

    if (!algorithms.HasShortestPathSearch())
    {
        return Error("NotImplemented",
                     "Shortest path search is not implemented for the chosen search algorithm.",
                     result);
    }

    BOOST_ASSERT(parameters.IsValid());

    // check start coordinate
    if (!CheckAllCoordinates(parameters.coordinates))
    {
        return Error("InvalidValue", "Invalid start coordinate", result);
    }

    if (parameters.coordinates.front() != parameters.coordinates.back())
    {
        return Error("InvalidValue", "Start and dest coordinate should be the same", result);
    }

    // check input polygon
    util::Log(logDEBUG) << [&]
    {
        std::ostringstream os;
        os << "Limiting polygon coords: " << std::endl;
        for (const auto c : parameters.polygon)
        {
            os << "[" << c.lon.__value << ", " << c.lat.__value << "]," << std::endl;
        }
        return std::move(os).str();
    }();

    if (auto sz = parameters.polygon.size(); sz == 0)
    {
        return Error("TooBig", "Limiting polygon must be specified", result);
    }
    else if (sz < 4)
    {
        return Error("InvalidValue", "Polygon must have at least 4 points", result);
    }
    else if (parameters.polygon.front() != parameters.polygon.back())
    {
        return Error("InvalidValue", "Polygon's start/end points should be the same", result);
    }
    else if (max_ri_polygon_points > 0 && sz > static_cast<size_t>(max_ri_polygon_points))
    {
        return Error("TooBig", "Limiting polygon has too many points", result);
    }

    if (!CheckAllCoordinates(parameters.polygon))
    {
        return Error("InvalidValue", "Invalid polygon point(s)", result);
    }

    if (max_ri_polygon_area_km_sqr > 0)
    {
        if (auto area = util::coordinate_calculation::computeArea(parameters.polygon) / 1e6;
            area > max_ri_polygon_area_km_sqr)
        {
            return Error(
                "TooBig", "Limiting polygon is too big with area=" + std::to_string(area), result);
        }
    }

    util::Polygon polygon{parameters.polygon};
    if (!polygon.IsValid())
    {
        return Error("InvalidValue",
                     "Resulting polygon is invalid with size=" + std::to_string(polygon.size()),
                     result);
    }

    // trivial check for route feasibility
    if (!polygon.Contains(parameters.coordinates.front()))
    {
        return Error("NoRoute", "Starting location should be inside the polygon", result);
    }

    // snap start coordinate to node
    auto phantom_node_pairs = GetPhantomNodes(facade, parameters);
    if (phantom_node_pairs.size() != parameters.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(phantom_node_pairs, parameters.coordinates),
                     result);
    }

    const auto snapped_phantoms = SnapPhantomNodes(std::move(phantom_node_pairs));
    BOOST_ASSERT(snapped_phantoms.size() == parameters.coordinates.size());

    const auto &start_phantom = snapped_phantoms.front().front();
    std::vector<NodeID> ri_path = route_inspection::routeInspection(facade, start_phantom, polygon);
    if (ri_path.size() < 3 || ri_path.front() != ri_path.back())
    {
        return Error("NoRoute", "Couldn't find a valid roundtrip route", result);
    }

    // TODO rework
    api::RouteInspectionParameters ext_params = parameters;
    // prepare all coords
    ext_params.coordinates.pop_back();
    for (const auto i : util::irange<std::size_t>(1UL, ri_path.size() - 1))
    {
        const auto node = ri_path[i];
        const auto [_, c] = route_inspection::detail::getNodeEndpoints(facade, node);
        BOOST_ASSERT(c.IsValid());
        ext_params.coordinates.emplace_back(c);
    }
    ext_params.coordinates.emplace_back(ext_params.coordinates.front());

    // get phantoms
    auto ext_phantom_node_pairs = GetPhantomNodes(facade, ext_params);
    if (ext_phantom_node_pairs.size() != ext_params.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(ext_phantom_node_pairs, ext_params.coordinates),
                     result);
    }

    const auto ext_snapped_phantoms = SnapPhantomNodes(std::move(ext_phantom_node_pairs));
    BOOST_ASSERT(ext_snapped_phantoms.size() == ext_params.coordinates.size());

    // get the route when visiting all nodes in optimized order
    InternalRouteResult route = ComputeRoute(algorithms, ext_snapped_phantoms);
    if (!route.is_valid())
    {
        return Error("NoRoute",
                     "Couldn't find a valid roundtrip route for calculated road sequence",
                     result);
    }

    // get api response
    const std::vector<InternalRouteResult> routes = {route};
    const api::RouteInspectionAPI ri_api{facade, ext_params};
    ri_api.MakeResponse(routes, ext_snapped_phantoms, result);

    return Status::Ok;
}

// Explicit instantiations for routing algorithms

// CH
template Status RouteInspectionPlugin::HandleRequest<routing_algorithms::ch::Algorithm>(
    const DataFacade<routing_algorithms::ch::Algorithm> &,
    const RoutingAlgorithmsInterface &,
    const api::RouteInspectionParameters &,
    osrm::engine::api::ResultT &) const;

// MLD
template Status RouteInspectionPlugin::HandleRequest<routing_algorithms::mld::Algorithm>(
    const DataFacade<routing_algorithms::mld::Algorithm> &,
    const RoutingAlgorithmsInterface &,
    const api::RouteInspectionParameters &,
    osrm::engine::api::ResultT &) const;

} // namespace osrm::engine::plugins
