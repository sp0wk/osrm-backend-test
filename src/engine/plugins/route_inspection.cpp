#include "engine/plugins/route_inspection.hpp"

#include "engine/api/route_inspection_api.hpp"
#include "engine/api/route_inspection_parameters.hpp"
#include "engine/route_inspection/route_inspection.hpp"
#include "engine/routing_algorithms/routing_base.hpp"

#include "util/coordinate_calculation.hpp"
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
template <typename AlgorithmT>
InternalRouteResult
RouteInspectionPlugin::BuildRoute(const DataFacade<AlgorithmT> &facade,
                                  const RoutingAlgorithmsInterface & /* algorithms */,
                                  const PhantomEndpointCandidates &endpoints,
                                  const route_inspection::RouteInspectionResult &ri_result) const
{
    BOOST_ASSERT(ri_result.IsValid());

    const auto route = routing_algorithms::extractRoute(
        facade, ri_result.cost, endpoints, ri_result.nodes, ri_result.edges);

    // TODO: replace costly edges with actual shortest paths

    return route;
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

    if (!algorithms.SupportsRouteInspection())
    {
        return Error("NotImplemented",
                     "Route inspection is not supported by the chosen search algorithm.",
                     result);
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
    const PhantomEndpointCandidates endpoints{snapped_phantoms.front(), snapped_phantoms.back()};

    const auto &start_phantom = endpoints.source_phantoms.front();
    const auto ri_result = route_inspection::routeInspection(
        facade, start_phantom, polygon, parameters.allowResidentialRoads);
    if (!ri_result.IsValid())
    {
        return Error("NoRoute", "Couldn't find a valid roundtrip route", result);
    }

    // get the route when visiting all nodes in optimized order
    InternalRouteResult route = BuildRoute(facade, algorithms, endpoints, ri_result);
    if (!route.is_valid())
    {
        return Error("NoRoute",
                     "Couldn't find a valid roundtrip route for calculated road sequence",
                     result);
    }

    // get api response
    const std::vector<InternalRouteResult> routes = {route};
    const api::RouteInspectionAPI ri_api{facade, parameters};
    ri_api.MakeResponse(routes, snapped_phantoms, result);

    return Status::Ok;
}

// Explicit instantiations for routing algorithms

// CH

template InternalRouteResult RouteInspectionPlugin::BuildRoute<routing_algorithms::ch::Algorithm>(
    const DataFacade<routing_algorithms::ch::Algorithm> &,
    const RoutingAlgorithmsInterface &,
    const PhantomEndpointCandidates &,
    const route_inspection::RouteInspectionResult &) const;

template Status RouteInspectionPlugin::HandleRequest<routing_algorithms::ch::Algorithm>(
    const DataFacade<routing_algorithms::ch::Algorithm> &,
    const RoutingAlgorithmsInterface &,
    const api::RouteInspectionParameters &,
    osrm::engine::api::ResultT &) const;

// MLD

template InternalRouteResult RouteInspectionPlugin::BuildRoute<routing_algorithms::mld::Algorithm>(
    const DataFacade<routing_algorithms::mld::Algorithm> &,
    const RoutingAlgorithmsInterface &,
    const PhantomEndpointCandidates &,
    const route_inspection::RouteInspectionResult &) const;

template Status RouteInspectionPlugin::HandleRequest<routing_algorithms::mld::Algorithm>(
    const DataFacade<routing_algorithms::mld::Algorithm> &,
    const RoutingAlgorithmsInterface &,
    const api::RouteInspectionParameters &,
    osrm::engine::api::ResultT &) const;

} // namespace osrm::engine::plugins
