#include "engine/plugins/route_inspection.hpp"

#include "engine/algorithm.hpp"
#include "engine/api/route_inspection_api.hpp"
#include "engine/api/route_inspection_parameters.hpp"
#include "engine/route_inspection/route_inspection.hpp"

#include "util/coordinate_calculation.hpp"
#include "util/polygon.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace osrm::engine::plugins
{

// given the node order in which to visit, compute the actual route (with geometry, travel time and
// so on) and return the result
// NOTE: Final route may freely go outside polygon if it's more optimal to do so
InternalRouteResult
RouteInspectionPlugin::ComputeRoute(const RoutingAlgorithmsInterface &algorithms,
                                    const std::vector<PhantomNodeCandidates> &waypoint_candidates,
                                    const std::vector<NodeID> & /* ri_path */) const
{
    // TODO handle ri_path

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

    // check input polygon
    if (auto sz = parameters.polygon.size(); sz == 0)
    {
        return Error("TooBig", "Limiting polygon must be specified", result);
    }
    else if (sz > static_cast<size_t>(max_ri_polygon_points))
    {
        return Error("TooBig", "Limiting polygon has too many points", result);
    }

    if (!CheckAllCoordinates(parameters.polygon))
    {
        return Error("InvalidValue", "Invalid polygon point(s)", result);
    }

    if (auto area = util::coordinate_calculation::computeArea(parameters.polygon) / 1e6;
        area > max_ri_polygon_area_km_sqr)
    {
        return Error(
            "TooBig", "Limiting polygon is too big with area=" + std::to_string(area), result);
    }

    util::Polygon polygon{parameters.polygon};
    if (!polygon.IsValid())
    {
        return Error("InvalidValue",
                     "Resulting polygon is invalid with size=" + std::to_string(polygon.size()),
                     result);
    }

    // check start coordinate
    if (!CheckAllCoordinates(parameters.coordinates))
    {
        return Error("InvalidValue", "Invalid start coordinate", result);
    }

    if (parameters.coordinates.front() != parameters.coordinates.back())
    {
        return Error("InvalidValue", "Start and dest coordinate should be the same", result);
    }

    if (!polygon.Contains(parameters.coordinates.front()))
    {
        return Error("InvalidValue", "Starting location should be inside the polygon", result);
    }

    // snap start coordinate to node
    auto phantom_node_pairs = GetPhantomNodes(facade, parameters);
    if (phantom_node_pairs.size() != parameters.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(phantom_node_pairs, parameters.coordinates),
                     result);
    }

    auto snapped_phantoms = SnapPhantomNodes(std::move(phantom_node_pairs));
    BOOST_ASSERT(snapped_phantoms.size() == parameters.coordinates.size());

    const auto &startPhantom = snapped_phantoms.front().front();
    std::vector<NodeID> ri_path = route_inspection::routeInspection(facade, startPhantom, polygon);

    // get the route when visiting all nodes in optimized order
    InternalRouteResult route = ComputeRoute(algorithms, snapped_phantoms, ri_path);

    // get api response
    const std::vector<InternalRouteResult> routes = {route};
    api::RouteInspectionAPI ri_api{facade, parameters};
    ri_api.MakeResponse(routes, snapped_phantoms, result);

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
