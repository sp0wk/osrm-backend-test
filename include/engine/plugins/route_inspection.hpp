#ifndef ROUTE_INSPECTION_HPP
#define ROUTE_INSPECTION_HPP

#include "engine/api/route_inspection_parameters.hpp"
#include "engine/datafacade.hpp"
#include "engine/plugins/plugin_base.hpp"
#include "engine/routing_algorithms.hpp"

#include <cstdlib>
#include <vector>

namespace osrm::engine::plugins
{

class RouteInspectionPlugin final : public BasePlugin
{
  private:
    const int max_ri_polygon_points;
    const int max_ri_polygon_area_km_sqr;

    InternalRouteResult ComputeRoute(const RoutingAlgorithmsInterface &algorithms,
                                     const std::vector<PhantomNodeCandidates> &waypoint_candidates,
                                     const std::vector<NodeID> &ri_path) const;

  public:
    explicit RouteInspectionPlugin(const int max_ri_polygon_points_,
                                   const int max_ri_polygon_area_km_sqr_,
                                   std::optional<double> default_radius)
        : BasePlugin(default_radius), max_ri_polygon_points(max_ri_polygon_points_),
          max_ri_polygon_area_km_sqr(max_ri_polygon_area_km_sqr_)
    {
    }

    template <typename AlgorithmT>
    Status HandleRequest(const DataFacade<AlgorithmT> &facade,
                         const RoutingAlgorithmsInterface &algorithms,
                         const api::RouteInspectionParameters &parameters,
                         osrm::engine::api::ResultT &result) const;
};

} // namespace osrm::engine::plugins

#endif // ROUTE_INSPECTION_HPP
