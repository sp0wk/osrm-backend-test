#ifndef ENGINE_API_ROUTE_INSPECTION_HPP
#define ENGINE_API_ROUTE_INSPECTION_HPP

#include "engine/api/route_api.hpp"
#include "engine/api/route_inspection_parameters.hpp"
#include "engine/datafacade/datafacade_base.hpp"

namespace osrm::engine::api
{

class RouteInspectionAPI final : public RouteAPI
{
  public:
    RouteInspectionAPI(const datafacade::BaseDataFacade &facade_,
                       const RouteInspectionParameters &parameters_)
        : RouteAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    // TODO: consider richer api

  protected:
    const RouteInspectionParameters &parameters;
};

} // namespace osrm::engine::api

#endif
