#ifndef OSRM_ROUTE_INSPECTION_HPP
#define OSRM_ROUTE_INSPECTION_HPP

#include "engine/datafacade.hpp"
#include "engine/route_inspection/detail/route_inspection_graph.hpp"
#include "engine/route_inspection/detail/types.hpp"
#include "engine/route_inspection/detail/utils.hpp"
#include "engine/route_inspection/input_graph_adaptors.hpp"

#include "util/coordinate_calculation.hpp"
#include "util/log.hpp"
#include "util/polygon.hpp"
#include "util/typedefs.hpp"

#include <boost/range/adaptors.hpp>

#include <cstdlib>
#include <numeric>
#include <sstream>
#include <vector>

namespace osrm::engine::route_inspection
{

using namespace detail;

//-------------------------------------------------------------------------------------------------
// Main route inspection search
//-------------------------------------------------------------------------------------------------

// Route inspection (directed Chinese Postman Problem) solver on EBG graph (basically, reduced DRPP)
inline Path routeInspectionImpl(RiGraph &g, const Vertex s)
{
    BOOST_ASSERT_MSG(isStronglyConnectedGraph(g), "Graph is not strongly connected\n");

    // collect node degree deltas
    auto deltas = collectNodeDegreeDeltas(g);

    // check whether graph is already Eulerian
    if (isEulerianGraph(deltas))
    {
        if (const auto circuit = findEulerianCircuit(g, s); !circuit.empty())
        {
            // Eulerian circuit found -> done
            return circuit;
        }
    }
    else
    {
        // non-Eulerian graph -> augment graph and retry
        if (augmentImbalancedGraph(g, deltas) && isEulerianGraph(deltas))
        {
            // graph is now Eulerian -> find circuit
            if (const auto circuit = findEulerianCircuit(g, s); !circuit.empty())
            {
                // Eulerian circuit found -> done
                return circuit;
            }
        }
    }

    // route inspection search was unsuccessful
    return {};
}

/**
 * @brief Extracts original NodeIDs from the resulting path and creates final route result.
 *
 * @param g RI graph used to create a closed path
 * @param path resulting closed path
 * @return std::vector<NodeID> final route result
 */
inline std::vector<NodeID> prepareFinalRoute(const RiGraph &g, const Path &path)
{
    if (const auto sz = path.size(); sz < 3)
    {
        util::Log(logDEBUG) << "Resulting path is empty or invalid";
        return {};
    }

    std::vector<NodeID> route;
    route.reserve(path.size());

    std::transform(path.cbegin(),
                   path.cend(),
                   std::back_inserter(route),
                   [&g](const auto v)
                   {
#ifndef NDEBUG
                       g.logVertexEnd(v, "final route");
#endif
                       // extract original NodeID
                       return get(boost::vertex_name, g, v);
                   });

    // debug stats
    util::Log(logDEBUG) << [&]
    {
        using namespace boost;

        auto roadsLength{0}, routeDist{0};

        // collect road lengths
        std::unordered_map<NodeID, int> lengths;
        const auto &facade = g.GetFacade();
        for (const auto v : make_iterator_range(vertices(g)))
        {
            auto node = get(vertex_name, g, v);
            auto [s, e] = detail::getNodeEndpoints(facade, node);
            auto len = util::coordinate_calculation::greatCircleDistance(s, e);
            lengths[node] = len;
            roadsLength += len;
        }

        // calc total route distance (approx using node endpoints)
        routeDist = std::accumulate(route.cbegin(),
                                    route.cend(),
                                    0,
                                    [&](auto total, auto node) { return total + lengths[node]; });

        std::ostringstream ss;
        ss << "Route inspection stats:" << std::endl;
        ss << "  Initial number of roads to visit: " << num_vertices(g) << std::endl;
        ss << "  Number of transitions (edges) to visit: " << num_edges(g) << std::endl;
        ss << "  Resulting number of visited roads (includes revisits): " << route.size()
           << std::endl;
        ss << "  Total roads length: " << roadsLength << std::endl;
        ss << "  Total route distance: " << routeDist << std::endl;

        return std::move(ss).str();
    }();

    return route;
}

/**
 * @brief Runs route inspection algorithm on a directed edge-based graph and returns resulting
 * closed path.
 *
 * @tparam Algorithm routing algorithm type
 * @param facade Algorithm-specific DataFacade representing input graph
 * @param start PhantomNode matching start location
 * @param polygon polygon limiting route inspection area
 * @return std::vector<NodeID> resulting closed path as a list of node IDs (empty if not found)
 */
template <typename Algorithm>
std::vector<NodeID> routeInspection(const DataFacade<Algorithm> &facade,
                                    const PhantomNode &start,
                                    const util::Polygon &polygon)
{
    using namespace util;
    using namespace detail;

    using BaseGraph = AlgorithmBasedInputGraphWrapper<Algorithm>;

    BOOST_ASSERT(polygon.empty() || polygon.Contains(start.input_location));

    // TODO better selection
    const NodeID s =
        start.IsValidForwardSource() ? start.forward_segment_id.id : start.reverse_segment_id.id;

    // prepare graph data
    BaseGraph baseGraph{facade};
    RiGraph rig{baseGraph.GetFacade()};

    if (!polygon.empty())
    {
        const PolygonFilter f{facade, polygon};
        rig = buildRiGraph(baseGraph, s, f);
    }
    else
    {
        // no polygon provided
        rig = buildRiGraph(baseGraph, s);
    }

    if (!isValidGraph(rig))
    {
        Log(logERROR) << "Constructed RiGraph is invalid";
        return {};
    }

    // Optimize RiGraph structure for correct and efficient route inspection
    optimizeRiGraph(rig);

    if (!isValidGraph(rig))
    {
        Log(logERROR) << "Optimized RiGraph is invalid";
        return {};
    }

    // run route inspection from the source (0) vertex
    const auto path = routeInspectionImpl(rig, Vertex{0});

    return prepareFinalRoute(rig, path);
}

} // namespace osrm::engine::route_inspection

#endif /* OSRM_ROUTE_INSPECTION_HPP */
