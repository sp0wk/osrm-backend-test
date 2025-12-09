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

// Final RI result type
struct RouteInspectionResult
{
    std::vector<NodeID> nodes;
    std::vector<EdgeID> edges;
    std::vector<EdgeWeight> weights;
    EdgeWeight cost{INVALID_EDGE_WEIGHT};

    bool IsValid() const noexcept
    {
        return nodes.size() >= 3 && nodes.front() == nodes.back() &&
               (edges.size() == nodes.size() - 1) && (edges.size() == weights.size()) &&
               cost != INVALID_EDGE_WEIGHT;
    }
};

/**
 * @brief Extracts original NodeIDs/EdgeIDs from the resulting path and creates final route result.
 *
 * @param g RI graph used to create a closed path
 * @param path resulting closed path
 * @return RouteInspectionResult final route result
 */
inline RouteInspectionResult prepareFinalRoute(const RiGraph &g, const Path &path)
{
    using namespace boost;

    if (const auto sz = path.size(); sz < 3)
    {
        util::Log(logDEBUG) << "Resulting path is empty or invalid";
        return {};
    }

    const auto toNodeID = [&](const auto v) { return get(boost::vertex_name, g, v); };

    const auto toEdgeID = [&](const auto e) { return get(boost::edge_name, g, e); };

    // init result
    RouteInspectionResult result;
    result.nodes.reserve(path.size());
    result.edges.reserve(path.size() - 1);
    result.cost = EdgeWeight{0};

    // first node
    result.nodes.emplace_back(toNodeID(path[0]));
#ifndef NDEBUG
    g.logVertexEnd(path[0], "final route #1");
#endif

    for (std::size_t n = 1; n < path.size(); ++n)
    {
        const auto u = path[n - 1];
        const auto v = path[n];

        // node
        result.nodes.emplace_back(toNodeID(v));
#ifndef NDEBUG
        g.logVertexEnd(v, "final route #" + std::to_string(n + 1));
#endif

        // edge
        const auto [e, exists] = edge(u, v, g);
        BOOST_ASSERT(exists);
        result.edges.emplace_back(toEdgeID(e));

        // cost
        const EdgeWeight w = get(edge_weight, g, e);
        result.weights.emplace_back(w);
        result.cost += w;
    }

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
            const auto node = get(vertex_name, g, v);
            const auto [s, e] = detail::getNodeEndpoints(facade, node);
            const auto len = util::coordinate_calculation::greatCircleDistance(s, e);
            lengths[node] = len;
            roadsLength += len;
        }

        // calc total route distance (approx using node endpoints)
        routeDist = std::accumulate(result.nodes.cbegin(),
                                    result.nodes.cend(),
                                    0,
                                    [&](auto total, auto node) { return total + lengths[node]; });

        std::ostringstream ss;
        ss << "Route inspection stats:" << std::endl;
        ss << "  Initial number of roads to visit: " << num_vertices(g) << std::endl;
        ss << "  Number of transitions (edges): " << num_edges(g) << std::endl;
        ss << "  Resulting number of visited roads (includes revisits): " << result.nodes.size()
           << std::endl;
        ss << "  Total roads length: " << roadsLength << std::endl;
        ss << "  Total route distance: " << routeDist << std::endl;
        ss << "  Total route cost: " << result.cost.__value << std::endl;

        return std::move(ss).str();
    }();

    return result;
}

/**
 * @brief Runs route inspection algorithm on a directed edge-based graph and returns resulting
 * closed path.
 *
 * @tparam Algorithm routing algorithm type
 * @param facade Algorithm-specific DataFacade representing input graph
 * @param start PhantomNode matching start location
 * @param polygon polygon limiting route inspection area
 * @return RouteInspectionResult resulting closed path
 */
template <typename Algorithm>
RouteInspectionResult routeInspection(const DataFacade<Algorithm> &facade,
                                      const PhantomNode &start,
                                      const util::Polygon &polygon,
                                      const bool allowResidentialRoads = false)
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
        PolygonFilter f{facade, polygon};
        if (allowResidentialRoads)
        {
            f.disallowedRoadClasses.erase("residential");
        }
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
