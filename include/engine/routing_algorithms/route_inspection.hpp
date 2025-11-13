#ifndef OSRM_ROUTE_INSPECTION_HPP
#define OSRM_ROUTE_INSPECTION_HPP

#include "util/bgl_graph_adaptor.hpp"
#include "util/coordinate.hpp"
#include "util/exception.hpp"
#include "util/log.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/range/adaptors.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <optional>
#include <stack>
#include <tuple>
#include <vector>

namespace osrm::engine::routing_algorithms
{

namespace detail
{

//-------------------------------------------------------------------------------------------------
// Common types
//-------------------------------------------------------------------------------------------------

// Main graph type (BGL based) used for route inspection implementation
template <typename BaseNode>
using RiGraph = boost::adjacency_list<boost::vecS,
                                      boost::vecS,
                                      boost::directedS,
                                      boost::property<boost::vertex_name_t, BaseNode>,
                                      boost::property<boost::edge_weight_t, EdgeWeight>>;

// Represents a sequence of nodes (vertices)
template <typename Vertex> using Path = std::vector<Vertex>;

using NodeDegreeDelta = int16_t;
// Stores difference between incoming (negative) and outgoing (positive) edges for nodes where array
// index is node id
using NodeDegreeDeltaArray = std::vector<NodeDegreeDelta>;

// Represents shortest path and its total cost
template <typename Vertex> struct ShortestPath
{
    Path<Vertex> path;
    EdgeWeight cost{INVALID_EDGE_WEIGHT};
};

// Represents shortest paths between all sources/targets
template <typename Vertex> struct PathMatrix
{
    struct Column
    {
        Vertex target;
        ShortestPath<Vertex> path;
    };

    struct Row
    {
        Vertex source;
        std::vector<Column> columns;
    };

    std::vector<Row> rows;
};

// Represents flow between two vertices in PathMatrix
struct EdgeFlow
{
    size_t sIdx;
    size_t tIdx;
    uint32_t flow{0};
};

// Contains positive-only flow between two vertices
using MinCostFlow = std::vector<EdgeFlow>;

//-------------------------------------------------------------------------------------------------
// Eulerian circuit
//-------------------------------------------------------------------------------------------------

// Checks whether directed graph is strongly connected
template <typename Graph> bool isStronglyConnectedGraph(const Graph &g)
{
    using namespace boost;
    // TODO consider using TarjanSCC from osrm::util instead
    std::vector<int> component(num_vertices(g));
    auto num =
        strong_components(g, make_iterator_property_map(component.begin(), get(vertex_index, g)));
    return num == 1;
}

// Collects edge degree delta for all nodes in the graph
template <typename Graph> NodeDegreeDeltaArray collectNodeDegreeDeltas(const Graph &g)
{
    NodeDegreeDeltaArray deltas(num_vertices(g));
    auto [b, e] = edges(g);
    for (; b != e; ++b)
    {
        deltas[source(*b, g)] += 1;
        deltas[target(*b, g)] -= 1;
    }
    return deltas;
}

// Checks whether graph is Eulerian based on node degree deltas
inline bool isEulerianGraph(const NodeDegreeDeltaArray &deltas)
{
    for (auto d : deltas)
    {
        if (d != 0)
        {
            return false;
        }
    }
    return true;
}

// Checks whether graph is Eulerian
template <typename Graph> bool isEulerianGraph(const Graph &g)
{
    return isEulerianGraph(collectNodeDegreeDeltas(g));
}

// Finds Eulerian circuit on directed Eulerian graph using Hierholzer’s algorithm
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
Path<Vertex> findEulerianCircuit(const Graph &g, const Vertex source)
{
    using EdgeIt = decltype(out_edges(source, g).first);

    // prepare initial state
    const auto nbOfVertices = num_vertices(g);
    std::vector<EdgeIt> unusedEdges(nbOfVertices);
    auto [vb, ve] = vertices(g);
    for (; vb != ve; ++vb)
    {
        unusedEdges[*vb] = out_edges(*vb, g).first;
    }
    std::stack<Vertex> st;
    Path<Vertex> circuit;
    circuit.reserve(nbOfVertices);

    // find circuit
    st.push(source);
    while (!st.empty())
    {
        auto v = st.top();
        if (auto e = unusedEdges[v]; e != out_edges(v, g).second)
        {
            // take next edge v->u
            auto u = target(*e, g);
            unusedEdges[v]++;
            st.push(u);
        }
        else
        {
            circuit.emplace_back(v);
            st.pop();
        }
    }

    // verify circuit visits all edges and is a closed one
    if ((circuit.size() - 1) != num_edges(g) || circuit.front() != circuit.back())
    {
        // incomplete circuit
        util::Log(logDEBUG) << "Incomplete Eulerian circuit for s=" << source;
        return {};
    }

    std::reverse(circuit.begin(), circuit.end());
    return circuit;
}

//-------------------------------------------------------------------------------------------------
// Shortest paths
//-------------------------------------------------------------------------------------------------

// Finds shortest paths from source s to all graph's vertices
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
void shortestPaths(const Graph &g,
                   const Vertex s,
                   std::vector<Vertex> &preds,
                   std::vector<EdgeWeight> &dists)
{
    BOOST_ASSERT(s < num_vertices(g));
    boost::dijkstra_shortest_paths(g,
                                   s,
                                   boost::predecessor_map(preds.data())
                                       .distance_map(dists.data())
                                       .distance_inf(INVALID_EDGE_WEIGHT));
}

// Extracts shortest paths from s to t using result of shortestPaths()
template <typename Vertex>
Path<Vertex> extractPath(const std::vector<Vertex> &preds, const Vertex s, const Vertex t)
{
    Path<Vertex> result;
    // initially reserve x2 of nodes between s and t
    result.reserve(std::max<size_t>(16, (t > s ? t - s : s - t) * 2));
    Vertex curNode{t};
    while (curNode != s)
    {
        result.emplace_back(curNode);
        curNode = preds[curNode];
        // safeguard against loops
        if (result.back() == curNode)
        {
            break;
        }
    }
    result.emplace_back(curNode);
    std::reverse(result.begin(), result.end());
    return result;
}

// Finds shortest paths from a source vertex to all target vertices
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
std::vector<ShortestPath<Vertex>>
oneToMany(const Graph &g, const Vertex source, const std::vector<Vertex> &targets)
{
    // prepare maps for predecessors and distances
    std::vector<Vertex> preds(num_vertices(g));
    std::vector<EdgeWeight> dists(num_vertices(g), INVALID_EDGE_WEIGHT);

    // calculate shortest paths from source to all vertices
    shortestPaths(g, source, preds, dists);

    // extract paths for all targets
    std::vector<ShortestPath<Vertex>> paths;
    paths.reserve(targets.size());
    for (auto t : targets)
    {
        auto &sp = paths.emplace_back();
        auto cost = dists[t];
        if (cost == INVALID_EDGE_WEIGHT)
        {
            // no path found
            util::Log(logDEBUG) << "No shortest path found for s=" << source << ", t=" << t;
            continue;
        }
        sp.path = extractPath(preds, source, t);
        sp.cost = cost;
    }

    return paths;
}

// Finds shortest paths between all sources and targets
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
PathMatrix<Vertex>
manyToMany(const Graph &g, const std::vector<Vertex> &sources, const std::vector<Vertex> &targets)
{
    PathMatrix<Vertex> m;
    m.rows.reserve(sources.size());
    for (auto s : sources)
    {
        auto paths = oneToMany(g, s, targets);
        BOOST_ASSERT(targets.size() == paths.size());
        auto &row = m.rows.emplace_back();
        row.source = s;
        for (auto i : util::irange(0UL, targets.size()))
        {
            row.columns.emplace_back(targets[i], std::move(paths[i]));
        }
    }
    return m;
}

//-------------------------------------------------------------------------------------------------
// Min-cost flow graph augmentation
//-------------------------------------------------------------------------------------------------

/**
 * @brief Builds min-cost flow graph compatible with
 * boost::successive_shortest_path_nonnegative_weights.
 *
 * @param pathMatrix table/matrix of surplus and deficit nodes with respective shortest paths
 * between them where rows represent surplus nodes (sources) and columns - deficit nodes (targets).
 * @param deltas degree delta storage for all surplus/deficit nodes
 * @param maxCapacity total amount of surplus
 * @return auto built graph together with super-source and super-sink nodes
 */
template <typename Vertex>
auto buildMcfGraph(const PathMatrix<Vertex> &pathMatrix,
                   const NodeDegreeDeltaArray &deltas,
                   const uint32_t maxCapacity = std::numeric_limits<uint32_t>::max())
{
    using namespace boost;

    BOOST_ASSERT(!pathMatrix.rows.empty());

    // required BGL traits
    using Traits = adjacency_list_traits<vecS, vecS, directedS>;
    using V = Traits::vertex_descriptor;
    using E = Traits::edge_descriptor;
    using VertexProperties = property<vertex_name_t, size_t>; // for vertex to matrix index map
    using EdgeProperties = property<
        edge_weight_t,
        int32_t,
        property<edge_capacity_t,
                 uint32_t,
                 property<edge_residual_capacity_t, uint32_t, property<edge_reverse_t, E>>>>;
    using McfGraph = adjacency_list<vecS, vecS, directedS, VertexProperties, EdgeProperties>;

    // create graph
    McfGraph g;
    auto capacityMap = get(edge_capacity, g);
    auto weightMap = get(edge_weight, g);
    auto revMap = get(edge_reverse, g);

    auto superSource = add_vertex(-1, g);
    auto superSink = add_vertex(-1, g);

    // helper
    auto const addEdge = [&](V from, V to, uint32_t capacity, int32_t cost)
    {
        E e, rev;
        bool success;
        tie(e, success) = add_edge(from, to, g);
        BOOST_ASSERT(success);
        tie(rev, success) = add_edge(to, from, g);
        BOOST_ASSERT(success);

        capacityMap[e] = capacity;
        capacityMap[rev] = 0; // reverse edge has 0 capacity
        weightMap[e] = cost;
        weightMap[rev] = -cost;
        revMap[e] = rev;
        revMap[rev] = e;
    };

    // populate graph
    std::unordered_map<Vertex, V> sinks; // to check already added sinks
    for (const auto &[rIdx, s] : adaptors::index(pathMatrix.rows))
    {
        // insert new surplus node and connect it with super-source
        auto sv = add_vertex(rIdx, g);
        auto capacity = std::abs(deltas[s.source]);
        addEdge(superSource, sv, capacity, 0);

        for (const auto &[cIdx, d] : adaptors::index(s.columns))
        {
            V dv = McfGraph::null_vertex();
            if (auto it = sinks.find(d.target); it != sinks.cend())
            {
                // sink node was already inserted
                dv = it->second;
            }
            else
            {
                // insert new sink node and connect it to super-sink
                dv = add_vertex(cIdx, g);
                sinks[d.target] = dv;
                // connect new deficit node with the sink
                auto capacity = deltas[d.target];
                addEdge(dv, superSink, capacity, 0);
            }
            // connect surplus node to deficit one if path exists
            if (auto cost = d.path.cost; cost != INVALID_EDGE_WEIGHT)
            {
                addEdge(sv, dv, maxCapacity, cost.__value);
            }
        }
    }

    return std::make_tuple(std::move(g), superSource, superSink);
}

/**
 * @brief Solves minimum-cost flow problem using nodes/edges from pathMatrix.
 *
 * @param pathMatrix table/matrix of surplus and deficit nodes with respective shortest paths
 * between them where rows represent surplus nodes (sources) and columns - deficit nodes (targets).
 * @param deltas degree delta storage for all surplus/deficit nodes
 * @param maxCapacity total amount of surplus
 * @return MinCostFlow edges with positive flow
 */
template <typename Vertex>
MinCostFlow solveMinCostFlow(const PathMatrix<Vertex> &pathMatrix,
                             const NodeDegreeDeltaArray &deltas,
                             const size_t maxCapacity = std::numeric_limits<size_t>::max())
{
    using namespace boost;

    // build min-cost flow graph
    auto [g, superSource, superSink] = buildMcfGraph(pathMatrix, deltas, maxCapacity);
    auto vertexToIdxMap = get(vertex_name, g);
    auto capacityMap = get(edge_capacity, g);
    auto residualCapacityMap = get(edge_residual_capacity, g);

    // solve min-cost flow
    successive_shortest_path_nonnegative_weights(g, superSource, superSink);

    // prepare result
    MinCostFlow result;
    result.reserve(pathMatrix.rows.size());
    for (auto e : make_iterator_range(edges(g)))
    {
        auto s = source(e, g);
        auto t = target(e, g);
        if (s == superSource || t == superSink)
        {
            // ignore virtual edges
            continue;
        }
        // add edge with positive flow to result
        if (auto flow = capacityMap[e] - residualCapacityMap[e]; flow > 0)
        {
            result.emplace_back(vertexToIdxMap[s], vertexToIdxMap[t], flow);
        }
    }

    return result;
}

// Add deficit edges to imbalanced graph through solving min-cost flow problem to make graph
// Eulerian
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
bool augmentImbalancedGraph(Graph &g, NodeDegreeDeltaArray &deltas)
{
    // collect surplus (delta < 0) and deficit (delta > 0) nodes
    // NOTE: normally for MCF nodes with delta > 0 become surplus nodes but, for convenience, we
    // swap surplus and deficit nodes to avoid inversing shortest paths and resulting nodes
    std::vector<Vertex> surplusNodes, deficitNodes;
    auto totalSurplus{0U}, totalDeficit{0U};
    // assume only half of nodes are either surplus or deficit ones
    surplusNodes.reserve(std::min<size_t>(16, deltas.size() / 4));
    deficitNodes.reserve(std::min<size_t>(16, deltas.size() / 4));
    for (const auto [i, delta] : boost::adaptors::index(deltas))
    {
        if (delta < 0)
        {
            // node requires additional outgoing edges
            surplusNodes.emplace_back(i);
            totalSurplus += std::abs(delta);
        }
        else if (delta > 0)
        {
            // node requires additional incoming edges
            deficitNodes.emplace_back(i);
            totalDeficit += delta;
        }
    }

    // sanity check to verify min-cost flow is solvable
    if (totalSurplus != totalDeficit)
    {
        BOOST_ASSERT_MSG(false, "totalSurplus != totalDeficit");
        util::Log(logERROR) << "Min-cost flow problem cannot be solved: surplus=" << totalSurplus
                            << ", deficit=" << totalDeficit;
        return false;
    }

    // calculate shortest paths between surplus and deficit nodes
    const auto pathMatrix = manyToMany(g, surplusNodes, deficitNodes);

    // solve min-cost flow to get edge duplication requirements
    MinCostFlow mcf = solveMinCostFlow(pathMatrix, deltas, totalSurplus);
    if (mcf.empty())
    {
        util::Log(logERROR) << "MCF could not be solved";
        return false;
    }

    // duplicate edges based on flow value
    for (const auto &e : mcf)
    {
        const auto &row = pathMatrix.rows[e.sIdx];
        const auto &col = row.columns[e.tIdx];
        for (auto i = 0U; i < e.flow; ++i)
        {
            const auto &p = col.path.path;
            for (auto it = std::next(p.cbegin()); it != p.cend(); ++it)
            {
                auto u = *std::prev(it);
                auto v = *it;
                auto [baseEdge, found] = edge(u, v, g);
                BOOST_ASSERT(found);
                // duplicate edge
                add_edge(u, v, get(boost::edge_weight, g, baseEdge), g);
                // adjust degree deltas
                ++deltas[u];
                --deltas[v];
            }
        }
    }

    return true;
}

//-------------------------------------------------------------------------------------------------
// Main route inspection search
//-------------------------------------------------------------------------------------------------

struct NoFilter
{
    template <typename... Args> constexpr bool operator()(Args &&...) const { return true; }
};

// Builds a RI (route inspection) compatible node-based directed subgraph using following rules:
// - Starts at the "start" node of the base graph
// - Building a subgraph happens by BFS-traversing base graph
// - EdgeFilter is applied to base graph's edges to filter them out from resulting subgraph
// - Replaces EBG nodes/edges with corresponding NBG nodes/edges
//   Example:
//      EBG:           ->           NBG:
//                                         o g
//                                         |
//              ↑                          o d
//              | 3              f      c /|\ e      h
//      2 <----- ----> 4    ->   o-------o-|-o-------o
//              ↑ 1                       \|/
//              |                          o b
//                                         |
//                                         o a
//
// Node mappings:   1 -> ab; 2 -> cf; 3 -> dg; 4 -> eh
// Edge mappings:   1,2 -> bc; 1,3 -> bd; 1,4 -> be
template <typename BaseGraph,
          typename BaseNode = boost::graph_traits<BaseGraph>,
          typename EdgeFilter = NoFilter>
RiGraph<BaseNode>
buildRiGraph(const BaseGraph &g, const BaseNode start, const EdgeFilter &edgeFilter = {})
{
    using namespace boost;

    using RIG = RiGraph<BaseNode>;
    using Vertex = graph_traits<RIG>::vertex_descriptor;
    using FilteredGraph = filtered_graph<BaseGraph, EdgeFilter>;

    RIG out;

    struct RiGraphBuilderVis : default_bfs_visitor
    {
        using V = graph_traits<FilteredGraph>::vertex_descriptor;
        using E = graph_traits<FilteredGraph>::edge_descriptor;

        RiGraphBuilderVis(RIG &o, const BaseNode start) : default_bfs_visitor(), out{o}
        {
            // one EBG node -> two NBG nodes
            // add start as a first NBG node
            curVertex = add_vertex(start, out);
            vmap[start].first = curVertex;
        }

        // before iterating edges
        void examine_vertex(const V u, const FilteredGraph &)
        {
            BOOST_ASSERT(vmap.contains(u));
            auto &m = vmap[u];
            // add second NBG node and connect it with the first
            m.second = add_vertex(u, out);
            // NOTE: EBG node itself will have 0 weight in NBG as only transition edges from this
            // NBG node will contain EBG node's weight
            add_edge(m.first, m.second, EdgeWeight{0}, out);
            curVertex = m.second;
        }

        // new vertex and edge
        void tree_edge(const E e, const FilteredGraph &fg)
        {
            // add first NBG node
            auto tv = target(e, fg);
            Vertex newVertex = add_vertex(tv, out);
            vmap[tv].first = newVertex;
            // NBG's transition edge will contain EBG's node weight + turn penalty
            add_edge(curVertex, newVertex, get(edge_weight, fg, e), out);
        }

        // new edge only
        void non_tree_edge(const E e, const FilteredGraph &fg)
        {
            // connect cur node to a previously added first NBG node
            // NBG's transition edge will contain EBG's node weight + turn penalty
            add_edge(curVertex, vmap[target(e, fg)].first, get(edge_weight, fg, e), out);
        }

        RIG &out;
        std::unordered_map<V, std::pair<Vertex, Vertex>> vmap; // EBG-to-NBG vertex mappings
        Vertex curVertex;
    };

    FilteredGraph fg{g, edgeFilter};
    // traverse and build a subgraph
    breadth_first_search(fg, start, visitor(RiGraphBuilderVis{out, start}));

    return out;
}

// Route inspection (directed Chinese Postman Problem) solver
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
Path<Vertex> routeInspectionImpl(Graph &g, const Vertex s)
{
    BOOST_ASSERT_MSG(isStronglyConnectedGraph(g), "Graph is not strongly connected\n");

    // collect node degree deltas
    auto deltas = collectNodeDegreeDeltas(g);

    // check whether graph is already Eulerian
    if (isEulerianGraph(deltas))
    {
        if (auto circuit = findEulerianCircuit(g, s); !circuit.empty())
        {
            // Eulerian circuit found -> done
            return circuit;
        }
    }

    // Eulerian circuit not found -> augment graph and retry
    if (augmentImbalancedGraph(g, deltas) && isEulerianGraph(deltas))
    {
        // graph is now Eulerian -> find circuit
        if (auto circuit = findEulerianCircuit(g, s); !circuit.empty())
        {
            // Eulerian circuit found -> done
            return circuit;
        }
    }

    // route inspection search was unsuccessful
    return {};
}

//-------------------------------------------------------------------------------------------------
// Polygon restriction utils
//-------------------------------------------------------------------------------------------------

namespace bg = boost::geometry;

using Point = bg::model::d2::point_xy<decltype(util::Coordinate::lon)::value_type>;
using Polygon = bg::model::polygon<Point>;

// Converts util::Coordinate to Point
inline Point toPoint(const util::Coordinate c) noexcept
{
    return Point{c.lon.__value, c.lat.__value};
}

// Creates a Polygon from a list of points
inline Polygon createPolygon(const std::vector<util::Coordinate> &points,
                             const double areaLimit = 0)
{
    BOOST_ASSERT(points.size() > 2 && points.front() == points.back());

    Polygon polygon;
    for (auto p : points)
    {
        bg::append(polygon, toPoint(p));
    }

    if (areaLimit > 0)
    {
        // TODO: consider using util::computeArea
        if (auto a = bg::area(polygon); a > areaLimit)
        {
            throw util::exception{"Polygon area is too big: " + std::to_string(a)};
        }
    }

    return polygon;
}

// Returns true if coordinate is inside a polygon (polygon edges count as "outside")
inline bool isInsidePolygon(const Polygon &polygon, const Point point)
{
    return bg::within(point, polygon);
}

// BaseGraph edge filter based on the area inside a polygon
struct PolygonFilter
{
    template <typename BaseEdge> bool operator()(const BaseEdge &) const
    {
        // TODO fetch edge endpoint
        return isInsidePolygon(*polygon, Point{});
    }
    const Polygon *polygon{nullptr};
};

} // namespace detail

/**
 * @brief Extracts original NodeIDs from the resulting path and creates final route result.
 *
 * @tparam Graph RI graph type used to create a closed path
 * @tparam Vertex graph's vertex type
 * @param g RI graph used to create a closed path
 * @param path resulting closed path
 * @return std::vector<NodeID> final route result
 */
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
std::vector<NodeID> prepareFinalRoute(const Graph &g, const detail::Path<Vertex> &path)
{
    if (path.empty())
    {
        util::Log(logDEBUG) << "Resulting path is empty";
        return {};
    }

    const auto toNodeID = [&g](const Vertex v)
    {
        auto baseNode = get(boost::vertex_name, g, v);
        return static_cast<NodeID>(baseNode);
    };

    std::vector<NodeID> route;
    route.reserve(path.size());

    auto it = path.cbegin();
    route.emplace_back(toNodeID(*it));
    ++it;
    for (; it != path.cend(); ++it)
    {
        auto n = toNodeID(*it);
        // NBG based RiGraph has two nodes per EBG node, so we ignore duplication
        if (route.back() != n)
        {
            route.emplace_back(n);
        }
    }

    return route;
}

/**
 * @brief Runs route inspection algorithm on a directed edge-based graph and returns resulting
 * closed path.
 *
 * @tparam EdgeBasedGraph directed edge-based graph type
 * @param graph directed edge-based graph
 * @param source starting NodeID for a node inside the graph
 * @param polygonPoints list of polygon points limiting route inspection area
 * @return std::vector<NodeID> resulting closed path as a list of node IDs (empty if not found)
 */
template <typename EdgeBasedGraph>
std::vector<NodeID> routeInspection(const EdgeBasedGraph &graph,
                                    const NodeID source,
                                    const std::vector<util::Coordinate> &polygonPoints = {})
{
    using namespace util;
    using namespace detail;

    using BaseGraph = BglGraphAdaptor<EdgeBasedGraph>;
    using BaseNode = boost::graph_traits<BaseGraph>::vertex_descriptor;
    using RIG = RiGraph<BaseNode>;
    using Vertex = boost::graph_traits<RIG>::vertex_descriptor;

    static constexpr double MAX_POLYGON_AREA = 0;

    // verify input
    const auto s = static_cast<BaseNode>(source);
    std::optional<Polygon> polygon;
    if (!polygonPoints.empty())
    {
        try
        {
            polygon = createPolygon(polygonPoints, MAX_POLYGON_AREA);
            // if (!isInsidePolygon(polygon, toPoint(start)))
            // {
            //     throw exception{"Start point should be inside the polygon"};
            // }
        }
        catch (const util::exception &e)
        {
            Log(logDEBUG) << "Input polygon is rejected: " << e.what();
            return {};
        }
    }

    // prepare graph data
    BaseGraph baseGraph{graph};
    RIG rig;

    if (polygon)
    {
        const PolygonFilter f{&polygon.value()};
        rig = buildRiGraph(baseGraph, s, f);
    }
    else
    {
        // no polygon provided
        rig = buildRiGraph(baseGraph, s);
    }

    // verify resulting graph is strongly connected
    if (!isStronglyConnectedGraph(rig))
    {
        Log(logDEBUG) << "Resulting RiGraph is not strongly connected";
        return {};
    }

    // run route inspection from the source (0) vertex
    const auto path = routeInspectionImpl(rig, Vertex{0});

    return prepareFinalRoute(rig, path);
}

} // namespace osrm::engine::routing_algorithms

#endif /* OSRM_ROUTE_INSPECTION_HPP */
