#ifndef OSRM_ROUTE_INSPECTION_HPP
#define OSRM_ROUTE_INSPECTION_HPP

#include "util/bgl_graph_adaptor.hpp"
#include "util/log.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/range/adaptors.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
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
using BglGraph = util::BglNodeBasedDynamicGraph;
using Vertex = BglGraph::Vertex;
using Edge = BglGraph::Edge;
using Weight = BglGraph::Weight;

// Represents a sequence of nodes (vertices)
using Path = std::vector<Vertex>;

using NodeDegreeDelta = int16_t;
// Stores difference between incoming (negative) and outgoing (positive) edges for nodes where array
// index is node id
using NodeDegreeDeltaArray = std::vector<NodeDegreeDelta>;

// Represents shorttest path and its total cost
struct ShortestPath
{
    Path path;
    Weight cost{INVALID_EDGE_WEIGHT};
};

struct PathMatrix
{
    struct Column
    {
        Vertex target;
        ShortestPath path;
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
inline bool isStronglyConnectedGraph(const BglGraph &g)
{
    using namespace boost;
    // TODO consider using TarjanSCC from osrm::util instead
    std::vector<int> component(num_vertices(g));
    auto num =
        strong_components(g, make_iterator_property_map(component.begin(), get(vertex_index, g)));
    return num == 1;
}

// Collects edge degree delta for all nodes in the graph
inline NodeDegreeDeltaArray collectNodeDegreeDeltas(const BglGraph &g)
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
inline bool isEulerianGraph(const BglGraph &g)
{
    return isEulerianGraph(collectNodeDegreeDeltas(g));
}

// Finds Eulerian circuit on directed Eulerian graph using Hierholzer’s algorithm
inline Path findEulerianCircuit(const BglGraph &g, const Vertex source)
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
    Path circuit;
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
inline void shortestPaths(const BglGraph &g,
                          const Vertex s,
                          std::vector<Vertex> &preds,
                          std::vector<Weight> &dists)
{
    BOOST_ASSERT(s < num_vertices(g));
    boost::dijkstra_shortest_paths(g,
                                   s,
                                   boost::predecessor_map(preds.data())
                                       .distance_map(dists.data())
                                       .distance_inf(INVALID_EDGE_WEIGHT));
}

// Extracts shortest paths from s to t using result of shortestPaths()
inline Path extractPath(const std::vector<Vertex> &preds, const Vertex s, const Vertex t)
{
    Path result;
    // initially reserve x2 of nodes between s and t
    result.reserve(std::max(16U, (std::max(s, t) - std::min(s, t)) * 2));
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
inline std::vector<ShortestPath>
oneToMany(const BglGraph &g, const Vertex source, const std::vector<Vertex> &targets)
{
    // prepare maps for predecessors and distances
    std::vector<Vertex> preds(num_vertices(g));
    std::vector<Weight> dists(num_vertices(g), INVALID_EDGE_WEIGHT);

    // calculate shortest paths from source to all vertices
    shortestPaths(g, source, preds, dists);

    // extract paths for all targets
    std::vector<ShortestPath> paths;
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
inline PathMatrix manyToMany(const BglGraph &g,
                             const std::vector<Vertex> &sources,
                             const std::vector<Vertex> &targets)
{
    PathMatrix m;
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
inline auto buildMcfGraph(const PathMatrix &pathMatrix,
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
inline MinCostFlow solveMinCostFlow(const PathMatrix &pathMatrix,
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
            std::cout << source(e, g) << " -> " << target(e, g) << " | flow " << flow << " | cost "
                      << get(edge_weight, g)[e] << "\n";

            result.emplace_back(vertexToIdxMap[s], vertexToIdxMap[t], flow);
        }
    }

    return result;
}

// Add deficit edges to imbalanced graph through solving min-cost flow problem to make graph
// Eulerian
inline bool augmentImbalancedGraph(BglGraph &g, NodeDegreeDeltaArray &deltas)
{
    // collect surplus (delta < 0) and deficit (delta > 0) nodes
    // NOTE: normally for MCF nodes with delta > 0 become surplus nodes but, for convenience, we
    // swap surplus and deficit nodes to avoid inversing shortest paths and resulting nodes
    std::vector<Vertex> surplusNodes, deficitNodes;
    auto totalSurplus{0U}, totalDeficit{0U};
    // assume only half of nodes are either surplus or deficit ones
    surplusNodes.reserve(std::min(16UL, deltas.size() / 4));
    deficitNodes.reserve(std::min(16UL, deltas.size() / 4));
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
                Weight weight = g.GetWeight(g.GetEdge(u, v));
                add_edge(u, v, weight, g);
                // adjust degree deltas
                ++deltas[u];
                --deltas[v];
            }
        }
    }

    return true;
}

//-------------------------------------------------------------------------------------------------
// Main route inspection finder
//-------------------------------------------------------------------------------------------------

// Route inspection (directed Chinese Postman Problem) solver
inline Path routeInspectionImpl(BglGraph &g, const Vertex s)
{
    // it's assumed graph is strongly connnected
    BOOST_ASSERT_MSG(isStronglyConnectedGraph(g), "Graph is not strongly connected\n");

    if (s >= num_vertices(g))
    {
        util::Log(logDEBUG) << "Invalid source vertex s=" << s
                            << " with num_vertices=" << num_vertices(g);
        return {};
    }

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

} // namespace detail

/**
 * @brief Runs route inspection algorithm on a directed node-based graph and returns resulting
 * closed path.
 *
 * @param graph directed node-based dynamic graph
 * @param source starting NodeID for a node inside the graph
 * @return std::vector<NodeID> resulting closed path as a list of node IDs (empty if not found)
 */
inline std::vector<NodeID> routeInspection(util::NodeBasedDynamicGraph &graph, const NodeID source)
{
    using namespace util;
    using BglGraph = detail::BglGraph;

    // prepare input data
    BglGraph g{graph};
    const auto s = static_cast<detail::Vertex>(source);

    // run route inspection
    const auto path = detail::routeInspectionImpl(g, s);

    // prepare final route result
    std::vector<NodeID> route;
    route.reserve(path.size());
    std::transform(path.cbegin(),
                   path.cend(),
                   std::back_inserter(route),
                   [&](const detail::Vertex v) { return static_cast<NodeID>(v); });
    return route;
}

} // namespace osrm::engine::routing_algorithms

#endif /* OSRM_ROUTE_INSPECTION_HPP */
