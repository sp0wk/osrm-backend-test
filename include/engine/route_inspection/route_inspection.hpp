#ifndef OSRM_ROUTE_INSPECTION_HPP
#define OSRM_ROUTE_INSPECTION_HPP

#include "engine/datafacade.hpp"
#include "engine/route_inspection/input_graph_adaptors.hpp"
#include "util/coordinate.hpp"
#include "util/exception.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/range/adaptors.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <optional>
#include <queue>
#include <stack>
#include <tuple>
#include <vector>

namespace osrm::engine::route_inspection
{

namespace detail
{

//-------------------------------------------------------------------------------------------------
// Common types
//-------------------------------------------------------------------------------------------------

// Main graph types (BGL based) used for route inspection implementation
using RiGraph = boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    boost::property<boost::vertex_name_t, NodeID>,
    boost::property<boost::edge_name_t, EdgeID, boost::property<boost::edge_weight_t, EdgeWeight>>>;

using Vertex = boost::graph_traits<RiGraph>::vertex_descriptor;
using Edge = boost::graph_traits<RiGraph>::edge_descriptor;

// Graph edge hasher
struct EdgeHash
{
    std::size_t operator()(const Edge &e) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, source(e, g));
        boost::hash_combine(seed, target(e, g));
        return seed;
    }

    const RiGraph &g;
};

// Represents a sequence of nodes (vertices)
using Path = std::vector<Vertex>;

using NodeDegreeDelta = int16_t;
// Stores difference between incoming (negative) and outgoing (positive) edges for nodes where array
// index is node id
using NodeDegreeDeltaArray = std::vector<NodeDegreeDelta>;

// Represents shortest path and its total cost
struct ShortestPath
{
    Path path;
    EdgeWeight cost{INVALID_EDGE_WEIGHT};
};

// Represents shortest paths between all sources/targets
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
inline bool isStronglyConnectedGraph(const RiGraph &g)
{
    using namespace boost;
    // TODO consider using TarjanSCC from osrm::util instead
    std::vector<int> component(num_vertices(g));
    auto num =
        strong_components(g, make_iterator_property_map(component.begin(), get(vertex_index, g)));
    return num == 1;
}

// Collects edge degree delta for all nodes in the graph
inline NodeDegreeDeltaArray collectNodeDegreeDeltas(const RiGraph &g)
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
inline bool isEulerianGraph(const RiGraph &g)
{
    return isEulerianGraph(collectNodeDegreeDeltas(g));
}

// Finds Eulerian circuit on directed Eulerian graph using Hierholzer’s algorithm
inline Path findEulerianCircuit(const RiGraph &g, const Vertex source)
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
template <typename Graph>
void shortestPaths(const Graph &g,
                   const Vertex s,
                   std::vector<Vertex> &preds,
                   std::vector<EdgeWeight> &dists)
{
    boost::dijkstra_shortest_paths(g,
                                   s,
                                   boost::predecessor_map(preds.data())
                                       .distance_map(dists.data())
                                       .distance_inf(INVALID_EDGE_WEIGHT));
}

// Extracts shortest paths from s to t using result of shortestPaths()
Path extractPath(const std::vector<Vertex> &preds, const Vertex s, const Vertex t)
{
    Path result;
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

// Finds shortest paths from source s to target t
inline ShortestPath shortestPath(const RiGraph &g, const Vertex s, const Vertex t)
{
    using namespace boost;

    struct Terminator : default_dijkstra_visitor
    {
        Terminator(const Vertex t) : default_dijkstra_visitor(), target{t} {}

        void examine_vertex(const Vertex v, const RiGraph &) const
        {
            if (v == target)
            {
                throw v;
            }
        }

        Vertex target;
    };

    // prepare maps for predecessors and distances
    std::vector<Vertex> preds(num_vertices(g));
    std::vector<EdgeWeight> dists(num_vertices(g), INVALID_EDGE_WEIGHT);

    try
    {
        dijkstra_shortest_paths(g,
                                s,
                                predecessor_map(preds.data())
                                    .distance_map(dists.data())
                                    .distance_inf(INVALID_EDGE_WEIGHT)
                                    .visitor(Terminator{t}));
    }
    catch (const Vertex &)
    {
        // path to target is found
        return {extractPath(preds, s, t), dists[t]};
    }

    // no path found
    return {};
}

// Finds shortest paths from a source vertex to all target vertices
inline std::vector<ShortestPath>
oneToMany(const RiGraph &g, const Vertex source, const std::vector<Vertex> &targets)
{
    // prepare maps for predecessors and distances
    std::vector<Vertex> preds(num_vertices(g));
    std::vector<EdgeWeight> dists(num_vertices(g), INVALID_EDGE_WEIGHT);

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
inline PathMatrix
manyToMany(const RiGraph &g, const std::vector<Vertex> &sources, const std::vector<Vertex> &targets)
{
    PathMatrix m;
    // allocate matrix
    m.rows.resize(sources.size());
    for (const auto &[idx, s] : boost::adaptors::index(sources))
    {
        auto &row = m.rows[idx];
        row.source = s;
        row.columns.resize(targets.size());
    }

    // parallelize multiple oneToMany calls
    tbb::parallel_for(tbb::blocked_range<size_t>(0UL, sources.size()),
                      [&](const tbb::blocked_range<size_t> &r)
                      {
                          for (auto sIdx = r.begin(); sIdx != r.end(); ++sIdx)
                          {
                              // calc 1-to-m routes
                              auto paths = oneToMany(g, sources[sIdx], targets);
                              BOOST_ASSERT(targets.size() == paths.size());
                              // populate matrix
                              auto &row = m.rows[sIdx];
                              for (const auto &[tIdx, t] : boost::adaptors::index(targets))
                              {
                                  auto &col = row.columns[tIdx];
                                  col.target = t;
                                  col.path = std::move(paths[tIdx]);
                              }
                          }
                      });

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
            result.emplace_back(vertexToIdxMap[s], vertexToIdxMap[t], flow);
        }
    }

    return result;
}

// Add deficit edges to imbalanced graph through solving min-cost flow problem to make graph
// Eulerian
inline bool augmentImbalancedGraph(RiGraph &g, NodeDegreeDeltaArray &deltas)
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
                auto [dupEdge, isNew] = add_edge(u, v, g);
                BOOST_ASSERT(isNew);
                put(boost::edge_name, g, dupEdge, get(boost::edge_name, g, baseEdge));
                put(boost::edge_weight, g, dupEdge, get(boost::edge_weight, g, baseEdge));
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
// - EBG nodes/edges are mapped directly to corresponding NBG nodes/edges
//   Example:
//      EBG:           ->           NBG:
//
//              ↑                      o 3
//              | 3                    ↑
//      2 <----- ----> 4    ->    2 o  |  o 4
//              ↑ 1                  \ | /
//              |                     \|/
//                                     o 1
template <typename BaseGraph, typename EdgeFilter = NoFilter>
auto buildRiGraph(const BaseGraph &g, const NodeID start, const EdgeFilter &edgeFilter = {})
{
    using namespace boost;

    BOOST_CONCEPT_ASSERT((InputGraphConcept<BaseGraph>));

    RiGraph out;

    // helper
    auto const addEdge =
        [&out](const Vertex u, const Vertex v, const EdgeID edgeName, const EdgeWeight weight)
    {
        auto [e, isNew] = add_edge(u, v, out);
        BOOST_ASSERT(isNew);
        put(edge_name, out, e, edgeName);
        put(edge_weight, out, e, weight);
    };

    // BFS traversal with filtering
    std::unordered_map<NodeID, Vertex> vmap; // old-new vertex mappings
    std::queue<NodeID> q;

    // add first vertex
    vmap[start] = add_vertex(start, out);
    q.push(start);

    while (!q.empty())
    {
        const auto u = q.front();
        q.pop();
        const auto curVertex = vmap[u];

        for (const auto e : g.GetOutEdgeRange(u))
        {
            if (!edgeFilter(e))
            {
                continue;
            }

            auto v = g.GetTarget(e);
            auto w = g.GetEdgeWeight(u, e);

            if (!vmap.contains(v))
            {
                // unvisited -> new vertex and edge
                Vertex newVertex = add_vertex(v, out);
                vmap[v] = newVertex;
                // new edge
                addEdge(curVertex, newVertex, e, w);
                q.push(v);
            }
            else
            {
                // already visited -> new edge only
                auto targetVertex = vmap[v];
                addEdge(curVertex, targetVertex, e, w);
            }
        }
    }

    return out;
}

// Greedily DFS traverse RI graph to create a circuit with possible disjoints
inline auto getDfsCircuitGreedy(const RiGraph &g, const Vertex start)
{
    using namespace boost;

    using SortedEdges = std::vector<Edge>;
    using EdgeIt = SortedEdges::const_iterator;
    using EdgeState = std::pair<EdgeIt, SortedEdges>;

    // prepare initial state
    const auto nbOfVertices = num_vertices(g);
    std::vector<EdgeState> unusedEdges(nbOfVertices);
    std::unordered_set<Vertex> visitedNodes;
    std::stack<Vertex> st;
    bool uniqueParent = false;

    // populate/sort edges for cheapest-first visit order
    for (auto v : make_iterator_range(vertices(g)))
    {
        auto &vEdges = unusedEdges[v].second;
        vEdges.reserve(out_degree(v, g));
        for (auto e : make_iterator_range(out_edges(v, g)))
        {
            vEdges.emplace_back(e);
        }
        // sort edges by their weight
        std::stable_sort(vEdges.begin(),
                         vEdges.end(),
                         [&g](auto a, auto b)
                         { return get(edge_weight, g, a) < get(edge_weight, g, b); });
        // set initial iterator
        unusedEdges[v].first = vEdges.cbegin();
    }

    // prepare result
    Path circuit;
    std::unordered_set<Vertex> disjointInNodes;
    std::unordered_set<Vertex> disjointOutNodes;
    circuit.reserve(nbOfVertices);
    disjointInNodes.reserve(nbOfVertices / 2);
    disjointOutNodes.reserve(nbOfVertices / 2);

    // add start vertex
    visitedNodes.emplace(start);

    // find circuit
    st.push(start);
    while (!st.empty())
    {
        auto v = st.top();
        const auto &vEdges = unusedEdges[v].second;
        auto eEnd = vEdges.cend();
        std::optional<EdgeIt> nextEdge;
        if (auto e = unusedEdges[v].first; e != eEnd)
        {
            for (; e != eEnd; ++e)
            {
                // check that target vertex is unvisited
                if (!visitedNodes.contains(target(*e, g)))
                {
                    nextEdge.emplace(e);
                    break;
                }
            }

            // all unvisited edges point to already visited nodes
            if (!nextEdge && !visitedNodes.contains(v))
            {
                if (out_degree(v, g) == 1)
                {
                    // add unique successor to circuit
                    auto u = target(*vEdges.cbegin(), g);
                    circuit.emplace_back(u);
                }
                else
                {
                    // out-disjoint node to be connected later
                    disjointOutNodes.emplace(v);
                }
            }
        }

        if (nextEdge)
        {
            if (uniqueParent)
            {
                // ensure edge to this unique parent exists
                circuit.emplace_back(v);
                uniqueParent = false;
            }
            // take next edge v->u
            auto e = *nextEdge;
            auto u = target(*e, g);
            unusedEdges[v].first = ++e;
            st.push(u);
        }
        else
        {
            uniqueParent = in_degree(v, g) == 1;
            if (!uniqueParent)
            {
                // in-disjoint node to be connected later
                disjointInNodes.emplace(v);
            }
            circuit.emplace_back(v);
            st.pop();
        }

        // mark node as visited
        visitedNodes.emplace(v);
    }

    std::reverse(circuit.begin(), circuit.end());

    return std::make_tuple(
        std::move(circuit), std::move(disjointInNodes), std::move(disjointOutNodes));
}

// Greedily examine graph and collect optimal set of edges for route inspection
inline auto collectMinCostEdgeSet(const RiGraph &g, const Vertex start)
{
    using namespace boost;

    using ResultSet = std::unordered_set<Edge, EdgeHash>;

    ResultSet usedEdges{0, EdgeHash{g}};
    usedEdges.reserve(num_edges(g));

    // 1) Get DFS circuit with disjoints
    auto [circuit, disjointInNodes, disjointOutNodes] = getDfsCircuitGreedy(g, start);
    BOOST_ASSERT(circuit.size() > 1);

    // 2) Process regular edges from the circuit
    for (auto it = std::next(circuit.cbegin()); it != circuit.cend(); ++it)
    {
        auto u = *std::prev(it);
        auto v = *it;
        auto [e, exists] = edge(u, v, g);
        if (exists)
        {
            usedEdges.emplace(e);
            // fix in-disjoint with this edge (if any)
            disjointInNodes.erase(v);
        }
    }

    if (disjointInNodes.empty() && disjointOutNodes.empty())
    {
        // no disjoints
        return usedEdges;
    }

    // 3) Find shortest paths from/to start
    std::vector<Vertex> preds(num_vertices(g));
    std::vector<EdgeWeight> dists(num_vertices(g), INVALID_EDGE_WEIGHT);
    std::vector<Vertex> revPreds(num_vertices(g));
    std::vector<EdgeWeight> revDists(num_vertices(g), INVALID_EDGE_WEIGHT);
    // for in-disjoints (start -> v)
    shortestPaths(g, start, preds, dists);
    // for out-disjoints (v -> start)
    auto revg = make_reverse_graph(g);
    shortestPaths(revg, start, revPreds, revDists);

    // 4) Fix out-disjoints by connecting them to start using best edge
    for (auto u : disjointOutNodes)
    {
        BOOST_ASSERT(out_degree(u, g) > 1);
        BOOST_ASSERT(revDists[u] != INVALID_EDGE_WEIGHT);
        Vertex v = revPreds[u]; // pred indicates best edge
        auto [e, exists] = edge(u, v, g);
        BOOST_ASSERT(exists);
        usedEdges.emplace(e);
        // fix in-disjoint with this edge (if any)
        disjointInNodes.erase(v);
    }

    // 5) Fix leftover in-disjoints by connecting start to them using best edge
    for (auto v : disjointInNodes)
    {
        BOOST_ASSERT(in_degree(v, g) > 1);
        BOOST_ASSERT(dists[v] != INVALID_EDGE_WEIGHT);
        Vertex u = preds[v]; // pred indicates best edge
        auto [e, exists] = edge(u, v, g);
        BOOST_ASSERT(exists);
        usedEdges.emplace(e);
    }

    return usedEdges;
}

// Removes edges from a graph which can be replaced with a cheaper shortest path
inline auto collectCostlyEdges(RiGraph &g)
{
    auto const isShortestPath = [&g](const Edge e)
    {
        auto u = source(e, g);
        auto v = target(e, g);
        const auto sp = shortestPath(g, u, v);
        const auto &p = sp.path;
        BOOST_ASSERT(!p.empty());
        // checks that shortest path is edge itself
        return p.size() == 2 && p[0] == u && p[1] == v;
    };

    std::vector<Edge> costlyEdges;
    costlyEdges.reserve(num_edges(g) / 2);

    for (const auto e : make_iterator_range(edges(g)))
    {
        if (!isShortestPath(e))
        {
            // edge is not the shortest path between u->v
            costlyEdges.emplace_back(e);
        }
    }

    return costlyEdges;
}

// Optimize RiGraph connectivity to minimize the number of transition edges
void optimizeRiGraph(RiGraph &g)
{
    using namespace boost;

    // 1) Preprocessing to prune edges which are not shortest paths
    const auto costlyEdges = collectCostlyEdges(g);
    for (const auto e : costlyEdges)
    {
        remove_edge(e, g);
    }

    // 2) Collect optimal edges
    Vertex start{0};
    auto usedEdges = collectMinCostEdgeSet(g, start);

    // 3) Collect unused edges
    std::vector<Edge> unusedEdges;
    unusedEdges.reserve(num_edges(g) - usedEdges.size());
    for (const auto e : make_iterator_range(edges(g)))
    {
        if (!usedEdges.contains(e))
        {
            // edge is estimated as not optimal -> remove
            unusedEdges.emplace_back(e);
        }
    }

    // 4) Actual edge removal
    for (const auto e : unusedEdges)
    {
        remove_edge(e, g);
    }
}

// Route inspection (directed Chinese Postman Problem) solver
inline Path routeInspectionImpl(RiGraph &g, const Vertex s)
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
    else
    {
        // non-Eulerian graph -> augment graph and retry
        if (augmentImbalancedGraph(g, deltas) && isEulerianGraph(deltas))
        {
            // graph is now Eulerian -> find circuit
            if (auto circuit = findEulerianCircuit(g, s); !circuit.empty())
            {
                // Eulerian circuit found -> done
                return circuit;
            }
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

template <typename DataFacade>
util::Coordinate getNodeEndpoint(const DataFacade &facade, const NodeID u)
{
    auto gi = facade.GetGeometryIndex(u);
    if (gi.forward)
    {
        auto geom = facade.GetUncompressedForwardGeometry(gi.id);
        BOOST_ASSERT(!geom.empty());
        auto endNode = *std::crbegin(geom);
        return facade.GetCoordinateOfNode(endNode);
    }
    else
    {
        auto geom = facade.GetUncompressedReverseGeometry(gi.id);
        BOOST_ASSERT(!geom.empty());
        auto endNode = *std::crbegin(geom);
        return facade.GetCoordinateOfNode(endNode);
    }
}

// Input graph edge filter based on the area inside a polygon
template <typename DataFacade> struct PolygonFilter
{
    bool operator()(const EdgeID &e) const
    {
        auto p = getNodeEndpoint(facade, facade.GetTarget(e));
        return isInsidePolygon(*polygon, Point{p});
    }

    const DataFacade &facade;
    const Polygon *polygon{nullptr};
};

} // namespace detail

/**
 * @brief Extracts original NodeIDs from the resulting path and creates final route result.
 *
 * @param g RI graph used to create a closed path
 * @param path resulting closed path
 * @return std::vector<NodeID> final route result
 */
std::vector<NodeID> prepareFinalRoute(const detail::RiGraph &g, const detail::Path &path)
{
    if (path.empty())
    {
        util::Log(logDEBUG) << "Resulting path is empty";
        return {};
    }

    const auto toNodeID = [&g](const detail::Vertex v) { return get(boost::vertex_name, g, v); };

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
            // TODO: improve final route by replacing costly edges with shortest paths (which
            // possibly go beyond target polygon)
            route.emplace_back(n);
        }
    }

    return route;
}

/**
 * @brief Runs route inspection algorithm on a directed edge-based graph and returns resulting
 * closed path.
 *
 * @tparam Algorithm routing algorithm type
 * @param facade Algorithm-specific DataFacade representing input graph
 * @param start PhantomNode matching start location
 * @param polygonPoints list of polygon points limiting route inspection area
 * @return std::vector<NodeID> resulting closed path as a list of node IDs (empty if not found)
 */
template <typename Algorithm>
std::vector<NodeID> routeInspection(const DataFacade<Algorithm> &facade,
                                    const PhantomNode &start,
                                    const std::vector<util::Coordinate> &polygonPoints = {})
{
    using namespace util;
    using namespace detail;

    using BaseGraph = AlgorithmBasedInputGraphWrapper<Algorithm>;

    static constexpr double MAX_POLYGON_AREA = 0;

    // verify input
    const NodeID s =
        start.IsValidForwardSource() ? start.forward_segment_id.id : start.reverse_segment_id.id;
    std::optional<Polygon> polygon;
    if (!polygonPoints.empty())
    {
        try
        {
            polygon = createPolygon(polygonPoints, MAX_POLYGON_AREA);
            if (!isInsidePolygon(*polygon, toPoint(start.location)))
            {
                throw exception{"Start point should be inside the polygon"};
            }
        }
        catch (const util::exception &e)
        {
            Log(logDEBUG) << "Input polygon is rejected: " << e.what();
            return {};
        }
    }

    // prepare graph data
    BaseGraph baseGraph{facade};
    RiGraph rig;

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

    optimizeRiGraph(rig);

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

} // namespace osrm::engine::route_inspection

#endif /* OSRM_ROUTE_INSPECTION_HPP */
