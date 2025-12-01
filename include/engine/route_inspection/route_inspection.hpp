#ifndef OSRM_ROUTE_INSPECTION_HPP
#define OSRM_ROUTE_INSPECTION_HPP

#include "engine/datafacade.hpp"
#include "engine/datafacade/datafacade_base.hpp"
#include "engine/route_inspection/input_graph_adaptors.hpp"

#include "util/coordinate.hpp"
#include "util/log.hpp"
#include "util/polygon.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
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
#include <utility>
#include <vector>

namespace osrm::engine::route_inspection
{

namespace detail
{

// convenience function to get node's start/end coors
inline auto getNodeEndpoints(const datafacade::BaseDataFacade &facade, const NodeID node)
{
    util::Coordinate c1, c2;
    auto gi = facade.GetGeometryIndex(node);
    if (gi.forward)
    {
        auto geom = facade.GetUncompressedForwardGeometry(gi.id);
        BOOST_ASSERT(!geom.empty());
        c1 = facade.GetCoordinateOfNode(geom.front());
        c2 = facade.GetCoordinateOfNode(geom.back());
    }
    else
    {
        auto geom = facade.GetUncompressedReverseGeometry(gi.id);
        BOOST_ASSERT(!geom.empty());
        c1 = facade.GetCoordinateOfNode(geom.front());
        c2 = facade.GetCoordinateOfNode(geom.back());
    }

    return std::make_pair(c1, c2);
}

//-------------------------------------------------------------------------------------------------
// Common types
//-------------------------------------------------------------------------------------------------

// Main graph types (BGL based) used for route inspection implementation
using RiGraphBase = boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    boost::property<boost::vertex_name_t, NodeID>,
    boost::property<boost::edge_name_t, EdgeID, boost::property<boost::edge_weight_t, EdgeWeight>>>;

using Vertex = boost::graph_traits<RiGraphBase>::vertex_descriptor;
using Edge = boost::graph_traits<RiGraphBase>::edge_descriptor;

class RiGraph : public RiGraphBase
{
  public:
    explicit RiGraph(const datafacade::BaseDataFacade &facade) : RiGraphBase(), facade{&facade} {}

    const datafacade::BaseDataFacade &GetFacade() const noexcept { return *facade; }

    // TODO replace with actual GeoJSON

    void logVertex(const Vertex u, std::string_view label = {}) const
    {
        const auto [c1, c2] = getVertexCoords(u);
        auto node = get(boost::vertex_name, *this, u);
        std::cout << label << " vertex " << u << " (" << node << "): [[" << c1.lon << "," << c1.lat
                  << "],[" << c2.lon << "," << c2.lat << "]]," << std::endl;
    }

    void logVertexStart(const Vertex u, std::string_view label = {}) const
    {
        const auto [c, _] = getVertexCoords(u);
        auto node = get(boost::vertex_name, *this, u);
        std::cout << label << " vertex start " << u << " (" << node << "): [" << c.lon << ","
                  << c.lat << "]" << std::endl;
    }

    void logVertexEnd(const Vertex u, std::string_view label = {}) const
    {
        const auto [_, c] = getVertexCoords(u);
        auto node = get(boost::vertex_name, *this, u);
        std::cout << label << " vertex end " << u << " (" << node << "): [" << c.lon << "," << c.lat
                  << "]" << std::endl;
    }

    void logEdge(const Edge e, std::string_view label = {}) const
    {
        const auto u = source(e, *this);
        const auto v = target(e, *this);
        const auto [c1, c2] = getVertexCoords(u);
        const auto [c3, c4] = getVertexCoords(v);
        BOOST_ASSERT(c2 == c3);
        const auto from = get(boost::vertex_name, *this, u);
        const auto to = get(boost::vertex_name, *this, v);
        std::cout << label << " edge " << u << " -> " << v << " (" << from << " -> " << to
                  << "): [[" << c1.lon << "," << c1.lat << "],[" << c2.lon << "," << c2.lat << "],["
                  << c4.lon << "," << c4.lat << "]]," << std::endl;
    }

    void logEdge(const Vertex u, const Vertex v, std::string_view label = {}) const
    {
        const auto [e, exists] = edge(u, v, *this);
        BOOST_ASSERT(exists);
        logEdge(e, label);
    };

  private:
    std::pair<util::FloatCoordinate, util::FloatCoordinate> getVertexCoords(const Vertex u) const
    {
        auto node = get(boost::vertex_name, *this, u);
        const auto [s, e] = getNodeEndpoints(GetFacade(), node);
        return std::make_pair(util::FloatCoordinate{s}, util::FloatCoordinate{e});
    }

    const datafacade::BaseDataFacade *facade{nullptr};
};

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
// Graph connectivity utils
//-------------------------------------------------------------------------------------------------

// Checks whether directed graph is strongly connected
inline bool isStronglyConnectedGraph(const RiGraph &g)
{
    using namespace boost;
    std::vector<int> component(num_vertices(g));
    auto num =
        strong_components(g, make_iterator_property_map(component.begin(), get(vertex_index, g)));
    return num == 1;
}

// RiGraph correctness verification
inline bool isValidGraph(const RiGraph &g)
{
    return num_vertices(g) >= 2 && num_edges(g) >= 2 && isStronglyConnectedGraph(g);
}

// Cut out RiGraph's vertices/edges which belong to minor SCCs and leave only 1 SCC (to ensure
// strongly connected graph)
inline void dropMinorSCCs(RiGraph &g)
{
    using namespace boost;

    std::vector<int> sccs(num_vertices(g));
    auto num = strong_components(g, make_iterator_property_map(sccs.begin(), get(vertex_index, g)));
    if (num == 1)
    {
        // only main SCC already
        return;
    }

    // main SCC is the one which contains start vertex
    const auto mainSCC = sccs[0];

    // remove minor sccs vertices
    std::vector<Vertex> toRemove;
    toRemove.reserve(sccs.size() / 2);

    for (const auto v : util::irange<std::size_t>(0, sccs.size()))
    {
        if (sccs[v] != mainSCC)
        {
            toRemove.emplace_back(static_cast<Vertex>(v));
        }
    }

    // remove vertices (and consequently edges)
    // NOTE: since we use vecS to store vertices in adjacency_list, we're iterating in reverse to
    // remove vertices in decreasing order (to not invalidate other vertex descriptors inside
    // toRemove)
    for (const auto v : adaptors::reverse(toRemove))
    {
        g.logVertex(v, "minorScc");
        clear_vertex(v, g);
        remove_vertex(v, g);
    }
}

//-------------------------------------------------------------------------------------------------
// Eulerian circuit
//-------------------------------------------------------------------------------------------------

// Collects edge degree delta for all nodes in the graph
inline NodeDegreeDeltaArray collectNodeDegreeDeltas(const RiGraph &g)
{
    NodeDegreeDeltaArray deltas(num_vertices(g));
    for (const auto e : boost::make_iterator_range(edges(g)))
    {
        deltas[source(e, g)] += 1;
        deltas[target(e, g)] -= 1;
    }
    return deltas;
}

// Checks whether graph is Eulerian based on node degree deltas
inline bool isEulerianGraph(const NodeDegreeDeltaArray &deltas)
{
    for (const auto d : deltas)
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
    for (const auto u : boost::make_iterator_range(vertices(g)))
    {
        unusedEdges[u] = out_edges(u, g).first;
    }
    std::stack<Vertex> st;
    Path circuit;
    circuit.reserve(nbOfVertices);

    // find circuit
    st.push(source);
    while (!st.empty())
    {
        const auto u = st.top();
        if (const auto e = unusedEdges[u]; e != out_edges(u, g).second)
        {
            // take next edge u->v
            const auto v = target(*e, g);
            unusedEdges[u]++;
            st.push(v);
        }
        else
        {
            circuit.emplace_back(u);
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
inline Path extractPath(const std::vector<Vertex> &preds, const Vertex s, const Vertex t)
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
        BOOST_ASSERT(dists[t] != INVALID_EDGE_WEIGHT);
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
        const auto cost = dists[t];
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

namespace mcf
{

using namespace boost;
struct McfGraphTraits
{
    using EdgeListType = listS; // NOTE: listS prevents edge_descriptor invalidation
    using VertexListType = vecS;

    using BaseTraits = adjacency_list_traits<EdgeListType, VertexListType, directedS>;

    using Weight = EdgeWeight::value_type;
    using Capacity = uint32_t;

    using VertexProperties = property<vertex_name_t, std::size_t>; // for PathMatrix indices
    using EdgeProperties =
        property<edge_weight_t,
                 Weight,
                 property<edge_capacity_t,
                          Capacity,
                          property<edge_residual_capacity_t,
                                   Capacity,
                                   property<edge_reverse_t, BaseTraits::edge_descriptor>>>>;

    using Graph =
        adjacency_list<EdgeListType, VertexListType, directedS, VertexProperties, EdgeProperties>;
};

using McfGraph = McfGraphTraits::Graph;

/**
 * @brief Builds min-cost flow graph compatible with
 * boost::successive_shortest_path_nonnegative_weights.
 *
 * @param g empty McfGraph to build
 * @param pathMatrix table/matrix of surplus and deficit nodes with respective shortest paths
 * between them where rows represent surplus nodes (sources) and columns - deficit nodes (targets).
 * @param deltas degree delta storage for all surplus/deficit nodes
 * @param maxCapacity total amount of surplus
 * @return auto super-source and super-sink vertex descriptors
 */
inline auto buildMcfGraph(McfGraph &g,
                          const PathMatrix &pathMatrix,
                          const NodeDegreeDeltaArray &deltas,
                          const McfGraphTraits::Capacity maxCapacity =
                              std::numeric_limits<McfGraphTraits::Capacity>::max())
{
    BOOST_ASSERT(num_vertices(g) == 0 && num_edges(g) == 0);
    BOOST_ASSERT(!pathMatrix.rows.empty());

    using V = graph_traits<McfGraph>::vertex_descriptor;
    using E = graph_traits<McfGraph>::edge_descriptor;
    using Cost = McfGraphTraits::Weight;
    using Cap = McfGraphTraits::Capacity;

    // retrieve property maps
    auto capacityMap = get(edge_capacity, g);
    auto weightMap = get(edge_weight, g);
    auto revMap = get(edge_reverse, g);

    // helpers
    const auto addVertex = [&g](const auto idx) { return add_vertex(idx, g); };

    const auto addEdge = [&](const V from, const V to, const Cap cap, const Cost cost)
    {
        BOOST_ASSERT(from != to && cap > 0 && cost >= 0);

        E e, rev;
        bool success;
        tie(e, success) = add_edge(from, to, g);
        BOOST_ASSERT(success);
        tie(rev, success) = add_edge(to, from, g);
        BOOST_ASSERT(success);

        capacityMap[e] = cap;
        capacityMap[rev] = 0; // reverse edge has 0 capacity
        weightMap[e] = cost;
        weightMap[rev] = -cost;
        revMap[e] = rev;
        revMap[rev] = e;
    };

    // populate graph

    const auto superSource = addVertex(-1);
    const auto superSink = addVertex(-1);

    std::unordered_map<Vertex, V> sinks; // to check already added sinks
    sinks.reserve(pathMatrix.rows[0].columns.size());
    for (const auto &[rIdx, s] : adaptors::index(pathMatrix.rows))
    {
        // insert new surplus node and connect it with super-source
        const auto sv = addVertex(rIdx);
        const auto capacity = std::abs(deltas[s.source]);
        addEdge(superSource, sv, capacity, 0);

        for (const auto &[cIdx, t] : adaptors::index(s.columns))
        {
            V tv = McfGraph::null_vertex();
            if (const auto it = sinks.find(t.target); it != sinks.cend())
            {
                // sink node was already inserted
                tv = it->second;
            }
            else
            {
                // insert new sink node and connect it to super-sink
                tv = addVertex(cIdx);
                sinks[t.target] = tv;
                // connect new deficit node with the sink
                const auto cap = std::abs(deltas[t.target]);
                addEdge(tv, superSink, cap, 0);
            }
            // connect surplus node to deficit node if path exists
            if (const auto cost = t.path.cost; cost != INVALID_EDGE_WEIGHT)
            {
                addEdge(sv, tv, maxCapacity, cost.__value);
            }
            else
            {
                util::Log(logDEBUG)
                    << "MCF: skipping unreachable path " << s.source << " -> " << t.target;
            }
        }
    }

    return std::make_pair(superSource, superSink);
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
                                    const McfGraphTraits::Capacity maxCapacity =
                                        std::numeric_limits<McfGraphTraits::Capacity>::max())
{
    BOOST_ASSERT(!pathMatrix.rows.empty());

    // build min-cost flow graph
    McfGraph g;
    const auto [superSource, superSink] = buildMcfGraph(g, pathMatrix, deltas, maxCapacity);

    // solve min-cost flow
    successive_shortest_path_nonnegative_weights(g, superSource, superSink);

    // prepare result
    auto vertexToIdxMap = get(vertex_name, g);
    auto capacityMap = get(edge_capacity, g);
    auto residualCapacityMap = get(edge_residual_capacity, g);

    MinCostFlow result;
    result.reserve(pathMatrix.rows.size());
    for (const auto e : make_iterator_range(edges(g)))
    {
        const auto s = source(e, g);
        const auto t = target(e, g);
        if (s == superSource || s == superSink || t == superSink || t == superSource)
        {
            // ignore virtual edges
            continue;
        }
        // add edge with positive flow to result
        if (const auto flow = capacityMap[e] - residualCapacityMap[e]; flow > 0)
        {
            const auto row = vertexToIdxMap[s];
            const auto col = vertexToIdxMap[t];
            BOOST_ASSERT(row < pathMatrix.rows.size() && col < pathMatrix.rows[0].columns.size());
            result.emplace_back(row, col, flow);
        }
    }

    return result;
}

} // namespace mcf

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
    const MinCostFlow mcf = mcf::solveMinCostFlow(pathMatrix, deltas, totalSurplus);
    if (mcf.empty())
    {
        util::Log(logERROR) << "MCF could not be solved";
        return false;
    }

    // duplicate edges based on flow value
    for (const auto &edgeFlow : mcf)
    {
        const auto &row = pathMatrix.rows[edgeFlow.sIdx];
        const auto &col = row.columns[edgeFlow.tIdx];
        for ([[maybe_unused]] const auto reps : util::irange(0U, edgeFlow.flow))
        {
            const auto &p = col.path.path;
            BOOST_ASSERT(p.size() > 1);
            for (auto it = std::next(p.cbegin()); it != p.cend(); ++it)
            {
                const auto u = *std::prev(it);
                const auto v = *it;
                const auto [baseEdge, exists] = edge(u, v, g);
                BOOST_ASSERT(exists);
                // duplicate edge
                const auto node = get(boost::edge_name, g, baseEdge);
                const auto weight = get(boost::edge_weight, g, baseEdge);
                const auto [dupEdge, isNew] = add_edge(u, v, g);
                BOOST_ASSERT(isNew);
                put(boost::edge_name, g, dupEdge, node);
                put(boost::edge_weight, g, dupEdge, weight);
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

// Input graph edge filter based on the area inside a polygon
template <typename DataFacade> struct PolygonFilter
{
    bool operator()(const EdgeID &e) const
    {
        const auto [_, p] = getNodeEndpoints(facade, facade.GetTarget(e));
        return polygon.Contains(p);
    }

    const DataFacade &facade;
    const util::Polygon &polygon;
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

    RiGraph out{g.GetFacade()};
    std::unordered_map<NodeID, Vertex> vmap; // old-new vertex mappings

    // helpers
    const auto addVertex = [&](const auto v)
    {
        const Vertex newVertex = add_vertex(v, out);
        vmap[v] = newVertex;
        return newVertex;
    };

    const auto addEdge =
        [&out](const Vertex u, const Vertex v, const EdgeID edgeName, const EdgeWeight weight)
    {
        const auto [e, isNew] = add_edge(u, v, out);
        BOOST_ASSERT(isNew);
        put(edge_name, out, e, edgeName);
        put(edge_weight, out, e, weight);
        return e;
    };

    // BFS traversal with filtering
    std::queue<NodeID> q;

    // add first vertex
    addVertex(start);
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

            const auto v = g.GetTarget(e);
            const auto w = g.GetEdgeWeight(u, e);
            BOOST_ASSERT(u != v && w != INVALID_EDGE_WEIGHT);

            if (!vmap.contains(v))
            {
                // unvisited -> new vertex and edge
                addEdge(curVertex, addVertex(v), e, w);
                q.push(v);
            }
            else
            {
                // already visited -> new edge only
                const auto targetVertex = vmap[v];
                addEdge(curVertex, targetVertex, e, w);
            }
        }
    }

    // verify start connectivity
    if (in_degree(Vertex{0}, out) == 0)
    {
        util::Log(logDEBUG) << "Invalid RiGraph: start vertex has 0 incoming edges";
        return RiGraph{g.GetFacade()};
    }

    // ensure single SCC without dead-ends
    dropMinorSCCs(out);

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
    for (const auto u : make_iterator_range(vertices(g)))
    {
        auto &uEdges = unusedEdges[u].second;
        uEdges.reserve(out_degree(u, g));
        for (const auto e : make_iterator_range(out_edges(u, g)))
        {
            uEdges.emplace_back(e);
        }
        // sort edges by their weight
        std::stable_sort(uEdges.begin(),
                         uEdges.end(),
                         [&g](auto a, auto b)
                         { return get(edge_weight, g, a) < get(edge_weight, g, b); });
        // set initial iterator
        unusedEdges[u].first = uEdges.cbegin();
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
        const auto u = st.top();
        const auto &uEdges = unusedEdges[u].second;
        std::optional<EdgeIt> nextEdge;
        if (auto e = unusedEdges[u].first; e != uEdges.cend())
        {
            for (; e != uEdges.cend(); ++e)
            {
                // check that target vertex is unvisited
                if (!visitedNodes.contains(target(*e, g)))
                {
                    nextEdge.emplace(e);
                    break;
                }
            }

            // all unvisited edges point to already visited nodes
            if (!nextEdge && !visitedNodes.contains(u))
            {
                // out-disjoint node to be connected later
                g.logVertexEnd(u, "disjointOut");
                disjointOutNodes.emplace(u);
            }
        }

        if (nextEdge)
        {
            if (uniqueParent)
            {
                // ensure edge to this unique parent exists
                circuit.emplace_back(u);
                uniqueParent = false;
            }
            // take next edge u->v
            auto e = *nextEdge;
            const auto v = target(*e, g);
            unusedEdges[u].first = ++e;
            st.push(v);
        }
        else
        {
            uniqueParent = in_degree(u, g) == 1;
            if (!uniqueParent)
            {
                // in-disjoint node to be connected later
                g.logVertexStart(u, "disjointIn");
                disjointInNodes.emplace(u);
            }
            circuit.emplace_back(u);
            st.pop();
        }

        // mark node as visited
        visitedNodes.emplace(u);
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
        const auto u = *std::prev(it);
        const auto v = *it;
        const auto [e, exists] = edge(u, v, g);
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

    // 4) Fix out-disjoints by connecting them to start using shortest path
    for (const auto u : disjointOutNodes)
    {
        BOOST_ASSERT(revDists[u] != INVALID_EDGE_WEIGHT);

        const auto sp = extractPath(revPreds, start, u);
        BOOST_ASSERT(sp.size() >= 2);

        // iterate in reverse due to reversed graph
        auto it = sp.crbegin();
        ++it;
        for (; it != sp.crend(); ++it)
        {
            const auto from = *std::prev(it);
            const auto to = *it;
            const auto [e, exists] = edge(from, to, g);
            BOOST_ASSERT(exists);
            usedEdges.emplace(e);
            // fix in-disjoint with this edge unless it's a disjoint->disjoint transition which is
            // disallowed to prevent cycles
            if (!disjointOutNodes.contains(from))
            {
                disjointInNodes.erase(to);
            }
        }
    }

    // 5) Fix leftover in-disjoints by connecting start to them using shortest path
    for (const auto v : disjointInNodes)
    {
        BOOST_ASSERT(in_degree(v, g) > 1);
        BOOST_ASSERT(dists[v] != INVALID_EDGE_WEIGHT);

        const auto sp = extractPath(preds, start, v);
        BOOST_ASSERT(sp.size() >= 2);

        auto it = sp.cbegin();
        ++it;
        for (; it != sp.cend(); ++it)
        {
            const auto from = *std::prev(it);
            const auto to = *it;
            const auto [e, exists] = edge(from, to, g);
            BOOST_ASSERT(exists);
            usedEdges.emplace(e);
        }
    }

    return usedEdges;
}

// Removes edges from a graph which can be replaced with a cheaper shortest path
inline auto collectCostlyEdges(RiGraph &g)
{
    auto const isShortestPath = [&g](const Edge e)
    {
        const auto u = source(e, g);
        const auto v = target(e, g);
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
inline void optimizeRiGraph(RiGraph &g)
{
    using namespace boost;

    // 1) Preprocessing to prune edges which are not shortest paths
    const auto costlyEdges = collectCostlyEdges(g);
    for (const auto e : costlyEdges)
    {
        g.logEdge(e, "costly");
        remove_edge(e, g);
    }

    BOOST_ASSERT(isValidGraph(g));

    // 2) Collect optimal edges
    const Vertex start{0};
    const auto usedEdges = collectMinCostEdgeSet(g, start);

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
        else
        {
            g.logEdge(e, "used");
        }
    }

    // 4) Actual edge removal
    for (const auto e : unusedEdges)
    {
        g.logEdge(e, "unused");
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

} // namespace detail

/**
 * @brief Extracts original NodeIDs from the resulting path and creates final route result.
 *
 * @param g RI graph used to create a closed path
 * @param path resulting closed path
 * @return std::vector<NodeID> final route result
 */
inline std::vector<NodeID> prepareFinalRoute(const detail::RiGraph &g, const detail::Path &path)
{
    if (path.empty())
    {
        util::Log(logDEBUG) << "Resulting path is empty";
        return {};
    }

    std::vector<NodeID> route;
    route.reserve(path.size());

    std::transform(path.cbegin(),
                   path.cend(),
                   std::back_inserter(route),
                   [&g](const auto v)
                   {
                       // extract original NodeID
                       g.logVertexEnd(v, "final route");
                       return get(boost::vertex_name, g, v);
                   });

    // TODO: improve final route by replacing costly edges with shortest paths (which
    // possibly go beyond limiting polygon)

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
