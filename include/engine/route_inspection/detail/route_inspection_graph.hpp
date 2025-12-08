#ifndef OSRM_ROUTE_INSPECTION_GRAPH_HPP
#define OSRM_ROUTE_INSPECTION_GRAPH_HPP

#include "engine/datafacade/datafacade_base.hpp"
#include "engine/route_inspection/detail/mcf_utils.hpp"
#include "engine/route_inspection/detail/types.hpp"
#include "engine/route_inspection/detail/utils.hpp"

#include "engine/route_inspection/input_graph_adaptors.hpp"

#include "util/coordinate.hpp"
#include "util/log.hpp"
#include "util/polygon.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/range/adaptors.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <optional>
#include <queue>
#include <stack>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace osrm::engine::route_inspection::detail
{

/**
 * \brief Main route inspection graph built from the input graph and used everywhere in the
 * algorithm.
 */
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

// strong connectivity helper
inline void dropMinorSCCs(RiGraph &g)
{
    const auto toRemove = findMinorSCCs(g);

    // remove vertices (and consequently edges)
    // NOTE: since we use vecS to store vertices in adjacency_list, we're iterating in reverse to
    // remove vertices in decreasing order (to not invalidate other vertex descriptors inside
    // toRemove)
    for (const auto v : boost::adaptors::reverse(toRemove))
    {
#ifndef NDEBUG
        g.logVertex(v, "minorScc");
#endif
        clear_vertex(v, g);
        remove_vertex(v, g);
    }
}

// Default input graph edge filter
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
#ifndef NDEBUG
                g.logVertex(u, "disjointOut");
#endif
                // out-disjoint node to be connected later
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
#ifndef NDEBUG
                g.logVertex(u, "disjointIn");
#endif
                // in-disjoint node to be connected later
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

    ResultSet usedEdges{0, EdgeHash{&g}};
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
            // fix out-disjoint with this edge (if any)
            disjointOutNodes.erase(u);
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
        // check whether this disjoint-out was already fixed
        for (const auto e : make_iterator_range(out_edges(u, g)))
        {
            if (usedEdges.contains(e))
            {
                continue; // was fixed already
            }
        }

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
    disjointInNodes.erase(start); // safeguard against disjoint start
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

    [[maybe_unused]] const auto initEdgeCount = num_edges(g);

    // 1) Preprocessing to prune edges which are not shortest paths
    const auto costlyEdges = collectCostlyEdges(g);
    for (const auto e : costlyEdges)
    {
#ifndef NDEBUG
        g.logEdge(e, "costly");
#endif
        remove_edge(e, g);
    }

    BOOST_ASSERT(isValidGraph(g));
    util::Log(logDEBUG) << "[optimizeRiGraph]  Number of edges after removing costly edges: "
                        << num_edges(g);

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
#ifndef NDEBUG
        else
        {
            g.logEdge(e, "used");
        }
#endif
    }

    // 4) Actual edge removal
    for (const auto e : unusedEdges)
    {
#ifndef NDEBUG
        g.logEdge(e, "unused");
#endif
        remove_edge(e, g);
    }

    util::Log(logDEBUG)
        << "[optimizeRiGraph]  Number of edges before/after removing unused edges:  "
        << initEdgeCount << " vs " << num_edges(g);
}

// Adds deficit edges to imbalanced graph through solving min-cost flow problem to make graph
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
        const auto &row = pathMatrix.data[edgeFlow.sIdx];
        for ([[maybe_unused]] const auto reps : util::irange(0U, edgeFlow.flow))
        {
            const auto s = pathMatrix.sources[edgeFlow.sIdx];
            const auto t = pathMatrix.targets[edgeFlow.tIdx];

            BOOST_ASSERT(row.dists[t] != INVALID_EDGE_WEIGHT);
            const auto p = extractPath(row.preds, s, t);
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

} // namespace osrm::engine::route_inspection::detail

#endif /* OSRM_ROUTE_INSPECTION_GRAPH_HPP */
