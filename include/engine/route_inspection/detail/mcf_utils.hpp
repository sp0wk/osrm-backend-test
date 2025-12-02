#ifndef OSRM_ROUTE_INSPECTION_MCF_HPP
#define OSRM_ROUTE_INSPECTION_MCF_HPP

#include "engine/route_inspection/detail/types.hpp"

#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/range/adaptors.hpp>

#include <cstdlib>
#include <unordered_map>
#include <utility>
#include <vector>

namespace osrm::engine::route_inspection::detail::mcf
{

using namespace boost;

//-------------------------------------------------------------------------------------------------
// Min-cost flow graph augmentation
//-------------------------------------------------------------------------------------------------

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

// Main Min-cost flow graph type
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
                BOOST_ASSERT(get(vertex_name, g, tv) == static_cast<std::size_t>(cIdx));
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
        const auto cap = capacityMap[e];
        const auto residualCap = residualCapacityMap[e];
        if (cap > residualCap)
        {
            const auto flow = cap - residualCap;
            const auto row = vertexToIdxMap[s];
            const auto col = vertexToIdxMap[t];
            BOOST_ASSERT(row < pathMatrix.rows.size() && col < pathMatrix.rows[0].columns.size());
            result.emplace_back(row, col, flow);
        }
    }

    return result;
}

} // namespace osrm::engine::route_inspection::detail::mcf

#endif
