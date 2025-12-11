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
 * @param densityLimit max nearest targets per source (perf optimization)
 * @return auto super-source and super-sink vertex descriptors
 */
inline auto buildMcfGraph(McfGraph &g,
                          const PathMatrix &pathMatrix,
                          const NodeDegreeDeltaArray &deltas,
                          const McfGraphTraits::Capacity maxCapacity =
                              std::numeric_limits<McfGraphTraits::Capacity>::max(),
                          const std::size_t densityLimit = 0)
{
    BOOST_ASSERT(num_vertices(g) == 0 && num_edges(g) == 0);
    BOOST_ASSERT(pathMatrix.data.size() > 0 &&
                 (pathMatrix.sources.size() == pathMatrix.data.size()));

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

    util::Log(logDEBUG) << "Building MCF graph for " << pathMatrix.sources.size() << " sources and "
                        << pathMatrix.targets.size() << " targets";

    const auto superSource = addVertex(-1);
    const auto superSink = addVertex(-1);

    std::unordered_map<Vertex, V> sources, sinks;
    sources.reserve(pathMatrix.sources.size());
    sinks.reserve(pathMatrix.targets.size());

    // create/add source nodes
    for (const auto &[sIdx, s] : adaptors::index(pathMatrix.sources))
    {
        // insert new source node and connect it with super-source
        const auto sv = addVertex(sIdx);
        sources[s] = sv;
        const auto cap = std::abs(deltas[s]);
        addEdge(superSource, sv, cap, 0);
    }

    // create/add sink nodes
    for (const auto &[tIdx, t] : adaptors::index(pathMatrix.targets))
    {
        // insert new sink node and connect it to super-sink
        const auto tv = addVertex(tIdx);
        sinks[t] = tv;
        const auto cap = std::abs(deltas[t]);
        addEdge(tv, superSink, cap, 0);
    }

    // helper to optimize MCF graph size
    std::unordered_set<Vertex> connectedTargets;
    connectedTargets.reserve(pathMatrix.targets.size());

    const auto collectBestEdges = [&](const auto sIdx)
    {
        std::vector<std::pair<Vertex, EdgeWeight>> edges;
        edges.reserve(pathMatrix.targets.size());

        for (const auto t : pathMatrix.targets)
        {
            edges.emplace_back(t, pathMatrix.data[sIdx].dists[t]);
        }

        // sort edges by shortest path distance to source sIdx vertex
        std::sort(edges.begin(),
                  edges.end(),
                  [&](const auto &a, const auto &b) { return a.second < b.second; });

        std::vector<std::pair<Vertex, EdgeWeight>> bestEdges;
        bestEdges.reserve(edges.size());

        // collect best edges while trying to connect all targets
        bool requireNewTarget{connectedTargets.size() < pathMatrix.targets.size()};
        for (const auto &e : edges)
        {
            const auto t = e.first;
            if (requireNewTarget && !connectedTargets.contains(t))
            {
                requireNewTarget = false;
            }

            bestEdges.emplace_back(e);
            connectedTargets.emplace(t);
            if (bestEdges.size() >= densityLimit && !requireNewTarget)
            {
                // stop when enough targets are collected and at least one new target is connected
                break;
            }
        }

        return bestEdges;
    };

    // create/add source nodes and source->sink edges
    for (const auto &[sIdx, data] : adaptors::index(pathMatrix.data))
    {
        const Vertex s = pathMatrix.sources[sIdx];
        const auto sv = sources[s];

        // connect surplus node to deficit nodes
        if (densityLimit > 0)
        {
            // Optimization: connect only best N targets per source
            const auto bestEdges = collectBestEdges(sIdx);
            for (const auto &[t, w] : bestEdges)
            {
                const auto tv = sinks[t];
                const auto cost = data.dists[t];
                BOOST_ASSERT(cost != INVALID_EDGE_WEIGHT);
                addEdge(sv, tv, maxCapacity, cost.__value);
            }
        }
        else
        {
            for (const auto t : pathMatrix.targets)
            {
                const auto tv = sinks[t];
                const auto cost = data.dists[t];
                BOOST_ASSERT(cost != INVALID_EDGE_WEIGHT);
                addEdge(sv, tv, maxCapacity, cost.__value);
            }
        }
    }

    // safeguard against disconnected sinks
    // TODO: implement retry mechanism to guarantee MCF feasibility in case we have sources without
    // any sinks with capacity > 0
    if (densityLimit > 0 && connectedTargets.size() < pathMatrix.targets.size())
    {
        util::Log(logDEBUG) << "MCF: connecting disconnected sinks...";
        for (const auto t : pathMatrix.targets)
        {
            if (connectedTargets.contains(t))
            {
                continue;
            }

            const auto tv = sinks[t];
            for (const auto &[sIdx, data] : adaptors::index(pathMatrix.data))
            {
                const auto s = pathMatrix.sources[sIdx];
                const auto sv = sources[s];
                const auto cost = data.dists[t];
                BOOST_ASSERT(cost != INVALID_EDGE_WEIGHT);
                addEdge(sv, tv, maxCapacity, cost.__value);
            }
        }
    }

    util::Log(logDEBUG) << "Number of edges in MCF graph: " << num_edges(g);

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
    BOOST_ASSERT(pathMatrix.data.size() > 0 &&
                 (pathMatrix.sources.size() == pathMatrix.data.size()));

    // reasonable time feasibility check
    std::size_t densityLimit{0};
    static constexpr const std::size_t MCF_LIMIT = 10000;
    if (const auto n = pathMatrix.sources.size() * pathMatrix.targets.size(); n > MCF_LIMIT)
    {
        const auto minDensity =
            std::max<std::size_t>(10, (pathMatrix.targets.size() / pathMatrix.sources.size()) + 1);
        densityLimit = std::max(minDensity, MCF_LIMIT / pathMatrix.sources.size());
        util::Log(logDEBUG) << "Number of sources/targets for MCF solver is too high (s*t=" << n
                            << " > " << MCF_LIMIT << "). Using densityLimit=" << densityLimit
                            << " to optimize MCF graph size";
    }

    // build min-cost flow graph
    McfGraph g;
    const auto [superSource, superSink] =
        buildMcfGraph(g, pathMatrix, deltas, maxCapacity, densityLimit);

    // solve min-cost flow
    // TODO: consider more scalable MCF solver than SSP (or optimize number of nodes)
    util::Log(logDEBUG) << "Running SSP search to determine optimal flow...";
    successive_shortest_path_nonnegative_weights(g, superSource, superSink);
    util::Log(logDEBUG) << "SSP search has finished";

    // prepare result
    auto vertexToIdxMap = get(vertex_name, g);
    auto capacityMap = get(edge_capacity, g);
    auto residualCapacityMap = get(edge_residual_capacity, g);

    MinCostFlow result;
    result.reserve(pathMatrix.data.size());
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
            BOOST_ASSERT(row < pathMatrix.sources.size() && col < pathMatrix.data[0].preds.size());
            result.emplace_back(row, col, flow);
        }
    }

    return result;
}

} // namespace osrm::engine::route_inspection::detail::mcf

#endif
