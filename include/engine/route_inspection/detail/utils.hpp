#ifndef OSRM_ROUTE_INSPECTION_UTILS_HPP
#define OSRM_ROUTE_INSPECTION_UTILS_HPP

#include "engine/datafacade/datafacade_base.hpp"
#include "engine/route_inspection/detail/types.hpp"

#include "util/coordinate.hpp"
#include "util/log.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/range/adaptors.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cstdlib>
#include <stack>
#include <utility>
#include <vector>

namespace osrm::engine::route_inspection::detail
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
// Graph connectivity utils
//-------------------------------------------------------------------------------------------------

// Checks whether directed graph is strongly connected
inline bool isStronglyConnectedGraph(const RiGraphBase &g)
{
    using namespace boost;
    std::vector<int> component(num_vertices(g));
    auto num =
        strong_components(g, make_iterator_property_map(component.begin(), get(vertex_index, g)));
    return num == 1;
}

// RiGraph correctness verification
inline bool isValidGraph(const RiGraphBase &g)
{
    return num_vertices(g) >= 2 && num_edges(g) >= 2 && isStronglyConnectedGraph(g);
}

// Retrieves RiGraph's vertices which belong to minor SCCs and prevents graph's strong connectivity
inline std::vector<Vertex> findMinorSCCs(RiGraphBase &g)
{
    using namespace boost;

    std::vector<int> sccs(num_vertices(g));
    auto num = strong_components(g, make_iterator_property_map(sccs.begin(), get(vertex_index, g)));
    if (num == 1)
    {
        // only main SCC already
        return {};
    }

    // main SCC is the one which contains start vertex
    const auto mainSCC = sccs[0];

    // remove minor sccs vertices
    std::vector<Vertex> minorSCCs;
    minorSCCs.reserve(sccs.size() / 2);

    for (const auto v : util::irange<std::size_t>(0, sccs.size()))
    {
        if (sccs[v] != mainSCC)
        {
            minorSCCs.emplace_back(static_cast<Vertex>(v));
        }
    }

    return minorSCCs;
}

//-------------------------------------------------------------------------------------------------
// Eulerian circuit
//-------------------------------------------------------------------------------------------------

// Collects edge degree delta for all nodes in the graph
inline NodeDegreeDeltaArray collectNodeDegreeDeltas(const RiGraphBase &g)
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
inline bool isEulerianGraph(const RiGraphBase &g)
{
    return isEulerianGraph(collectNodeDegreeDeltas(g));
}

// Finds Eulerian circuit on directed Eulerian graph using Hierholzer’s algorithm
inline Path findEulerianCircuit(const RiGraphBase &g, const Vertex source)
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

// Finds shortest paths from source s to all graph's vertices.
// Templated to support reverse_graph as well.
template <typename Graph, typename Vertex = boost::graph_traits<Graph>::vertex_descriptor>
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
inline ShortestPath shortestPath(const RiGraphBase &g, const Vertex s, const Vertex t)
{
    using namespace boost;

    struct Terminator : default_dijkstra_visitor
    {
        Terminator(const Vertex t) : default_dijkstra_visitor(), target{t} {}

        void examine_vertex(const Vertex v, const RiGraphBase &) const
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
inline std::pair<std::vector<Vertex>, std::vector<EdgeWeight>> oneToMany(const RiGraphBase &g,
                                                                         const Vertex source)
{
    // prepare maps for predecessors and distances
    std::vector<Vertex> preds(num_vertices(g));
    std::vector<EdgeWeight> dists(num_vertices(g), INVALID_EDGE_WEIGHT);

    // calculate shortest paths from source to all vertices
    shortestPaths(g, source, preds, dists);

    return std::make_pair(std::move(preds), std::move(dists));
}

// Finds shortest paths between all sources and targets
inline PathMatrix manyToMany(const RiGraphBase &g,
                             const std::vector<Vertex> &sources,
                             const std::vector<Vertex> &targets)
{
    PathMatrix m;
    // allocate matrix
    m.initialize(sources.size(), targets.size());
    m.sources = sources;
    m.targets = targets;

    util::Log(logDEBUG) << "Running manyToMany shortest paths search...";

    // parallelize multiple oneToMany calls
    tbb::parallel_for(tbb::blocked_range<size_t>(0UL, sources.size()),
                      [&](const tbb::blocked_range<size_t> &r)
                      {
                          for (auto sIdx = r.begin(); sIdx != r.end(); ++sIdx)
                          {
                              // calc 1-to-m routes
                              auto paths = oneToMany(g, sources[sIdx]);
                              BOOST_ASSERT(paths.first.size() == paths.second.size());
                              // populate matrix
                              auto &data = m.data[sIdx];
                              data.preds = std::move(paths.first);
                              data.dists = std::move(paths.second);
                          }
                      });

    return m;
}

} // namespace osrm::engine::route_inspection::detail

#endif
