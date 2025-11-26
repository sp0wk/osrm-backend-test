#ifndef OSRM_UTIL_BGL_GRAPH_ADAPTORS_HPP
#define OSRM_UTIL_BGL_GRAPH_ADAPTORS_HPP

#include "storage/shared_memory_ownership.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

#include <iterator>
#include <utility>

namespace osrm::util
{

// General template for BGL (Boost Graph Library) graph adaptor implementations
template <typename GraphT, storage::Ownership Ownership = storage::Ownership::View>
class BglGraphAdaptor;

// Specialization for directed NodeBasedDynamicGraph
template <> class BglGraphAdaptor<NodeBasedDynamicGraph, storage::Ownership::View>
{
  public:
    using Graph = NodeBasedDynamicGraph;
    using Vertex = NodeID;
    using Edge = EdgeID;
    using Weight = EdgeWeight;
    using VertexProperty = boost::property<boost::vertex_index_t, Vertex>;
    using EdgeProperty = boost::property<boost::edge_weight_t, Weight>;

    explicit BglGraphAdaptor(const Graph &graph) : g{graph} {}

    const Graph &GetGraph() const noexcept { return g; }

    // convenience methods

    Weight GetWeight(Edge e) const
    {
        BOOST_ASSERT(e < g.GetEdgeCapacity());
        return g.GetEdgeData(e).weight;
    }

    // BGL required functions

    friend auto num_vertices(const BglGraphAdaptor &g) { return g.g.GetNumberOfNodes(); }

    friend auto num_edges(const BglGraphAdaptor &g) { return g.g.GetNumberOfEdges(); }

    friend auto vertices(const BglGraphAdaptor &g)
    {
        const auto range = g.g.GetNodeRange();
        return std::make_pair(range.begin(), range.end());
    }

    friend auto edges(const BglGraphAdaptor &g)
    {
        const auto range = g.g.GetDirectedEdgeRange();
        return std::make_pair(std::cbegin(range), std::cend(range));
    }

    friend auto edge(const Vertex u, const Vertex v, const BglGraphAdaptor &g)
    {
        if (Edge e = g.g.FindEdge(u, v); e != SPECIAL_EDGEID)
        {
            return std::make_pair(e, true);
        }
        return std::make_pair(SPECIAL_EDGEID, false);
    }

    friend Vertex source(const Edge e, const BglGraphAdaptor &g) { return g.g.GetSource(e); }

    friend Vertex target(const Edge e, const BglGraphAdaptor &g) { return g.g.GetTarget(e); }

    friend auto out_edges(const Vertex u, const BglGraphAdaptor &g)
    {
        const auto range = g.g.GetAdjacentDirectedEdgeRange(u);
        return std::make_pair(std::cbegin(range), std::cend(range));
    }

    friend auto out_degree(const Vertex u, const BglGraphAdaptor &g)
    {
        return g.g.GetDirectedOutDegree(u);
    }

    // BGL property maps

    struct WeightMap
    {
        using key_type = BglGraphAdaptor::Edge;
        using value_type = BglGraphAdaptor::Weight;
        using reference = value_type;
        using category = boost::readable_property_map_tag;

        friend value_type get(const WeightMap &map, const key_type e)
        {
            BOOST_ASSERT(map.g != nullptr);
            return map.g->GetWeight(e);
        }

        const BglGraphAdaptor *g{nullptr};
    };

    friend auto get(boost::edge_weight_t, const BglGraphAdaptor &g)
    {
        return BglGraphAdaptor::WeightMap{&g};
    }

    friend auto get(boost::edge_weight_t, const BglGraphAdaptor &g, const Edge e)
    {
        return g.GetWeight(e);
    }

    friend auto get(boost::vertex_index_t, const BglGraphAdaptor &)
    {
        // vertex descriptor is already an index
        return boost::typed_identity_property_map<Vertex>{};
    }

    friend auto get(boost::vertex_index_t, const BglGraphAdaptor &, const Vertex u)
    {
        // vertex descriptor is already an index
        return u;
    }

    friend auto get(boost::edge_index_t, const BglGraphAdaptor &)
    {
        // edge descriptor is already an index
        return boost::typed_identity_property_map<Edge>{};
    }

    friend auto get(boost::edge_index_t, const BglGraphAdaptor &, const Edge e)
    {
        // edge descriptor is already an index
        return e;
    }

  private:
    const Graph &g;
};

using BglNodeBasedDynamicGraph = BglGraphAdaptor<NodeBasedDynamicGraph, storage::Ownership::View>;

} // namespace osrm::util

namespace boost
{

// Graph traits for BglNodeBasedDynamicGraph
//-------------------------------------------------------------------------------------------------
template <> struct graph_traits<osrm::util::BglNodeBasedDynamicGraph>
{
    using G = osrm::util::BglNodeBasedDynamicGraph;
    using directed_category = directed_tag;
    using edge_parallel_category = allow_parallel_edge_tag;
    struct traversal_category : virtual incidence_graph_tag,
                                virtual vertex_list_graph_tag,
                                virtual edge_list_graph_tag
    {
    };

    using vertex_descriptor = G::Vertex;
    using edge_descriptor = G::Edge;
    using vertex_iterator = decltype(vertices(std::declval<G>()).first);
    using edge_iterator = decltype(edges(std::declval<G>()).first);
    using out_edge_iterator =
        decltype(out_edges(std::declval<vertex_descriptor>(), std::declval<G>()).first);
    using in_edge_iterator = void *; // not supported

    using vertices_size_type = decltype(num_vertices(std::declval<G>()));
    using edges_size_type = decltype(num_edges(std::declval<G>()));
    using degree_size_type =
        decltype(out_degree(std::declval<vertex_descriptor>(), std::declval<G>()));

    using vertex_property_type = G::VertexProperty;
    using edge_property_type = G::EdgeProperty;

    static constexpr vertex_descriptor null_vertex()
    {
        return std::numeric_limits<vertex_descriptor>::max();
    }
};

template <> struct property_map<osrm::util::BglNodeBasedDynamicGraph, vertex_index_t>
{
    using G = osrm::util::BglNodeBasedDynamicGraph;
    using type = boost::typed_identity_property_map<G::Vertex>;
    using const_type = type;
};

template <> struct property_map<osrm::util::BglNodeBasedDynamicGraph, edge_weight_t>
{
    using G = osrm::util::BglNodeBasedDynamicGraph;
    using type = G::WeightMap;
    using const_type = type;
};
//-------------------------------------------------------------------------------------------------

} // namespace boost

BOOST_CONCEPT_ASSERT((boost::concepts::IncidenceGraph<osrm::util::BglNodeBasedDynamicGraph>));
BOOST_CONCEPT_ASSERT(
    (boost::concepts::VertexAndEdgeListGraph<osrm::util::BglNodeBasedDynamicGraph>));

#endif // OSRM_UTIL_BGL_GRAPH_ADAPTORS_HPP
