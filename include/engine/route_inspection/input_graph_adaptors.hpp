#ifndef ENGINE_RI_INPUT_GRAPH_ADAPTORS_HPP
#define ENGINE_RI_INPUT_GRAPH_ADAPTORS_HPP

#include "engine/datafacade.hpp"
#include "engine/datafacade/algorithm_datafacade.hpp"

#include "engine/routing_algorithms/routing_base_ch.hpp"

#include "util/alias.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/concepts.hpp>

#include <type_traits>
#include <unordered_set>

namespace osrm::engine::route_inspection
{

template <typename G> struct InputGraphConcept
{
    BOOST_CONCEPT_USAGE(InputGraphConcept)
    {
        // Get range over all outgoing directed edges of a node
        auto edges = g.GetOutEdgeRange(edge_based_node_id);
        BOOST_CONCEPT_ASSERT((boost::ForwardRangeConcept<decltype(edges)>));
        static_assert(std::is_same_v<std::decay_t<decltype(*edges.begin())>, EdgeID>);

        // Get full edge weight (EBG node weight + turn penalty)
        edge_weight = g.GetEdgeWeight(edge_based_node_id, edge_based_edge_id);

        // Get target node of an edge
        edge_based_node_id = g.GetTarget(edge_based_edge_id);
    }

  private:
    G g;
    NodeID edge_based_node_id;
    EdgeID edge_based_edge_id;
    EdgeWeight edge_weight;
};

//-------------------------------------------------------------------------------------------------

// Generic template for input graphs for various routing algorithms
template <typename AlgorithmT> class AlgorithmBasedInputGraphWrapper;

//-------------------------------------------------------------------------------------------------

// Input graph wrapper for CH's edge-based graph exposed via DataFacade
template <> class AlgorithmBasedInputGraphWrapper<datafacade::CH>
{
  public:
    using Graph = DataFacade<datafacade::CH>;

    explicit AlgorithmBasedInputGraphWrapper(const Graph &facade) : facade{facade} {}

    const Graph &GetSourceGraph() const noexcept { return facade; }

    // Interface implementation

    // ATTENTION: By design CH doesn't support fast lookup of original edges, so edges are
    // dynamically unpacked on the fly which is rather slow.
    auto GetOutEdgeRange(const NodeID v) const
    {
        // to dedup same edges from different shortcuts
        const auto nbOfEdges = facade.GetOutDegree(v);
        std::vector<EdgeID> outEdges;
        outEdges.reserve(nbOfEdges);
        std::unordered_set<NodeID> visitedNodes;
        visitedNodes.reserve(nbOfEdges);

        for (const auto e : facade.GetAdjacentEdgeRange(v))
        {
            // only out edges
            if (!facade.GetEdgeData(e).forward)
            {
                continue;
            }

            // extract first original edge from the shortcut
            std::vector<NodeID> unpackedEdge;
            routing_algorithms::ch::unpackEdge(facade, v, facade.GetTarget(e), unpackedEdge);
            BOOST_ASSERT(unpackedEdge.size() >= 2 && unpackedEdge.front() == v);
            auto origEdge = facade.FindSmallestEdge(
                unpackedEdge[0], unpackedEdge[1], [](const auto &data) { return data.forward; });
            BOOST_ASSERT(origEdge != SPECIAL_EDGEID);

            // check for uniqueness
            if (auto u = facade.GetTarget(origEdge); !visitedNodes.contains(u))
            {
                outEdges.emplace_back(origEdge);
                visitedNodes.emplace(u);
            }
        }

        return outEdges;
    }

    EdgeWeight GetEdgeWeight(const NodeID, const EdgeID e) const
    {
        return facade.GetEdgeData(e).weight;
    }

    NodeID GetTarget(const EdgeID e) const { return facade.GetTarget(e); }

  private:
    const Graph &facade;
};

//-------------------------------------------------------------------------------------------------

// Input graph wrapper for MLD's edge-based graph exposed via DataFacade
template <> class AlgorithmBasedInputGraphWrapper<datafacade::MLD>
{
  public:
    using Graph = DataFacade<datafacade::MLD>;

    explicit AlgorithmBasedInputGraphWrapper(const Graph &facade) : facade{facade} {}

    const Graph &GetSourceGraph() const noexcept { return facade; }

    // Interface implementation

    auto GetOutEdgeRange(const NodeID v) const
    {
        // on 0 level same as GetAdjacentEdgeRange
        const auto edges = facade.GetBorderEdgeRange(0, v);
        return boost::adaptors::filter(edges,
                                       [this](const auto e) { return facade.IsForwardEdge(e); });
    }

    EdgeWeight GetEdgeWeight(const NodeID v, const EdgeID e) const
    {
        const auto &edge_data = facade.GetEdgeData(e);
        EdgeWeight w = facade.GetNodeWeight(v);
        const auto turn_penalty = facade.GetWeightPenaltyForEdgeID(edge_data.turn_id);
        return w + alias_cast<EdgeWeight>(turn_penalty);
    }

    NodeID GetTarget(const EdgeID e) const { return facade.GetTarget(e); }

  private:
    const Graph &facade;
};

BOOST_CONCEPT_ASSERT((InputGraphConcept<AlgorithmBasedInputGraphWrapper<datafacade::MLD>>));

} // namespace osrm::engine::route_inspection

#endif // ENGINE_RI_INPUT_GRAPH_ADAPTORS_HPP
