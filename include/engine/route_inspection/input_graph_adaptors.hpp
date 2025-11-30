#ifndef ENGINE_RI_INPUT_GRAPH_ADAPTORS_HPP
#define ENGINE_RI_INPUT_GRAPH_ADAPTORS_HPP

#include "engine/datafacade.hpp"
#include "engine/datafacade/algorithm_datafacade.hpp"
#include "engine/datafacade/datafacade_base.hpp"

#include "engine/routing_algorithms/routing_base_ch.hpp"

#include "util/alias.hpp"
#include "util/exception.hpp"
#include "util/typedefs.hpp"

#include <boost/concept/assert.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/concepts.hpp>

#include <type_traits>

namespace osrm::engine::route_inspection
{

template <typename G> struct InputGraphConcept
{
    BOOST_CONCEPT_USAGE(InputGraphConcept)
    {
        // Get base datafacade
        f = &g.GetFacade();

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
    const G g;
    const datafacade::BaseDataFacade *f;
    NodeID edge_based_node_id;
    EdgeID edge_based_edge_id;
    EdgeWeight edge_weight;
};

//-------------------------------------------------------------------------------------------------

// Generic template for input graphs for various routing algorithms
template <typename AlgorithmT> class AlgorithmBasedInputGraphWrapper;

//-------------------------------------------------------------------------------------------------

// Dummy input graph wrapper for CH's edge-based graph exposed via DataFacade
// It should never be actually used at runtime
template <> class AlgorithmBasedInputGraphWrapper<datafacade::CH>
{
  public:
    using Graph = DataFacade<datafacade::CH>;

    explicit AlgorithmBasedInputGraphWrapper(const Graph &facade) : facade{facade}
    {
        throw util::exception{"Route inspection on CH graph is not supported"};
    }

    const Graph &GetFacade() const noexcept { return facade; }

    // Fake interface implementation
    std::vector<EdgeID> GetOutEdgeRange(const NodeID) const { return {}; }
    EdgeWeight GetEdgeWeight(const NodeID, const EdgeID) const { return INVALID_EDGE_WEIGHT; }
    NodeID GetTarget(const EdgeID) const { return SPECIAL_NODEID; }

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

    const Graph &GetFacade() const noexcept { return facade; }

    // Interface implementation

    auto GetOutEdgeRange(const NodeID v) const
    {
        BOOST_ASSERT(v != SPECIAL_NODEID);
        const auto edges = facade.GetAdjacentEdgeRange(v);
        return boost::adaptors::filter(edges,
                                       [this](const auto e) { return facade.IsForwardEdge(e); });
    }

    EdgeWeight GetEdgeWeight(const NodeID v, const EdgeID e) const
    {
        BOOST_ASSERT(v != SPECIAL_NODEID && e != SPECIAL_EDGEID);
        const auto &edge_data = facade.GetEdgeData(e);
        EdgeWeight w = facade.GetNodeWeight(v);
        BOOST_ASSERT(w != INVALID_EDGE_WEIGHT);
        const auto turn_penalty =
            alias_cast<EdgeWeight>(facade.GetWeightPenaltyForEdgeID(edge_data.turn_id));
        BOOST_ASSERT(turn_penalty != INVALID_EDGE_WEIGHT);
        return w + turn_penalty;
    }

    NodeID GetTarget(const EdgeID e) const
    {
        BOOST_ASSERT(e != SPECIAL_EDGEID);
        return facade.GetTarget(e);
    }

  private:
    const Graph &facade;
};

BOOST_CONCEPT_ASSERT((InputGraphConcept<AlgorithmBasedInputGraphWrapper<datafacade::MLD>>));

} // namespace osrm::engine::route_inspection

#endif // ENGINE_RI_INPUT_GRAPH_ADAPTORS_HPP
