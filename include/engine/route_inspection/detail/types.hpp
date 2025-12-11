#ifndef OSRM_ROUTE_INSPECTION_TYPES_HPP
#define OSRM_ROUTE_INSPECTION_TYPES_HPP

#include "util/exception.hpp"
#include "util/integer_range.hpp"
#include "util/typedefs.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include <cstdlib>
#include <vector>

namespace osrm::engine::route_inspection::detail
{

// Main graph types (BGL based) used for route inspection implementation

// tag to indicate unused (non-optimal edges)
struct edge_unused_tag_t
{
    using kind = boost::edge_property_tag;
};
static constexpr const edge_unused_tag_t edge_unused_tag{};

using RiGraphBase = boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    boost::property<boost::vertex_name_t, NodeID>,
    boost::property<boost::edge_name_t,
                    EdgeID,
                    boost::property<boost::edge_weight_t,
                                    EdgeWeight,
                                    boost::property<edge_unused_tag_t, bool>>>>;

using Vertex = boost::graph_traits<RiGraphBase>::vertex_descriptor;
using Edge = boost::graph_traits<RiGraphBase>::edge_descriptor;

struct EdgeHash
{
    std::size_t operator()(const Edge &e) const noexcept
    {
        BOOST_ASSERT(g != nullptr);
        std::size_t seed = 0;
        boost::hash_combine(seed, source(e, *g));
        boost::hash_combine(seed, target(e, *g));
        return seed;
    }

    const RiGraphBase *g{nullptr};
};

// Represents a sequence of vertices
using Path = std::vector<Vertex>;

// Graph node's edge degree difference (out - in)
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
    // 1-to-all shortest paths
    struct Data
    {
        std::vector<Vertex> preds;
        std::vector<EdgeWeight> dists;
    };

    void initialize(const std::size_t sCount, const std::size_t tCount)
    {
        // sanity check
        if (sCount * tCount > 100000000)
        {
            throw util::exception{"Unexpectedly large PathMatrix (max 100m entries)"};
        }

        sources.resize(sCount);
        targets.resize(tCount);
        data.resize(sCount);
        for (const auto i : util::irange<std::size_t>(0, sCount))
        {
            data[i].preds.resize(tCount);
            data[i].dists.resize(tCount);
        }
    }

    std::vector<Data> data; // 1 entry per source
    std::vector<Vertex> sources;
    std::vector<Vertex> targets;
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

} // namespace osrm::engine::route_inspection::detail

#endif
