#ifndef OSRM_ROUTE_INSPECTION_TYPES_HPP
#define OSRM_ROUTE_INSPECTION_TYPES_HPP

#include "util/typedefs.hpp"

#include <boost/graph/adjacency_list.hpp>

#include <cstdlib>
#include <vector>

namespace osrm::engine::route_inspection::detail
{

// Main graph types (BGL based) used for route inspection implementation
using RiGraphBase = boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    boost::property<boost::vertex_name_t, NodeID>,
    boost::property<boost::edge_name_t, EdgeID, boost::property<boost::edge_weight_t, EdgeWeight>>>;

using Vertex = boost::graph_traits<RiGraphBase>::vertex_descriptor;
using Edge = boost::graph_traits<RiGraphBase>::edge_descriptor;

struct EdgeHash
{
    std::size_t operator()(const Edge &e) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, source(e, g));
        boost::hash_combine(seed, target(e, g));
        return seed;
    }

    const RiGraphBase &g;
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

} // namespace osrm::engine::route_inspection::detail

#endif
