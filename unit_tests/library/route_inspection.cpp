#include <boost/test/unit_test.hpp>

#include "engine/route_inspection/input_graph_adaptors.hpp"
#include "engine/route_inspection/route_inspection.hpp"

#include "osrm/route_inspection_parameters.hpp"
#include <engine/api/flatbuffers/fbresult_generated.h>

#include "osrm/coordinate.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include "util/node_based_graph.hpp"
#include "util/rectangle.hpp"
#include "util/typedefs.hpp"

#include "mocks/mock_datafacade.hpp"

#include "coordinates.hpp"
#include "fixture.hpp"

#include <boost/graph/graph_traits.hpp>

BOOST_AUTO_TEST_SUITE(route_inspection)

using namespace osrm::util;
namespace ri = osrm::engine::route_inspection;
namespace rid = osrm::engine::route_inspection::detail;

//-------------------------------------------------------------------------------------------------

// Mock input graph for tests
class MockInputGraphWrapper
{
  public:
    using Graph = NodeBasedDynamicGraph; // nodes are edges in EBG

    explicit MockInputGraphWrapper(const Graph &g) : g{g} {}

    const auto &GetFacade() const noexcept
    {
        static const osrm::test::MockBaseDataFacade f;
        return f;
    }

    // Interface implementation

    auto GetOutEdgeRange(const NodeID v) const
    {
        const auto edges = g.GetAdjacentEdgeRange(v);
        return boost::adaptors::filter(edges,
                                       [this](const auto e) { return !g.GetEdgeData(e).reversed; });
    }

    EdgeWeight GetEdgeWeight(const NodeID, const EdgeID e) const { return g.GetEdgeData(e).weight; }

    NodeID GetTarget(const EdgeID e) const { return g.GetTarget(e); }

  private:
    const Graph &g;
};

BOOST_CONCEPT_ASSERT((ri::InputGraphConcept<MockInputGraphWrapper>));

//-------------------------------------------------------------------------------------------------

// helpers
namespace
{

inline auto weight(int w)
{
    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{w};
    return data;
}

template <bool useOptimizedGraph = true>
auto makeRiGraph(const NodeBasedDynamicGraph &g, const NodeID start)
{
    MockInputGraphWrapper ig{g};
    auto rig = rid::buildRiGraph(ig, start);
    BOOST_REQUIRE(rid::isStronglyConnectedGraph(rig));
    if constexpr (useOptimizedGraph)
    {
        rid::optimizeRiGraph(rig);
        BOOST_REQUIRE(rid::isStronglyConnectedGraph(rig));
    }
    return rig;
}

template <typename RIG, typename Vertex = boost::graph_traits<RIG>::vertex_descriptor>
auto runRouteInspection(RIG &rig, const Vertex start)
{
    const auto p = ri::routeInspectionImpl(rig, start);
    const auto res = ri::prepareFinalRoute(rig, p);
    return res.nodes;
}

template <typename G, typename V> void testEdge(const G &g, V u, V v, int expectedWeight)
{
    const auto [e, exists] = edge(u, v, g);
    BOOST_REQUIRE(exists);
    auto w = get(boost::edge_weight, g, e);
    BOOST_TEST(w == EdgeWeight{expectedWeight});
}

template <typename G, typename V> void testNoEdge(const G &g, V u, V v)
{
    const auto [_, exists] = edge(u, v, g);
    BOOST_TEST(!exists);
}

} // namespace

BOOST_AUTO_TEST_CASE(test_not_strongly_connected_graph)
{
    // Nodes are EBG nodes
    // 0 <--> 1 --> 2 --> 3 --> 4
    //        ↑__________/

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v1, w);
    g.InsertEdge(v3, v4, w);
    // no v4->v3 edge

    MockInputGraphWrapper ig{g};
    auto rig = rid::buildRiGraph(ig, v0);

    // 3->4 deadend was removed
    BOOST_TEST(rid::isStronglyConnectedGraph(rig));
}

BOOST_AUTO_TEST_CASE(test_shortest_path)
{
    // Nodes are EBG nodes
    // 0 --> 1 --> 2 --> 3
    //       |___________↑

    NodeBasedDynamicGraph g;

    auto w = weight(10);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v0, w);
    auto e13 = g.InsertEdge(v1, v3, weight(100));

    BOOST_TEST_CONTEXT("Costly 1->3 edge")
    {
        g.GetEdgeData(e13).weight = EdgeWeight{100};
        auto rig = makeRiGraph<false>(g, v0);
        const auto [preds, dists] = rid::oneToMany(rig, v0);
        BOOST_REQUIRE(!preds.empty() && !dists.empty());

        BOOST_TEST(dists[v3] == EdgeWeight{30});
        const auto path = rid::extractPath(preds, v0, v3);
        BOOST_REQUIRE(path.size() == 4);
        BOOST_TEST(path[0] == v0);
        BOOST_TEST(path[1] == v1);
        BOOST_TEST(path[2] == v2);
        BOOST_TEST(path[3] == v3);
    }

    BOOST_TEST_CONTEXT("Cheap 1->3 edge")
    {
        g.GetEdgeData(e13).weight = EdgeWeight{1};
        auto rig = makeRiGraph<false>(g, v0);
        const auto [preds, dists] = rid::oneToMany(rig, v0);
        BOOST_REQUIRE(!preds.empty() && !dists.empty());

        BOOST_TEST(dists[v3] == EdgeWeight{11});
        const auto path = rid::extractPath(preds, v0, v3);
        BOOST_REQUIRE(path.size() == 3);
        BOOST_TEST(path[0] == v0);
        BOOST_TEST(path[1] == v1);
        BOOST_TEST(path[2] == v3);
    }
}

BOOST_AUTO_TEST_CASE(test_already_eulerian_graph_trivial)
{
    // Nodes are EBG nodes
    // 0 --> 1 --> 2 --> 3
    // ↑_________________/

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v0, w);

    auto rig = makeRiGraph<false>(g, v0);
    BOOST_REQUIRE(rid::isEulerianGraph(rig));
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 5);
    BOOST_TEST(path[0] == v0);
    BOOST_TEST(path[1] == v1);
    BOOST_TEST(path[2] == v2);
    BOOST_TEST(path[3] == v3);
    BOOST_TEST(path[4] == v0);
}

BOOST_AUTO_TEST_CASE(test_already_eulerian_graph)
{
    // Nodes are EBG nodes
    //  ____________
    // ↓            |
    // 0 <--> 1 --> 2 <--> 3
    // |      ↑__________/ ↑
    // |___________________|

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v0, v3, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v2, v0, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v1, w);
    g.InsertEdge(v3, v2, w);

    auto rig = makeRiGraph<false>(g, v0);
    BOOST_REQUIRE(rid::isEulerianGraph(rig));
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 9);
    BOOST_TEST(path[0] == v0);
    BOOST_TEST(path[1] == v1);
    BOOST_TEST(path[2] == v0);
    BOOST_TEST(path[3] == v3);
    BOOST_TEST(path[4] == v1);
    BOOST_TEST(path[5] == v2);
    BOOST_TEST(path[6] == v3);
    BOOST_TEST(path[7] == v2);
    BOOST_TEST(path[8] == v0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_trivial)
{
    // Nodes are EBG nodes
    // 0 <--> 1 --> 2 <--> 3
    //        ↑           /
    //        |__________/

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode(); // 1 surplus
    auto v3 = g.InsertNode(); // 1 deficit
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v1, w);
    g.InsertEdge(v3, v2, w);

    auto rig = makeRiGraph<false>(g, v0);
    BOOST_REQUIRE(!rid::isEulerianGraph(rig));
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 8);
    BOOST_TEST(path[0] == v0);
    BOOST_TEST(path[1] == v1);
    BOOST_TEST(path[2] == v2);
    BOOST_TEST(path[3] == v3);
    BOOST_TEST(path[4] == v2);
    BOOST_TEST(path[5] == v3);
    BOOST_TEST(path[6] == v1);
    BOOST_TEST(path[7] == v0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection)
{
    // Nodes are EBG nodes
    // .----- 4      _____________
    // ↓      ↑     ↓             |
    // 0 <--> 1 --> 2 <--> 3 <--> 5
    //        ↑           /
    //        |__________/

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode(); // 1 surplus
    auto v1 = g.InsertNode(); // 1 deficit
    auto v2 = g.InsertNode(); // 2 surplus
    auto v3 = g.InsertNode(); // 1 deficit
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode(); // 1 deficit
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v1, v4, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v1, w);
    g.InsertEdge(v3, v2, w);
    g.InsertEdge(v3, v5, w);
    g.InsertEdge(v4, v0, w);
    g.InsertEdge(v5, v2, weight(10));
    g.InsertEdge(v5, v3, w);

    auto rig = makeRiGraph<false>(g, v0);
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 16);
    BOOST_TEST(path[0] == v0);
    BOOST_TEST(path[1] == v1);
    BOOST_TEST(path[2] == v2);
    BOOST_TEST(path[3] == v3);
    BOOST_TEST(path[4] == v2);
    BOOST_TEST(path[5] == v3);
    BOOST_TEST(path[6] == v5);
    BOOST_TEST(path[7] == v2);
    BOOST_TEST(path[8] == v3);
    BOOST_TEST(path[9] == v5);
    BOOST_TEST(path[10] == v3);
    BOOST_TEST(path[11] == v1);
    BOOST_TEST(path[12] == v0);
    BOOST_TEST(path[13] == v1);
    BOOST_TEST(path[14] == v4);
    BOOST_TEST(path[15] == v0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_costing)
{
    // Nodes are EBG nodes
    //        .---> 2 --> 3 ---.
    //        |                ↓
    // 0 <--> 1 -------------> 4 --> 6
    //        ↑ \___→ 5 _______↑     |
    //        |                      |
    //        |_________ 7 ←_________|
    //        ↑____ 8 ←_/

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode(); // 1 deficit
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode(); // 2 surplus
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode(); // 1 deficit
    auto v8 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v1, v4, w);
    g.InsertEdge(v1, v5, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v4, w);
    g.InsertEdge(v4, v6, w);
    g.InsertEdge(v5, v4, w);
    g.InsertEdge(v6, v7, w);
    auto e71 = g.InsertEdge(v7, v1, w);
    g.InsertEdge(v7, v8, w);
    g.InsertEdge(v8, v1, w);

    BOOST_TEST_CONTEXT("Equal weights")
    {
        auto rig = makeRiGraph<false>(g, v0);
        const auto path = runRouteInspection(rig, 0);

        BOOST_REQUIRE(path.size() == 19);
        BOOST_TEST(path[0] == v0);
        BOOST_TEST(path[1] == v1);
        BOOST_TEST(path[2] == v2);
        BOOST_TEST(path[3] == v3);
        BOOST_TEST(path[4] == v4);
        BOOST_TEST(path[5] == v6);
        BOOST_TEST(path[6] == v7);
        BOOST_TEST(path[7] == v1);
        BOOST_TEST(path[8] == v4);
        BOOST_TEST(path[9] == v6);
        BOOST_TEST(path[10] == v7);
        BOOST_TEST(path[11] == v8);
        BOOST_TEST(path[12] == v1);
        BOOST_TEST(path[13] == v5);
        BOOST_TEST(path[14] == v4);
        BOOST_TEST(path[15] == v6);
        // 7 -> 1 is cheaper
        BOOST_TEST(path[16] == v7);
        BOOST_TEST(path[17] == v1);
        BOOST_TEST(path[18] == v0);
    }

    BOOST_TEST_CONTEXT("Costly 7->1 edge")
    {
        g.GetEdgeData(e71).weight = EdgeWeight{100};
        auto rig = makeRiGraph<false>(g, v0);
        const auto path = runRouteInspection(rig, 0);

        BOOST_REQUIRE(path.size() == 20);
        BOOST_TEST(path[0] == v0);
        BOOST_TEST(path[1] == v1);
        BOOST_TEST(path[2] == v2);
        BOOST_TEST(path[3] == v3);
        BOOST_TEST(path[4] == v4);
        BOOST_TEST(path[5] == v6);
        BOOST_TEST(path[6] == v7);
        BOOST_TEST(path[7] == v1);
        BOOST_TEST(path[8] == v4);
        BOOST_TEST(path[9] == v6);
        BOOST_TEST(path[10] == v7);
        BOOST_TEST(path[11] == v8);
        BOOST_TEST(path[12] == v1);
        BOOST_TEST(path[13] == v5);
        BOOST_TEST(path[14] == v4);
        BOOST_TEST(path[15] == v6);
        // 7 -> 8 -> 1 is cheaper
        BOOST_TEST(path[16] == v7);
        BOOST_TEST(path[17] == v8);
        BOOST_TEST(path[18] == v1);
        BOOST_TEST(path[19] == v0);
    }
}

BOOST_AUTO_TEST_CASE(test_build_ri_graph_from_ebg_trivial)
{
    // Input EBG:
    //    0      2      3      5
    // <-----> -----> ----> <----->
    //    1   ↑___________/    6
    //              4

    // Expected NBG:
    //   0      2      3      5
    //   o----->o----->o----->o
    //   ↨      ↑     /       ↓
    //   o<------\---o<-------o
    //   1           4        6

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v0, v2, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v4, weight(10));
    g.InsertEdge(v3, v5, w);
    g.InsertEdge(v4, v1, w);
    g.InsertEdge(v4, v2, w);
    g.InsertEdge(v5, v6, weight(20));
    g.InsertEdge(v6, v4, weight(15));

    auto rig = makeRiGraph(g, v0);

    BOOST_TEST(num_vertices(rig) == 7);
    BOOST_TEST(num_edges(rig) == 7); // 3 edges were optimized out

    testNoEdge(rig, 0, 1);
    testNoEdge(rig, 3, 4);
    testEdge(rig, 5, 6, 20);
    testEdge(rig, 6, 4, 15);
    testNoEdge(rig, 4, 2);
}

BOOST_AUTO_TEST_CASE(test_build_ri_graph_from_ebg_complex_intersection)
{
    // Input EBG:
    //               <-------9-------↑
    //               | ↑----5----->  |
    //             11| 2          |  |
    //               ↓ |          8  6
    //     <---1---- *** <---10---↓  |
    //     |         *** -----3----->|
    //     4          ↑
    //     ↓----7---->| 0
    //
    // Intersection connections: 0->1, 0->2, 0->3, 10->1, 10->2, 11->1, 11->2
    //

    // Expected NBG:
    //             11 o<------------------o 9
    //                |  o5-------->o 8   ↑
    //                |  ↑          |     |
    //                ↓  |          |     |
    //                |->o 2        |     |
    //  4          1 /  / \__       ↓     |
    //  o<----------o←-/-----↑------o 10  |
    //  |            \ | /→o------------->o 6
    //  |              |   3
    //  ↓              |
    //  o------------->o 0
    //  7

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode();
    auto v8 = g.InsertNode();
    auto v9 = g.InsertNode();
    auto v10 = g.InsertNode();
    auto v11 = g.InsertNode();
    g.InsertEdge(v0, v1, weight(999)); // costly maneuver
    g.InsertEdge(v0, v2, weight(10));  // slightly costly
    g.InsertEdge(v0, v3, w);
    g.InsertEdge(v1, v4, w);
    g.InsertEdge(v2, v5, w);
    g.InsertEdge(v3, v6, w);
    g.InsertEdge(v4, v7, w);
    g.InsertEdge(v5, v8, w);
    g.InsertEdge(v6, v9, w);
    g.InsertEdge(v7, v0, w);
    g.InsertEdge(v8, v10, w);
    g.InsertEdge(v9, v11, w);
    g.InsertEdge(v10, v1, w);
    g.InsertEdge(v10, v2, w);
    g.InsertEdge(v11, v1, w);
    g.InsertEdge(v11, v2, weight(100)); // costly uturn

    auto rig = makeRiGraph(g, v0);

    BOOST_TEST(num_vertices(rig) == 12);
    BOOST_TEST(num_edges(rig) == 13); // 3 edges were optimized out

    testNoEdge(rig, 0, 1);
    testEdge(rig, 0, 2, 10);
    testNoEdge(rig, 11, 2);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_trivial)
{
    // Input EBG:
    //    0      2      3      5
    // <-----> -----> ----> <----->
    //    1   ↑___________/    6
    //              4

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v0, v2, w);
    g.InsertEdge(v1, v0, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v4, w);
    g.InsertEdge(v3, v5, w);
    g.InsertEdge(v4, v1, w);
    g.InsertEdge(v4, v2, w);
    g.InsertEdge(v5, v6, w);
    g.InsertEdge(v6, v4, w);

    auto rig = makeRiGraph(g, v0);
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 8);
    BOOST_TEST(path[0] == 0);
    BOOST_TEST(path[1] == 2);
    BOOST_TEST(path[2] == 3);
    BOOST_TEST(path[3] == 5);
    BOOST_TEST(path[4] == 6);
    BOOST_TEST(path[5] == 4);
    BOOST_TEST(path[6] == 1);
    BOOST_TEST(path[7] == 0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_complex_intersection)
{
    // Input EBG:
    //               <-------9-------↑
    //               | ↑----5----->  |
    //             11| 2          |  |
    //               ↓ |          8  6
    //     <---1---- *** <---10---↓  |
    //     |         *** -----3----->|
    //     4          ↑
    //     ↓----7---->| 0
    //
    // Intersection connections: 0->1, 0->2, 0->3, 10->1, 10->2, 11->1, 11->2

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode();
    auto v8 = g.InsertNode();
    auto v9 = g.InsertNode();
    auto v10 = g.InsertNode();
    auto v11 = g.InsertNode();
    g.InsertEdge(v0, v1, weight(999)); // costly maneuver
    g.InsertEdge(v0, v2, weight(10));  // slightly costly
    g.InsertEdge(v0, v3, w);
    g.InsertEdge(v1, v4, w);
    g.InsertEdge(v2, v5, w);
    g.InsertEdge(v3, v6, w);
    g.InsertEdge(v4, v7, w);
    g.InsertEdge(v5, v8, w);
    g.InsertEdge(v6, v9, w);
    g.InsertEdge(v7, v0, w);
    g.InsertEdge(v8, v10, w);
    g.InsertEdge(v9, v11, w);
    g.InsertEdge(v10, v1, weight(5));
    g.InsertEdge(v10, v2, w);
    g.InsertEdge(v11, v1, w);
    g.InsertEdge(v11, v2, weight(100)); // costly uturn

    auto rig = makeRiGraph(g, v0);
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 17);
    BOOST_TEST(path[0] == 0);
    BOOST_TEST(path[1] == 2);
    BOOST_TEST(path[2] == 5);
    BOOST_TEST(path[3] == 8);
    BOOST_TEST(path[4] == 10);
    BOOST_TEST(path[5] == 1);
    BOOST_TEST(path[6] == 4);
    BOOST_TEST(path[7] == 7);
    BOOST_TEST(path[8] == 0);
    BOOST_TEST(path[9] == 3);
    BOOST_TEST(path[10] == 6);
    BOOST_TEST(path[11] == 9);
    BOOST_TEST(path[12] == 11);
    BOOST_TEST(path[13] == 1);
    BOOST_TEST(path[14] == 4);
    BOOST_TEST(path[15] == 7);
    BOOST_TEST(path[16] == 0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_deadend_eulerian)
{
    // Input EBG:
    //
    //   4   2 3
    //   o<--o o----->o 5
    //   |    \↑      |
    //   ↓     |      |
    // 6 o---->o 1    |
    //         ↑      |
    //         |      ↓
    //       0 o<-----o 7

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v1, v3, w);
    g.InsertEdge(v2, v4, w);
    g.InsertEdge(v3, v5, w);
    g.InsertEdge(v4, v6, w);
    g.InsertEdge(v5, v7, w);
    g.InsertEdge(v6, v1, w);
    g.InsertEdge(v7, v0, w);

    auto rig = makeRiGraph(g, v0);
    BOOST_REQUIRE(rid::isEulerianGraph(rig));
    const auto path = runRouteInspection(rig, 0);

    BOOST_REQUIRE(path.size() == 10);
    BOOST_TEST(path[0] == 0);
    BOOST_TEST(path[1] == 1);
    BOOST_TEST(path[2] == 2);
    BOOST_TEST(path[3] == 4);
    BOOST_TEST(path[4] == 6);
    BOOST_TEST(path[5] == 1);
    BOOST_TEST(path[6] == 3);
    BOOST_TEST(path[7] == 5);
    BOOST_TEST(path[8] == 7);
    BOOST_TEST(path[9] == 0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_deadend)
{
    // Input EBG:
    //
    //   4   2 3
    //   o<--o o----->o 5
    //   |    \↑      |\_
    //   ↓     |      |  \_→ o 7
    // 6 o---->o 1    |      |
    //         ↑      |  ___/
    //         |      ↓ /
    //       0 o<-----o 8

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode();
    auto v8 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v2, weight(5));
    g.InsertEdge(v1, v3, weight(10));
    g.InsertEdge(v2, v4, w);
    g.InsertEdge(v3, v5, w);
    g.InsertEdge(v4, v6, w);
    g.InsertEdge(v5, v7, weight(10));
    g.InsertEdge(v5, v8, weight(20));
    g.InsertEdge(v6, v1, w);
    g.InsertEdge(v7, v8, weight(12));
    g.InsertEdge(v8, v0, w);

    auto rig = makeRiGraph(g, v0);
    const auto path = runRouteInspection(rig, 0);

    // TODO fux by removing unused shortcuts
    BOOST_REQUIRE(path.size() == 11);
    BOOST_TEST(path[0] == 0);
    BOOST_TEST(path[1] == 1);
    BOOST_TEST(path[2] == 2);
    BOOST_TEST(path[3] == 4);
    BOOST_TEST(path[4] == 6);
    BOOST_TEST(path[5] == 1);
    BOOST_TEST(path[6] == 3);
    BOOST_TEST(path[7] == 5);
    BOOST_TEST(path[8] == 7);
    BOOST_TEST(path[9] == 8);
    BOOST_TEST(path[10] == 0);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_many_disjoints)
{
    // Input EBG:
    //
    //       9 o<------------|
    //        /|             |
    //   4  2↓ ↓3            |
    //   o<--o o----->o 5    |
    //   |    \↑      |\_    |
    //   ↓     |      |  \_→ o 7
    // 6 o---->o 1    |      |
    //   |     ↑      |  ___/
    //   |     |      ↓ /
    //   |---->o<-----o 8
    //         0

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode();
    auto v8 = g.InsertNode();
    auto v9 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    g.InsertEdge(v1, v2, weight(5));
    g.InsertEdge(v1, v3, weight(10));
    g.InsertEdge(v2, v4, w);
    g.InsertEdge(v3, v5, w);
    g.InsertEdge(v4, v6, w);
    g.InsertEdge(v5, v7, weight(10));
    auto e58 = g.InsertEdge(v5, v8, weight(20));
    g.InsertEdge(v6, v0, weight(100));
    g.InsertEdge(v6, v1, w);
    g.InsertEdge(v7, v8, weight(12));
    g.InsertEdge(v7, v9, weight(30));
    g.InsertEdge(v8, v0, w);
    g.InsertEdge(v9, v2, weight(10));
    g.InsertEdge(v9, v3, weight(8));

    BOOST_TEST_CONTEXT("Use 5->8 edge")
    {
        auto rig = makeRiGraph(g, v0);
        const auto path = runRouteInspection(rig, 0);

        BOOST_REQUIRE(path.size() == 14);
        BOOST_TEST(path[0] == 0);
        BOOST_TEST(path[1] == 1);
        BOOST_TEST(path[2] == 2);
        BOOST_TEST(path[3] == 4);
        BOOST_TEST(path[4] == 6);
        BOOST_TEST(path[5] == 1);
        BOOST_TEST(path[6] == 3);
        BOOST_TEST(path[7] == 5);
        BOOST_TEST(path[8] == 7);
        BOOST_TEST(path[9] == 9);
        BOOST_TEST(path[10] == 3);
        BOOST_TEST(path[11] == 5);
        BOOST_TEST(path[12] == 8);
        BOOST_TEST(path[13] == 0);
    }

    BOOST_TEST_CONTEXT("Avoid 5->8 edge")
    {
        g.GetEdgeData(e58).weight = EdgeWeight{100};
        auto rig = makeRiGraph(g, v0);
        const auto path = runRouteInspection(rig, 0);

        BOOST_REQUIRE(path.size() == 15);
        BOOST_TEST(path[0] == 0);
        BOOST_TEST(path[1] == 1);
        BOOST_TEST(path[2] == 2);
        BOOST_TEST(path[3] == 4);
        BOOST_TEST(path[4] == 6);
        BOOST_TEST(path[5] == 1);
        BOOST_TEST(path[6] == 3);
        BOOST_TEST(path[7] == 5);
        BOOST_TEST(path[8] == 7);
        BOOST_TEST(path[9] == 9);
        BOOST_TEST(path[10] == 3);
        BOOST_TEST(path[11] == 5);
        BOOST_TEST(path[12] == 7);
        BOOST_TEST(path[13] == 8);
        BOOST_TEST(path[14] == 0);
    }
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_optimal_cost)
{
    // Input EBG:
    //     <--5---<-----3------↑
    //     |      |\           |
    //     |    6 | | 4        | 1
    //     7      | ↑<----2----↑
    //     |      ↓----8-----→/|
    //     |                   | 0
    //     ↓---------9-------->|

    // Expected NBG:
    // 7           5
    // o<----------o<------------o 3
    // |          /\             ↑
    // |       6 o  ↑            |
    // |         |  4            |
    // |         ↓  o<--------2o\o 1
    // |       8 o------------/>/↑
    // |                         |
    // ↓                         |
    // o------------------------>o 0
    // 9

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode();
    auto v8 = g.InsertNode();
    auto v9 = g.InsertNode();
    g.InsertEdge(v0, v1, weight(10));
    g.InsertEdge(v0, v2, weight(20));
    g.InsertEdge(v1, v3, w);
    g.InsertEdge(v2, v4, w);
    g.InsertEdge(v3, v5, weight(10));
    g.InsertEdge(v3, v6, weight(30));
    g.InsertEdge(v4, v5, weight(40));
    g.InsertEdge(v4, v6, weight(25));
    g.InsertEdge(v5, v7, w);
    g.InsertEdge(v6, v8, w);
    g.InsertEdge(v7, v9, w);
    g.InsertEdge(v8, v1, weight(45));
    g.InsertEdge(v8, v2, weight(30));
    g.InsertEdge(v9, v0, w);

    auto rig = makeRiGraph(g, v0);
    const auto path = runRouteInspection(rig, 0);

    // TODO fix test with better edge pruning heuristic
    // Current cost: 10 + 2 + 30 + 2 + 30 + 2 + 40 + 2 + 2 + 2 = 122
    // Minimum cost: 20 + 2 + 25 + 2 + 45 + 2 + 10 + 2 + 2 + 2 = 112
    BOOST_REQUIRE(path.size() == 11);
    BOOST_TEST(path[0] == 0);
    BOOST_TEST(path[1] == 2);
    BOOST_TEST(path[2] == 4);
    BOOST_TEST(path[3] == 6);
    BOOST_TEST(path[4] == 8);
    BOOST_TEST(path[5] == 1);
    BOOST_TEST(path[6] == 3);
    BOOST_TEST(path[7] == 5);
    BOOST_TEST(path[8] == 7);
    BOOST_TEST(path[9] == 9);
    BOOST_TEST(path[10] == 0);
}

//-------------------------------------------------------------------------------------------------
// Integration tests
//-------------------------------------------------------------------------------------------------

// sends RI request
osrm::Status run_route_inspection_json(const osrm::OSRM &osrm,
                                       const osrm::RouteInspectionParameters &params,
                                       osrm::json::Object &json_result,
                                       bool use_json_only_api)
{
    if (use_json_only_api)
    {
        return osrm.RouteInspection(params, json_result);
    }
    osrm::engine::api::ResultT result = osrm::json::Object();
    auto rc = osrm.RouteInspection(params, result);
    json_result = std::get<osrm::json::Object>(result);
    return rc;
}

using Polygon = osrm::engine::api::RouteInspectionParameters::Polygon;

Polygon makePolygon(const std::vector<std::pair<double, double>> &points)
{
    using namespace osrm::util;
    Polygon polygon;
    polygon.reserve(points.size() + 1);
    for (const auto &point : points)
    {
        polygon.emplace_back(FloatLongitude{point.first}, FloatLatitude{point.second});
    }
    if (polygon.front() != polygon.back())
    {
        polygon.emplace_back(polygon.front());
    }
    return polygon;
}

Polygon makePolygon(const osrm::util::RectangleInt2D bb)
{
    using namespace osrm::util;
    const auto min_lon = toFloating(bb.min_lon).__value;
    const auto max_lon = toFloating(bb.max_lon).__value;
    const auto min_lat = toFloating(bb.min_lat).__value;
    const auto max_lat = toFloating(bb.max_lat).__value;
    return makePolygon(
        {{min_lon, min_lat}, {min_lon, max_lat}, {max_lon, max_lat}, {max_lon, min_lat}});
}

//-------------------------------------------------------------------------------------------------

void test_ri_response_for_small_area(bool use_json_only_api)
{
    using namespace osrm;
    using namespace osrm::util;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/mld/monaco.osrm", osrm::EngineConfig::Algorithm::MLD);
    const auto location =
        Location{util::FloatLongitude{7.434346008945369}, util::FloatLatitude{43.74749060149961}};

    RouteInspectionParameters params;
    params.coordinates.push_back(location);
    params.coordinates.push_back(location);

    params.polygon = makePolygon({{7.430766291940898, 43.74481022468697},
                                  {7.43360907004822, 43.74387679776672},
                                  {7.4356746587683915, 43.7469213894843},
                                  {7.434308316015745, 43.74828500179743},
                                  {7.432749267418956, 43.7476471014152},
                                  {7.430766291940898, 43.74481022468697}});

    json::Object json_result;
    const auto rc = run_route_inspection_json(osrm, params, json_result, use_json_only_api);
    BOOST_CHECK(rc == Status::Ok);

    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");
    BOOST_REQUIRE(code == "Ok");

    const auto &waypoints = std::get<json::Array>(json_result.values.at("waypoints")).values;
    BOOST_REQUIRE(waypoints.size() >= 2);

    auto const getWaypointCoord = [](const auto &waypoint)
    {
        const auto &waypoint_object = std::get<json::Object>(waypoint);
        const auto location = std::get<json::Array>(waypoint_object.values.at("location")).values;
        const auto longitude = std::get<json::Number>(location[0]).value;
        const auto latitude = std::get<json::Number>(location[1]).value;
        return util::FloatCoordinate{util::FloatLongitude{longitude},
                                     util::FloatLatitude{latitude}};
    };

    const auto firstWp = getWaypointCoord(waypoints.front());
    const auto lastWp = getWaypointCoord(waypoints.back());
    BOOST_TEST((firstWp == lastWp));

    for (const auto &waypoint : waypoints)
    {
        const auto wp = getWaypointCoord(waypoint);
        BOOST_TEST(wp.IsValid());
    }
}
BOOST_AUTO_TEST_CASE(test_ri_response_for_small_area_old_api)
{
    test_ri_response_for_small_area(true);
}
BOOST_AUTO_TEST_CASE(test_ri_response_for_small_area_new_api)
{
    test_ri_response_for_small_area(false);
}

//-------------------------------------------------------------------------------------------------

void test_ri_response_for_large_monaco_area(bool use_json_only_api)
{
    using namespace osrm;
    using namespace osrm::util;

    auto osrm = getOSRM(OSRM_TEST_DATA_DIR "/mld/monaco.osrm", osrm::EngineConfig::Algorithm::MLD);
    const auto location = Location(util::FloatLongitude{7.416055}, util::FloatLatitude{43.730143});

    RouteInspectionParameters params;
    params.coordinates.push_back(location);
    params.coordinates.push_back(location);

    const RectangleInt2D rect{FloatLongitude(7.406543521200644),
                              FloatLongitude{7.437528399740683},
                              FloatLatitude{43.724536932824506},
                              FloatLatitude{43.7418400686539}};
    params.polygon = makePolygon(rect);

    json::Object json_result;
    const auto rc = run_route_inspection_json(osrm, params, json_result, use_json_only_api);
    BOOST_CHECK(rc == Status::Ok);

    const auto code = std::get<json::String>(json_result.values.at("code")).value;
    BOOST_CHECK_EQUAL(code, "Ok");
    BOOST_REQUIRE(code == "Ok");

    const auto &waypoints = std::get<json::Array>(json_result.values.at("waypoints")).values;
    BOOST_REQUIRE(waypoints.size() >= 2);

    auto const getWaypointCoord = [](const auto &waypoint)
    {
        const auto &waypoint_object = std::get<json::Object>(waypoint);
        const auto location = std::get<json::Array>(waypoint_object.values.at("location")).values;
        const auto longitude = std::get<json::Number>(location[0]).value;
        const auto latitude = std::get<json::Number>(location[1]).value;
        return util::FloatCoordinate{util::FloatLongitude{longitude},
                                     util::FloatLatitude{latitude}};
    };

    const auto firstWp = getWaypointCoord(waypoints.front());
    const auto lastWp = getWaypointCoord(waypoints.back());
    BOOST_TEST((firstWp == lastWp));

    for (const auto &waypoint : waypoints)
    {
        const auto wp = getWaypointCoord(waypoint);
        BOOST_TEST(wp.IsValid());
    }
}
BOOST_AUTO_TEST_CASE(test_ri_response_for_large_monaco_area_old_api)
{
    test_ri_response_for_large_monaco_area(true);
}
BOOST_AUTO_TEST_CASE(test_ri_response_for_large_monaco_area_new_api)
{
    test_ri_response_for_large_monaco_area(false);
}

BOOST_AUTO_TEST_SUITE_END()
