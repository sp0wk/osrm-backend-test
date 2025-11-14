#include <boost/test/unit_test.hpp>

#include "engine/routing_algorithms/route_inspection.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

BOOST_AUTO_TEST_SUITE(route_inspection)

using namespace osrm::util;
namespace ra = osrm::engine::routing_algorithms;
namespace rad = osrm::engine::routing_algorithms::detail;

// Use node-based BGL graph as RI graph for basic testing
using BaseGraph = BglNodeBasedDynamicGraph;

// helpers
namespace
{

inline auto weight(int w)
{
    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{w};
    return data;
}

template <typename G, typename V> void testEdge(const G &g, V u, V v, int expectedWeight)
{
    const auto [e, exists] = edge(u, v, g);
    BOOST_REQUIRE(exists);
    auto w = get(boost::edge_weight, g, e);
    BOOST_TEST(w == EdgeWeight{expectedWeight});
}

} // namespace

BOOST_AUTO_TEST_CASE(test_not_strongly_connected_graph)
{
    // 0 --> 1 --> 2 --> 3
    //       ↑__________/

    NodeBasedDynamicGraph g;

    auto w = weight(2);
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, w);
    // no v1->v0 edge
    g.InsertEdge(v1, v2, w);
    g.InsertEdge(v2, v3, w);
    g.InsertEdge(v3, v1, w);

    BOOST_TEST(!rad::isStronglyConnectedGraph(BaseGraph{g}));
}

BOOST_AUTO_TEST_CASE(test_shortest_path)
{
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
    auto e13 = g.InsertEdge(v1, v3, weight(100));

    BaseGraph bglGraph{g};

    BOOST_TEST_CONTEXT("Costly 1->3 edge")
    {
        g.GetEdgeData(e13).weight = EdgeWeight{100};
        const auto paths = rad::oneToMany(bglGraph, v0, {v3});

        BOOST_REQUIRE(paths.size() == 1);
        BOOST_TEST(paths[0].cost == EdgeWeight{30});
        const auto &path = paths[0].path;
        BOOST_REQUIRE(path.size() == 4);
        BOOST_TEST(path[0] == v0);
        BOOST_TEST(path[1] == v1);
        BOOST_TEST(path[2] == v2);
        BOOST_TEST(path[3] == v3);
    }

    BOOST_TEST_CONTEXT("Cheap 1->3 edge")
    {
        g.GetEdgeData(e13).weight = EdgeWeight{1};
        const auto paths = rad::oneToMany(bglGraph, v0, {v3});

        BOOST_REQUIRE(paths.size() == 1);
        BOOST_TEST(paths[0].cost == EdgeWeight{11});
        const auto &path = paths[0].path;
        BOOST_REQUIRE(path.size() == 3);
        BOOST_TEST(path[0] == v0);
        BOOST_TEST(path[1] == v1);
        BOOST_TEST(path[2] == v3);
    }
}

BOOST_AUTO_TEST_CASE(test_already_eulerian_graph_trivial)
{
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

    BaseGraph bglGraph{g};
    BOOST_REQUIRE(rad::isEulerianGraph(bglGraph));

    const auto path = rad::routeInspectionImpl(bglGraph, v0);

    BOOST_REQUIRE(path.size() == 5);
    BOOST_TEST(path[0] == v0);
    BOOST_TEST(path[1] == v1);
    BOOST_TEST(path[2] == v2);
    BOOST_TEST(path[3] == v3);
    BOOST_TEST(path[4] == v0);
}

BOOST_AUTO_TEST_CASE(test_already_eulerian_graph)
{
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

    BaseGraph bglGraph{g};
    BOOST_REQUIRE(rad::isEulerianGraph(bglGraph));

    const auto path = rad::routeInspectionImpl(bglGraph, v0);

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

    BaseGraph bglGraph{g};
    BOOST_REQUIRE(!rad::isEulerianGraph(bglGraph));

    const auto path = rad::routeInspectionImpl(bglGraph, v0);

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
    g.InsertEdge(v5, v2, w);
    g.InsertEdge(v5, v3, w);

    BaseGraph bglGraph{g};
    BOOST_REQUIRE(!rad::isEulerianGraph(bglGraph));

    const auto path = rad::routeInspectionImpl(bglGraph, v0);

    BOOST_REQUIRE(path.size() == 16);
    BOOST_TEST(path[0] == v0);
    BOOST_TEST(path[1] == v1);
    BOOST_TEST(path[2] == v2);
    BOOST_TEST(path[3] == v3);
    BOOST_TEST(path[4] == v5);
    BOOST_TEST(path[5] == v2);
    BOOST_TEST(path[6] == v3);
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

    BaseGraph bglGraph{g};
    BOOST_REQUIRE(!rad::isEulerianGraph(bglGraph));

    BOOST_TEST_CONTEXT("Equal weights")
    {
        auto g2 = g;
        BaseGraph bglGraph2{g2};
        const auto path = rad::routeInspectionImpl(bglGraph2, v0);

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
        auto g2 = g;
        g2.GetEdgeData(e71).weight = EdgeWeight{100};
        BaseGraph bglGraph2{g2};

        const auto path = rad::routeInspectionImpl(bglGraph2, v0);

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
    // 0      1  3     5  6      7  9      11
    // o----->o->o---->o->o----->o->o----->o
    // ↑      ↓  ↑               ↓  ↑      ↓
    // o<-----o<-o---------------o<-o<-----o
    // 4      2  10              8  13     12

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

    BaseGraph baseGraph{g};
    BOOST_REQUIRE(rad::isStronglyConnectedGraph(baseGraph));

    auto rig = rad::buildRiGraph(baseGraph, v0);

    BOOST_TEST(rad::isStronglyConnectedGraph(rig));
    BOOST_TEST(num_vertices(rig) == 14);
    BOOST_TEST(num_edges(rig) == 17);

    testEdge(rig, 7, 8, 10);
    testEdge(rig, 11, 12, 20);
    testEdge(rig, 13, 8, 15);
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
    //                   20              16
    //             21 o<-o<------------o<-o 15
    //                |  o8------13>o     ↑
    //                |  o 7        o 14  |
    //                ↓  ↑          |     |
    //             23 o->o 3        ↓     |
    //  6            /  / \         o 18  |
    //  o<-o<5-----2o←-/---o<22-----o 19  |
    //  |            \ | /→o4--------9>o->o 10
    //  ↓              o 1
    //  o 11           ↑
    //  o---------->o->o 0
    //  12         17
    //
    // Intersection connections: 1->2, 1->3, 1->4, 22->2, 22->3, 23->2, 23->3

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

    BaseGraph baseGraph{g};
    BOOST_REQUIRE(rad::isStronglyConnectedGraph(baseGraph));

    auto rig = rad::buildRiGraph(baseGraph, v0);

    BOOST_TEST(rad::isStronglyConnectedGraph(rig));
    BOOST_TEST(num_vertices(rig) == 24);
    BOOST_TEST(num_edges(rig) == 28);

    // TODO optimize graph for RPP
    testEdge(rig, 1, 2, 999);
    testEdge(rig, 1, 3, 10);
    testEdge(rig, 23, 3, 100);
}

BOOST_AUTO_TEST_CASE(test_route_inspection_with_ebg_trivial)
{
    // Input EBG:
    //    0      2      3      5
    // <-----> -----> ----> <----->
    //    1   ↑___________/    6
    //              4

    // Expected NBG:
    // 0      1  3     5  6      7  9      11
    // o----->o->o---->o->o----->o->o----->o
    // ↑      ↓  ↑               ↓  ↑      ↓
    // o<-----o<-o---------------o<-o<-----o
    // 4      2  10              8  13     12

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

    BaseGraph baseGraph{g};
    BOOST_REQUIRE(rad::isStronglyConnectedGraph(baseGraph));
    auto rig = rad::buildRiGraph(baseGraph, v0);
    BOOST_REQUIRE(rad::isStronglyConnectedGraph(rig));

    const auto path = rad::routeInspectionImpl(rig, 0);
    const auto route = ra::prepareFinalRoute(rig, path);

    // TODO fix test by solving RPP
    BOOST_REQUIRE(route.size() == 8);
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
    //

    // Expected NBG:
    //                   20              16
    //             21 o<-o<------------o<-o 15
    //                |  o8------13>o     ↑
    //                |  o 7        o 14  |
    //                ↓  ↑          |     |
    //             23 o->o 3        ↓     |
    //  6            /  / \         o 18  |
    //  o<-o<5-----2o←-/---o<22-----o 19  |
    //  |            \ | /→o4--------9>o->o 10
    //  ↓              o 1
    //  o 11           ↑
    //  o---------->o->o 0
    //  12         17
    //
    // Intersection connections: 1->2, 1->3, 1->4, 22->2, 22->3, 23->2, 23->3

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

    BaseGraph baseGraph{g};
    BOOST_REQUIRE(rad::isStronglyConnectedGraph(baseGraph));
    auto rig = rad::buildRiGraph(baseGraph, v0);
    BOOST_REQUIRE(rad::isStronglyConnectedGraph(rig));

    const auto path = rad::routeInspectionImpl(rig, 0);
    const auto route = ra::prepareFinalRoute(rig, path);

    // TODO fix/adapt test by solving RPP
    BOOST_REQUIRE(route.size() == 17);
    BOOST_TEST(path[0] == 0);
    BOOST_TEST(path[1] == 2);
    BOOST_TEST(path[2] == 5);
    BOOST_TEST(path[3] == 8);
    BOOST_TEST(path[4] == 10);
    BOOST_TEST(path[5] == 1);
    BOOST_TEST(path[6] == 4);
    BOOST_TEST(path[7] == 7);
    BOOST_TEST(path[8] == 8);
    BOOST_TEST(path[9] == 3);
    BOOST_TEST(path[10] == 6);
    BOOST_TEST(path[11] == 9);
    BOOST_TEST(path[12] == 11);
    BOOST_TEST(path[13] == 1);
    BOOST_TEST(path[14] == 4);
    BOOST_TEST(path[15] == 7);
    BOOST_TEST(path[16] == 0);
}

BOOST_AUTO_TEST_SUITE_END()
