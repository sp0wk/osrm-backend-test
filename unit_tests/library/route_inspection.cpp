#include <boost/test/unit_test.hpp>

#include "engine/routing_algorithms/route_inspection.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

BOOST_AUTO_TEST_SUITE(route_inspection)

using namespace osrm::util;
namespace ra = osrm::engine::routing_algorithms;
namespace rad = osrm::engine::routing_algorithms::detail;

BOOST_AUTO_TEST_CASE(test_not_strongly_connected_graph)
{
    // 0 --> 1 --> 2 --> 3
    //       ↑__________/

    NodeBasedDynamicGraph g;

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{2};
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, data);
    // no v1->v0 edge
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v2, v3, data);
    g.InsertEdge(v3, v1, data);

    BOOST_TEST(!rad::isStronglyConnectedGraph(rad::BaseGraph{g}));
}

BOOST_AUTO_TEST_CASE(test_shortest_path)
{
    // 0 --> 1 --> 2 --> 3
    //       |___________↑

    NodeBasedDynamicGraph g;

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{10};
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, data);
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v2, v3, data);
    data.weight = EdgeWeight{100};
    auto e13 = g.InsertEdge(v1, v3, data);

    rad::BaseGraph bglGraph{g};

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
    const NodeID source{0};

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{2};
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, data);
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v2, v3, data);
    g.InsertEdge(v3, v0, data);
    BOOST_REQUIRE(rad::isEulerianGraph(rad::BaseGraph{g}));

    const auto path = ra::routeInspection(g, source);
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
    const NodeID source{0};

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{2};
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    g.InsertEdge(v0, v1, data);
    g.InsertEdge(v0, v3, data);
    g.InsertEdge(v1, v0, data);
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v2, v0, data);
    g.InsertEdge(v2, v3, data);
    g.InsertEdge(v3, v1, data);
    g.InsertEdge(v3, v2, data);
    BOOST_REQUIRE(rad::isEulerianGraph(rad::BaseGraph{g}));

    const auto path = ra::routeInspection(g, source);
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
    const NodeID source{0};

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{2};
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode();
    auto v2 = g.InsertNode(); // 1 surplus
    auto v3 = g.InsertNode(); // 1 deficit
    g.InsertEdge(v0, v1, data);
    g.InsertEdge(v1, v0, data);
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v2, v3, data);
    g.InsertEdge(v3, v1, data);
    g.InsertEdge(v3, v2, data);
    BOOST_REQUIRE(!rad::isEulerianGraph(rad::BaseGraph{g}));

    const auto path = ra::routeInspection(g, source);
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
    const NodeID source{0};

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{2};
    auto v0 = g.InsertNode(); // 1 surplus
    auto v1 = g.InsertNode(); // 1 deficit
    auto v2 = g.InsertNode(); // 2 surplus
    auto v3 = g.InsertNode(); // 1 deficit
    auto v4 = g.InsertNode();
    auto v5 = g.InsertNode(); // 1 deficit
    g.InsertEdge(v0, v1, data);
    g.InsertEdge(v1, v0, data);
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v1, v4, data);
    g.InsertEdge(v2, v3, data);
    g.InsertEdge(v3, v1, data);
    g.InsertEdge(v3, v2, data);
    g.InsertEdge(v3, v5, data);
    g.InsertEdge(v4, v0, data);
    g.InsertEdge(v5, v2, data);
    g.InsertEdge(v5, v3, data);
    BOOST_REQUIRE(!rad::isEulerianGraph(rad::BaseGraph{g}));

    const auto path = ra::routeInspection(g, source);

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
    //        .---> 2 --> 3 ---.
    //        |                ↓
    // 0 <--> 1 -------------> 4 --> 6
    //        ↑ \___→ 5 _______↑     |
    //        |                      |
    //        |_________ 7 ←_________|
    //        ↑____ 8 ←_/

    NodeBasedDynamicGraph g;
    const NodeID source{0};

    NodeBasedDynamicGraph::EdgeData data;
    data.weight = EdgeWeight{2};
    auto v0 = g.InsertNode();
    auto v1 = g.InsertNode(); // 1 deficit
    auto v2 = g.InsertNode();
    auto v3 = g.InsertNode();
    auto v4 = g.InsertNode(); // 2 surplus
    auto v5 = g.InsertNode();
    auto v6 = g.InsertNode();
    auto v7 = g.InsertNode(); // 1 deficit
    auto v8 = g.InsertNode();
    g.InsertEdge(v0, v1, data);
    g.InsertEdge(v1, v0, data);
    g.InsertEdge(v1, v2, data);
    g.InsertEdge(v1, v4, data);
    g.InsertEdge(v1, v5, data);
    g.InsertEdge(v2, v3, data);
    g.InsertEdge(v3, v4, data);
    g.InsertEdge(v4, v6, data);
    g.InsertEdge(v5, v4, data);
    g.InsertEdge(v6, v7, data);
    auto e71 = g.InsertEdge(v7, v1, data);
    g.InsertEdge(v7, v8, data);
    g.InsertEdge(v8, v1, data);
    BOOST_REQUIRE(!rad::isEulerianGraph(rad::BaseGraph{g}));

    BOOST_TEST_CONTEXT("Equal weights")
    {
        auto graph = g;
        const auto path = ra::routeInspection(graph, source);
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
        auto graph = g;
        graph.GetEdgeData(e71).weight = EdgeWeight{100};
        const auto path = ra::routeInspection(graph, source);
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

BOOST_AUTO_TEST_SUITE_END()
