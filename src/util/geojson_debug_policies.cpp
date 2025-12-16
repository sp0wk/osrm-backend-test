#include "util/geojson_debug_policies.hpp"
#include "util/coordinate.hpp"
#include "util/geojson_debug_policy_toolkit.hpp"

#include <algorithm>

namespace osrm::util
{

//----------------------------------------------------------------
NodeIdVectorToLineString::NodeIdVectorToLineString(
    const std::vector<util::Coordinate> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}

// converts a vector of node ids into a linestring geojson feature
util::json::Object
NodeIdVectorToLineString::operator()(const std::vector<NodeID> &node_ids,
                                     const std::optional<json::Object> &properties) const
{
    util::json::Array coordinates;
    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   NodeIdToCoordinate(node_coordinates));

    return makeFeature("LineString", std::move(coordinates), properties);
}

//----------------------------------------------------------------
NodeIdVectorToMultiPoint::NodeIdVectorToMultiPoint(
    const std::vector<util::Coordinate> &node_coordinates)
    : node_coordinates(node_coordinates)
{
}

util::json::Object
NodeIdVectorToMultiPoint::operator()(const std::vector<NodeID> &node_ids,
                                     const std::optional<json::Object> &properties) const
{
    util::json::Array coordinates;
    std::transform(node_ids.begin(),
                   node_ids.end(),
                   std::back_inserter(coordinates.values),
                   NodeIdToCoordinate(node_coordinates));

    return makeFeature("MultiPoint", std::move(coordinates), properties);
}

//----------------------------------------------------------------
util::json::Object
CoordinateVectorToMultiPoint::operator()(const std::vector<util::Coordinate> &input_coordinates,
                                         const std::optional<json::Object> &properties) const
{
    auto coordinates = makeJsonArray(input_coordinates);
    return makeFeature("MultiPoint", std::move(coordinates), properties);
}

//----------------------------------------------------------------
util::json::Object
CoordinateVectorToLineString::operator()(const std::vector<util::Coordinate> &input_coordinates,
                                         const std::optional<json::Object> &properties) const
{
    auto coordinates = makeJsonArray(input_coordinates);
    return makeFeature("LineString", std::move(coordinates), properties);
}

//----------------------------------------------------------------
util::json::Object CoordinateVectorsToMultiLineString::operator()(
    const std::vector<std::vector<util::Coordinate>> &coordinates,
    const std::optional<json::Object> &properties) const
{
    util::json::Array full;
    full.values.reserve(coordinates.size());
    for (const auto &line : coordinates)
    {
        full.values.emplace_back(makeJsonArray(line));
    }
    return makeFeature("MultiLineString", std::move(full), properties);
}

} // namespace osrm::util