#ifndef OSRM_RI_DEBUG_LOGGER_HPP
#define OSRM_RI_DEBUG_LOGGER_HPP

#include "util/coordinate.hpp"
#include "util/geojson_debug_logger.hpp"
#include "util/geojson_debug_policies.hpp"
#include "util/geojson_debug_policy_toolkit.hpp"

#include <cstdlib>
#include <map>
#include <vector>

namespace osrm::engine::route_inspection::detail
{

// NOTE: Feature order is important for proper layering in GeoJson output
enum class DebugFeatureType : uint8_t
{
    UNKNOWN = 0,
    START_NODE,
    DEADEND_NODES,
    DUPLICATE_EDGES,
    COSTLY_EDGES,
    UNUSED_EDGES,
    USED_EDGES,
    DISJOINT_IN_NODES,
    DISJOINT_OUT_NODES
};

inline std::string toString(const DebugFeatureType t)
{
    switch (t)
    {
    case DebugFeatureType::START_NODE:
        return "START_NODE";
    case DebugFeatureType::DEADEND_NODES:
        return "DEADEND_NODES";
    case DebugFeatureType::DUPLICATE_EDGES:
        return "DUPLICATE_EDGES";
    case DebugFeatureType::COSTLY_EDGES:
        return "COSTLY_EDGES";
    case DebugFeatureType::UNUSED_EDGES:
        return "UNUSED_EDGES";
    case DebugFeatureType::USED_EDGES:
        return "USED_EDGES";
    case DebugFeatureType::DISJOINT_IN_NODES:
        return "DISJOINT_IN_NODES";
    case DebugFeatureType::DISJOINT_OUT_NODES:
        return "DISJOINT_OUT_NODES";
    default:
        return "UNKNOWN";
    }
    return "UNKNOWN";
}

/**
 * \brief Route inspection debugger which generates GeoJson plot for the result.
 */
class RiDebugLogger final
{
  public:
    explicit RiDebugLogger(const std::string &plotFileName = "ri_debug.geojson")
        : logger_{plotFileName}
    {
    }

    ~RiDebugLogger()
    {
        for (const auto &[k, f] : features_)
        {
            util::json::Object style;
            switch (k)
            {
            case DebugFeatureType::START_NODE:
                style = util::makeStyle(util::xxx_large, util::green);
                break;
            case DebugFeatureType::DEADEND_NODES:
                style = util::makeStyle(util::xx_large, util::black);
                break;
            case DebugFeatureType::DUPLICATE_EDGES:
                style = util::makeStyle(util::xx_large, util::purple);
                break;
            case DebugFeatureType::COSTLY_EDGES:
                style = util::makeStyle(util::extra_large, util::orange);
                break;
            case DebugFeatureType::UNUSED_EDGES:
                style = util::makeStyle(util::large, util::red);
                break;
            case DebugFeatureType::USED_EDGES:
                style = util::makeStyle(util::medium, util::blue);
                break;
            case DebugFeatureType::DISJOINT_IN_NODES:
                style = util::makeStyle(util::small, util::cyan);
                break;
            case DebugFeatureType::DISJOINT_OUT_NODES:
                style = util::makeStyle(util::small, util::yellow);
                break;
            default:
                style = util::makeStyle(util::medium, util::brown);
            }

            logger_.Write(f, style);
        }
    }

    void logVertex(const util::Coordinate start,
                   const util::Coordinate end,
                   const DebugFeatureType type = DebugFeatureType::UNKNOWN)
    {
        auto &feature = features_[type];
        feature.emplace_back(std::vector{start, end});
    }

    void logEdge(const util::Coordinate start,
                 const util::Coordinate middle,
                 const util::Coordinate end,
                 const DebugFeatureType type = DebugFeatureType::UNKNOWN)
    {
        auto &feature = features_[type];
        feature.emplace_back(std::vector{start, middle, end});
    }

  private:
    util::ScopedGeojsonLoggerGuard<util::CoordinateVectorsToMultiLineString> logger_;
    // use ordered feature map to preserve feature layer preference
    std::map<DebugFeatureType, std::vector<std::vector<util::Coordinate>>> features_;
};

} // namespace osrm::engine::route_inspection::detail

#endif /* OSRM_RI_DEBUG_LOGGER_HPP */
