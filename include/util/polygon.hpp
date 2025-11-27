#ifndef OSRM_UTIL_POLYGON_HPP
#define OSRM_UTIL_POLYGON_HPP

#include "util/coordinate.hpp"

#include <boost/assert.hpp>
#include <boost/concept/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/range/concepts.hpp>

namespace osrm::util
{

/**
 * \brief Represents a simple polygon without inner rings
 */
struct Polygon
{
    using Coord = util::Coordinate;

    Polygon() = default;

    template <typename CoordRange> explicit Polygon(const CoordRange &coords)
    {
        BOOST_CONCEPT_ASSERT((boost::ForwardRangeConcept<CoordRange>));
        for (const auto c : coords)
        {
            Append(c);
        }
    }

    // Appends new point to a polygon
    void Append(const Coord c)
    {
        BOOST_ASSERT(c.IsValid());
        boost::geometry::append(polygon, toPoint(c));
    }

    // Checks whether list of points resulted in a valid polygon
    bool IsValid() const { return boost::geometry::is_valid(polygon); }

    // Checks whether coordinate is inside polygon (edges are counted as "outside")
    bool Contains(const Coord c) const noexcept
    {
        return boost::geometry::within(toPoint(c), polygon);
    }

    size_t size() const noexcept { return boost::geometry::num_points(polygon); }
    bool empty() const noexcept { return size() == 0; }

  private:
    using CoordUnderlyingType = decltype(Coord::lon)::value_type;
    using PointImpl = boost::geometry::model::d2::point_xy<CoordUnderlyingType>;
    using PolygonImpl = boost::geometry::model::polygon<PointImpl>;

    PointImpl toPoint(const Coord c) const noexcept
    {
        return PointImpl{c.lon.__value, c.lat.__value};
    }

    PolygonImpl polygon;
};

} // namespace osrm::util

#endif
