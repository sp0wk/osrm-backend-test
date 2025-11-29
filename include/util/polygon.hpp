#ifndef OSRM_UTIL_POLYGON_HPP
#define OSRM_UTIL_POLYGON_HPP

#include "util/coordinate.hpp"

#include <boost/assert.hpp>
#include <boost/concept/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/iterator/transform_iterator.hpp>
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
        using namespace boost;
        BOOST_CONCEPT_ASSERT((ForwardRangeConcept<CoordRange>));

        auto const convert = [this](const auto c)
        {
            BOOST_ASSERT(c.IsValid());
            return toPoint(c);
        };

        auto begin = make_transform_iterator(std::cbegin(coords), convert);
        auto end = make_transform_iterator(std::cend(coords), convert);

        geometry::append(polygon, make_iterator_range(begin, end));
        geometry::correct(polygon);
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
