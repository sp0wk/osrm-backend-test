#ifndef ROUTE_INSPECTION_PARAMETERS_GRAMMAR_HPP
#define ROUTE_INSPECTION_PARAMETERS_GRAMMAR_HPP

#include "server/api/route_parameters_grammar.hpp"
#include "engine/api/route_inspection_parameters.hpp"

#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace osrm::server::api
{

namespace
{
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
} // namespace

template <typename Iterator = std::string::iterator,
          typename Signature = void(engine::api::RouteInspectionParameters &)>
struct RouteInspectionParametersGrammar final : public RouteParametersGrammar<Iterator, Signature>
{
    using BaseGrammar = RouteParametersGrammar<Iterator, Signature>;

    RouteInspectionParametersGrammar() : BaseGrammar(root_rule)
    {
        polygon_point_rule = (double_ > qi::lit(',') >
                              double_)[qi::_val = ph::bind(
                                           [](double lon, double lat)
                                           {
                                               return util::Coordinate(
                                                   util::toFixed(util::UnsafeFloatLongitude{lon}),
                                                   util::toFixed(util::UnsafeFloatLatitude{lat}));
                                           },
                                           qi::_1,
                                           qi::_2)];

        polygon_rule =
            qi::lit("polygon=") >
            (polygon_point_rule %
             ';')[ph::bind(&engine::api::RouteInspectionParameters::polygon, qi::_r1) = qi::_1];

        root_rule = BaseGrammar::query_rule(qi::_r1) > BaseGrammar::format_rule(qi::_r1) >
                    -('?' > (polygon_rule(qi::_r1) | BaseGrammar::base_rule(qi::_r1)) % '&');
    }

  private:
    using BaseGrammar::double_;

    qi::rule<Iterator, osrm::util::Coordinate()> polygon_point_rule;
    qi::rule<Iterator, Signature> polygon_rule;
    qi::rule<Iterator, Signature> root_rule;
};
} // namespace osrm::server::api

#endif
