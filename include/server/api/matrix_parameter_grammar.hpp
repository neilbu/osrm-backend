#ifndef MATRIX_PARAMETERS_GRAMMAR_HPP
#define MATRIX_PARAMETERS_GRAMMAR_HPP

#include "server/api/base_parameters_grammar.hpp"
#include "engine/api/matrix_parameters.hpp"

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace osrm
{
namespace server
{
namespace api
{

namespace
{
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
}

template <typename Iterator = std::string::iterator,
          typename Signature = void(engine::api::MatrixParameters &)>
struct MatrixParametersGrammar final : public BaseParametersGrammar<Iterator, Signature>
{
    using BaseGrammar = BaseParametersGrammar<Iterator, Signature>;

    MatrixParametersGrammar() : BaseGrammar(root_rule)
    {
#ifdef BOOST_HAS_LONG_LONG
        if (std::is_same<std::size_t, unsigned long long>::value)
            size_t_ = qi::ulong_long;
        else
            size_t_ = qi::ulong_;
#else
        size_t_ = qi::ulong_;
#endif

        root_rule = BaseGrammar::query_rule(qi::_r1) > -qi::lit(".json") >
                    -('?' > (BaseGrammar::base_rule(qi::_r1)) % '&');
    }

  private:
    qi::rule<Iterator, Signature> root_rule;
    qi::rule<Iterator, std::size_t()> size_t_;
};
}
}
}

#endif
