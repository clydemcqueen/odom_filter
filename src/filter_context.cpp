#include "filter_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace filter_node {

void FilterContext::load_parameters(rclcpp::Node &node)
{
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_LOAD_PARAM(n, a)
  CXT_MACRO_ALL_PARAMS
}

void FilterContext::change_parameters(rclcpp::Node &node, std::vector<rclcpp::Parameter> parameters)
{
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_CHANGE_PARAM(n, a)
  for (auto parameter : parameters) {
    CXT_MACRO_ALL_PARAMS
  }
}

} // namespace orca_base
