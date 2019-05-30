#ifndef FILTER_CONTEXT_HPP
#define FILTER_CONTEXT_HPP

#include <math.h>
#include <string>
#include <vector>

#include "context_macros.hpp"

namespace rclcpp
{
  class Node;

  class Parameter;
}

namespace filter_node
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(sub_odom, true, bool)                            /* Subscribe to odom, assume map => base  */ \
  CXT_ELEM(sub_pose, false, bool)                           /* Subscribe to pose, assume base => sensor  */ \
  \
  CXT_ELEM(map_frame, "map", std::string)                   /* Map frame  */ \
  CXT_ELEM(base_frame, "base_link", std::string)            /* Base frame  */ \
  CXT_ELEM(sensor_frame, "camera_frame", std::string)       /* Sensor frame  */ \
  \
  CXT_ELEM(pub_odom, true, bool)                            /* Publish odom  */ \
  CXT_ELEM(pub_tf_map_base, true, bool)                     /* Publish tf map to base  */ \
  CXT_ELEM(pub_tf_map_sensor, false, bool)                  /* Publish tf map to sensor  */ \
  \
  CXT_ELEM(t_sensor_base_x, 0, double)                      /* Transform sensor to base TODO flip this?  */ \
  CXT_ELEM(t_sensor_base_y, 0, double)                      /* Transform sensor to base  */ \
  CXT_ELEM(t_sensor_base_z, 0, double)                      /* Transform sensor to base  */ \
  CXT_ELEM(t_sensor_base_roll, 0, double)                   /* Transform sensor to base  */ \
  CXT_ELEM(t_sensor_base_pitch, 0, double)                  /* Transform sensor to base  */ \
  CXT_ELEM(t_sensor_base_yaw, 0, double)                    /* Transform sensor to base  */ \
/* End of list */

  struct FilterContext
  {
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_FIELD_DEF(n, a)
    CXT_MACRO_ALL_PARAMS

    void load_parameters(rclcpp::Node &node);

    void change_parameters(rclcpp::Node &node, std::vector<rclcpp::Parameter> parameters);
  };

} // namespace filter_node

#endif // FILTER_CONTEXT_HPP
