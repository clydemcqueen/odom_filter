#ifndef FILTER_CONTEXT_HPP
#define FILTER_CONTEXT_HPP

#include <math.h>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

namespace filter_node
{

#define FILTER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(sub_odom, bool, true)                            /* Subscribe to odom, assume map => base  */ \
  CXT_MACRO_MEMBER(sub_pose, bool, false)                           /* Subscribe to pose, assume base => sensor  */ \
  \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")                   /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")            /* Base frame  */ \
  CXT_MACRO_MEMBER(sensor_frame, std::string, "camera_frame")       /* Sensor frame  */ \
  \
  CXT_MACRO_MEMBER(pub_odom, bool, true)                            /* Publish odom  */ \
  CXT_MACRO_MEMBER(pub_tf_map_base, bool, true)                     /* Publish tf map to base  */ \
  CXT_MACRO_MEMBER(pub_tf_map_sensor, bool, false)                  /* Publish tf map to sensor  */ \
  \
  CXT_MACRO_MEMBER(t_sensor_base_x, double, 0)                      /* Transform sensor to base TODO flip this?  */ \
  CXT_MACRO_MEMBER(t_sensor_base_y, double, 0)                      /* Transform sensor to base  */ \
  CXT_MACRO_MEMBER(t_sensor_base_z, double, 0)                      /* Transform sensor to base  */ \
  CXT_MACRO_MEMBER(t_sensor_base_roll, double, 0)                   /* Transform sensor to base  */ \
  CXT_MACRO_MEMBER(t_sensor_base_pitch, double, 0)                  /* Transform sensor to base  */ \
  CXT_MACRO_MEMBER(t_sensor_base_yaw, double, 0)                    /* Transform sensor to base  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct FilterContext
  {
    FILTER_NODE_ALL_PARAMS
  };

} // namespace filter_node

#endif // FILTER_CONTEXT_HPP
