#ifndef FILTER_NODE_H
#define FILTER_NODE_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "filter_context.hpp"
#include "kf.hpp"
#include "monotonic.hpp"

namespace filter_node {

class FilterNode: public rclcpp::Node
{
  // Parameters
  FilterContext cxt_;

  // Pose of base_frame in sensor frame
  tf2::Transform t_sensor_base_;

  // Node state
  bool mission_;
  rclcpp::Time prev_stamp_;
  kf::KalmanFilter filter_;

  // Publications
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sensor_pose_sub_;

  // Callbacks
  void sensor_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

  // Callback wrappers
  Monotonic<FilterNode *, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_cb_{this,
    &FilterNode::sensor_pose_callback};

public:

  explicit FilterNode();

  ~FilterNode() {}
};

} // namespace filter_node

#endif // FILTER_NODE_H
