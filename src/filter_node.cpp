#include "filter_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//==================================================================
// Estimate the current pose and velocity using a Kalman filter
//
// Model pose, velocity and acceleration (18 variables)
// Measure pose (6 variables)
//
// Transformation notation:
//    t_destination_source is a transform
//    vector_destination = t_destination_source * vector_source
//    xxx_f_destination means xxx is expressed in destination frame
//    xxx_pose_f_destination is equivalent to t_destination_xxx
//    t_a_c = t_a_b * t_b_c
//==================================================================

namespace filter_node {

constexpr int STATE_DIM = 18;
constexpr int MEASUREMENT_DIM = 6;

//==================================================================
// Utility functions
//==================================================================

// Create measurement matrix
void to_z(const tf2::Transform &in, Eigen::MatrixXd &out)
{
  const tf2::Vector3 &z_p = in.getOrigin();
  const tf2::Matrix3x3 &z_r = in.getBasis();

  tf2Scalar roll, pitch, yaw;
  z_r.getRPY(roll, pitch, yaw);

  out = Eigen::MatrixXd(MEASUREMENT_DIM, 1);
  out << z_p.x(), z_p.y(), z_p.z(), roll, pitch, yaw;
}

// Create measurement covariance matrix
void to_R(const std::array<double, 36> &in, Eigen::MatrixXd &out)
{
  out = Eigen::MatrixXd(MEASUREMENT_DIM, MEASUREMENT_DIM);
  for (int i = 0; i < MEASUREMENT_DIM; i++) {
    for (int j = 0; j < MEASUREMENT_DIM; j++) {
      out(i, j) = in[i * MEASUREMENT_DIM + j];
    }
  }
}

// Extract pose from state
void pose_from_x(const Eigen::MatrixXd &in, geometry_msgs::msg::Pose &out)
{
  out.position.x = in(0, 0);
  out.position.y = in(1, 0);
  out.position.z = in(2, 0);

  tf2::Matrix3x3 m;
  m.setRPY(in(3, 0), in(4, 0), in(5, 0));

  tf2::Quaternion q;
  m.getRotation(q);

  out.orientation = tf2::toMsg(q);
}

// Extract pose from state
void pose_from_x(const Eigen::MatrixXd &in, tf2::Transform &out)
{
  out.setOrigin(tf2::Vector3(in(0, 0), in(1, 0), in(2, 0)));

  tf2::Matrix3x3 m;
  m.setRPY(in(3, 0), in(4, 0), in(5, 0));
  out.setBasis(m);
}

// Extract velocity from state
void twist_from_x(const Eigen::MatrixXd &in, geometry_msgs::msg::Twist &out)
{
  out.linear.x = in(6, 0);
  out.linear.y = in(7, 0);
  out.linear.z = in(8, 0);

  out.angular.x = in(9, 0);
  out.angular.y = in(10, 0);
  out.angular.z = in(11, 0);
}

// Extract pose covariance
void pose_covar_from_P(const Eigen::MatrixXd &in, std::array<double, 36> &out)
{
  for (int i = 0; i < MEASUREMENT_DIM; i++) {
    for (int j = 0; j < MEASUREMENT_DIM; j++) {
      out[i * MEASUREMENT_DIM + j] = in(i, j);  // [0:6, 0:6]
    }
  }
}

// Extract velocity covariance
void twist_covar_from_P(const Eigen::MatrixXd &in, std::array<double, 36> &out)
{
  for (int i = 0; i < MEASUREMENT_DIM; i++) {
    for (int j = 0; j < MEASUREMENT_DIM; j++) {
      out[i * MEASUREMENT_DIM + j] = in(i + MEASUREMENT_DIM, j + MEASUREMENT_DIM);  // [6:12, 6:12]
    }
  }
}

//==================================================================
// Class FilterNode
//==================================================================

FilterNode::FilterNode():
  Node{"filter_node"},
  filter_{STATE_DIM, MEASUREMENT_DIM}
{
  cxt_.load_parameters(*this);

  // Get t_sensor_base from parameters
  if (cxt_.sub_pose_ || cxt_.pub_tf_map_sensor_) {
    tf2::Vector3 t{cxt_.t_sensor_base_x_, cxt_.t_sensor_base_y_, cxt_.t_sensor_base_z_};
    tf2::Quaternion q;
    q.setEuler(cxt_.t_sensor_base_yaw_, cxt_.t_sensor_base_pitch_, cxt_.t_sensor_base_roll_);
    t_sensor_base_ = tf2::Transform(q, t);
    t_base_sensor_ = t_sensor_base_.inverse();
  }

  // Measurement function
  Eigen::MatrixXd H(MEASUREMENT_DIM, STATE_DIM);
  H <<
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  filter_.set_H(H);

  // Process noise
  filter_.set_Q(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.0001);

  // Publications
  if (cxt_.pub_odom_) {
    filtered_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 1);
  }
  if (cxt_.pub_tf_map_base_ || cxt_.pub_tf_map_sensor_) {
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
  }

  // Subscriptions
  if (cxt_.sub_odom_) {
    base_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom",
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void { this->odom_cb_.call(msg); });
  }
  if (cxt_.sub_pose_) {
    sensor_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("sensor_pose",
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void { this->pose_cb_.call(msg); });
  }

  RCLCPP_INFO(get_logger(), "filter_node ready");
}

void FilterNode::base_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, bool first)
{
  if (!first) {
    // Robot pose
    tf2::Transform t_map_base;
    tf2::fromMsg(msg->pose.pose, t_map_base);

    // Measurement and covariance matrices
    Eigen::MatrixXd z, R;
    to_z(t_map_base, z);
    to_R(msg->pose.covariance, R);

    // Process measurement
    process(odom_cb_.curr(), odom_cb_.dt(), z, R);
  }
}

// Process raw camera pose and publish estimated drone pose
void FilterNode::sensor_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
{
  if (!first) {
    // Sensor pose
    tf2::Transform t_map_sensor;
    tf2::fromMsg(msg->pose.pose, t_map_sensor);

    // Robot pose
    tf2::Transform t_map_base = t_map_sensor * t_sensor_base_;

    // Measurement and covariance matrices
    Eigen::MatrixXd z, R;
    to_z(t_map_base, z);
    to_R(msg->pose.covariance, R);

    // Process measurement
    process(pose_cb_.curr(), pose_cb_.dt(), z, R);
  }
}

void FilterNode::process(const rclcpp::Time &stamp, const double dt, const Eigen::MatrixXd &z, const Eigen::MatrixXd &R)
{
  // Transfer function
  auto dt2 = 0.5 * dt * dt;
  Eigen::MatrixXd F(STATE_DIM, STATE_DIM);
  F <<
    1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, dt2, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, dt2, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, dt2, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, dt2, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, dt2, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, dt2,

    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt,

    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
  filter_.set_F(F);

  // Predict step
  filter_.predict();

  // Update step
  filter_.update(z, R);

  // Publish odometry with pose and twist
  if (cxt_.pub_odom_ && count_subscribers(filtered_odom_pub_->get_topic_name()) > 0) {
    nav_msgs::msg::Odometry msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = cxt_.map_frame_;
    msg.child_frame_id = cxt_.base_frame_;

    pose_from_x(filter_.x(), msg.pose.pose);
    pose_covar_from_P(filter_.P(), msg.pose.covariance);
    twist_from_x(filter_.x(), msg.twist.twist);
    twist_covar_from_P(filter_.P(), msg.twist.covariance);

    filtered_odom_pub_->publish(msg);
  }

  // Publish tf
  if ((cxt_.pub_tf_map_base_ || cxt_.pub_tf_map_sensor_) && count_subscribers(tf_pub_->get_topic_name()) > 0) {
    tf2_msgs::msg::TFMessage tf_msg;

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = cxt_.map_frame_;

    tf2::Transform t_map_base;
    pose_from_x(filter_.x(), t_map_base);

    if (cxt_.pub_tf_map_base_) {
      msg.child_frame_id = cxt_.base_frame_;
      msg.transform = tf2::toMsg(t_map_base);
      tf_msg.transforms.emplace_back(msg);
    }
    if (cxt_.pub_tf_map_sensor_) {
      msg.child_frame_id = cxt_.sensor_frame_;
      msg.transform = tf2::toMsg(t_map_base * t_base_sensor_);
      tf_msg.transforms.emplace_back(msg);
    }

    tf_pub_->publish(tf_msg);
  }
}

} // namespace filter_node

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<filter_node::FilterNode>();

  // Spin node
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}