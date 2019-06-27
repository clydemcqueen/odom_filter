A simple ROS2 odometry (Kalman) filter.

Supports Crystal and Dashing.

Requires Eigen3 and [ros2_shared](https://github.com/ptrmu/ros2_shared).

The Kalman filter (`kf.hpp`, `kf.cpp`) is mostly generic, but the angle normalization is hard-coded:
~~~
    // Normalize the angles in y
    y_(3, 0) = norm_angle(y_(3, 0));
    y_(4, 0) = norm_angle(y_(4, 0));
    y_(5, 0) = norm_angle(y_(5, 0));
~~~