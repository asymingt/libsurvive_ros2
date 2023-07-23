// Copyright 2022 Andrew Symington
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef LIBSURVIVE_ROS2__DRIVER_COMPONENT_HPP_
#define LIBSURVIVE_ROS2__DRIVER_COMPONENT_HPP_

// So we can access the full gamut of callbacks from libsurvive.
#define SURVIVE_ENABLE_FULL_API

// C system
#include <os_generic.h>

// C++ system
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Other
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "libsurvive/survive_api.h"
#include "libsurvive/survive.h"
#include "libsurvive_ros2/msg/angle.hpp"
#include "libsurvive_ros2/msg/lighthouse.hpp"
#include "libsurvive_ros2/msg/tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace libsurvive_ros2
{

class DriverComponent : public rclcpp::Node
{
public:
  explicit DriverComponent(const rclcpp::NodeOptions & options);
  virtual ~DriverComponent();
  rclcpp::Time get_ros_time(FLT timecode_s);
  void publish_imu(const sensor_msgs::msg::Imu & msg);
  void publish_angle(const libsurvive_ros2::msg::Angle & msg);
  void publish_lighthouse(const libsurvive_ros2::msg::Lighthouse & msg);
  void publish_tracker(const libsurvive_ros2::msg::Tracker & msg);
  void publish_button(const sensor_msgs::msg::Joy & msg);
  void publish_tf(const geometry_msgs::msg::TransformStamped & msg, bool is_static);
  void publish_lighthouse_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void publish_tracker_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void add_or_update_lighthouse(const libsurvive_ros2::msg::Lighthouse & msg);
  void add_or_update_tracker(const libsurvive_ros2::msg::Tracker & msg);
  const std::string & get_tracking_frame() const;

private:
  void timer_callback();
  void work();
  SurviveContext * ctx_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    tracker_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    lighthouse_pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr button_publisher_;
  rclcpp::Publisher<libsurvive_ros2::msg::Angle>::SharedPtr angle_publisher_;
  rclcpp::Publisher<libsurvive_ros2::msg::Lighthouse>::SharedPtr lighthouse_publisher_;
  rclcpp::Publisher<libsurvive_ros2::msg::Tracker>::SharedPtr tracker_publisher_;
  std::unordered_map<std::string, libsurvive_ros2::msg::Lighthouse> lighthouses_;
  std::unordered_map<std::string, libsurvive_ros2::msg::Tracker> trackers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_thread_;
  rclcpp::Time last_base_station_update_;
  std::string tracking_frame_;
  std::string driver_config_in_;
  std::string driver_config_out_;
  std::string driver_args_;
  std::string meta_config_;
  bool recalibrate_;
  int lighthouse_count_;
};

}  // namespace libsurvive_ros2

#endif  // LIBSURVIVE_ROS2__DRIVER_COMPONENT_HPP_
