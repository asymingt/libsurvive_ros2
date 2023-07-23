// Copyright 2023 Andrew Symington
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

// DESCRIPTION
//
// This code was written to fill in several gaps I see in the pose estimation
// capabilities of libsurvive project. They are:
//
//   1. The MPFIT poser algorithm doesn't seem to converge cleanly when there
//      are many trackers and many base stations in an experiment. It might be
//      because the graphical model gets too big to solve in real-time.
//
//   2. A brief analysis of the posers available in the stock compile (MPFIT,
//      KalmanOnly, EPNP) suggests they either use IMU measurements or light
//      measurements, but not both. We can improve tracking by using both.
//
//   3. In large tracking volumes the body being tracked can occlude one or
//      more trackers' view of any lighthouses In this case the estimate
//      becomes wildly inaccurate. To circumvent this problem we should
//      exploit prior knowledge about system, and consider solving for the
//      pose of a rigid body, using any available raw measurement from all
//      trackers attached to this rigid body.
//
// Our tracking problem can be modeled with a factor graph. In this paradigm
// nodes represent quantities we want to estimate, and factors are constraints
// that limit the relative values that nodes can assume. The nice thing about
// this representation is that probability is handled well, and measurements
// are all weighted to produce the best estimate possible. Once of the tools
// we can use to perform this function is GTSAM, but in order to use this tool
// effectively we have to formally model the problem. Please refer to the
// README for a complete description of how this poser models the problem.


#ifndef LIBSURVIVE_ROS2__POSER_COMPONENT_HPP_
#define LIBSURVIVE_ROS2__POSER_COMPONENT_HPP_

// STL includes
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

// Project includes
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gtsam/inference/Symbol.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/PriorFactor.h"
#include "gtsam/slam/BetweenFactor.h"
#include "libsurvive_ros2/msg/angle.hpp"
#include "libsurvive_ros2/msg/lighthouse.hpp"
#include "libsurvive_ros2/msg/tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace libsurvive_ros2
{

// Nanosecond timestamp w.r.t unix epoch
typedef int64_t Timestamp;

// Encodes body information
struct BodyInfo
{
  // Sequence of poses of the body in the global frame
  std::map<Timestamp, gtsam::Key> gTb;

  // Sequence of velocities in the body frame.
  std::map<Timestamp, gtsam::Key> g_V;

  // Tracker head to body transforms for all sensors
  std::unordered_map<std::string, gtsam::Pose3> bTh;
};

// Encodes tracker information
struct TrackerInfo
{
  // Associated body -- we don't store a pointer here because there are no
  // guarantees about how a std::unordered_map organizes its memory. It's
  // cheap to look up on-demand, though.
  std::string body_id;

  // Things that are provided by the tracker, but can be estimated over time.
  gtsam::Pose3 tTi;
  gtsam::Pose3 tTh;

  // Sensor <locations, normals> in the tracking frame.
  std::unordered_map<uint8_t, std::pair<gtsam::Point3, gtsam::Point3>> t_sensors;

  // Constant scale and bias factors for the IMU, in the imu frame.
  gtsam::Vector3 accel_scale;
  gtsam::Vector3 accel_bias;
  gtsam::Vector3 omega_scale;
  gtsam::Vector3 omega_bias;

  // IMU pre-integrator
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegrator;

  // Time varying IMU accelerometer and gyroscope bias in the body frame.
  std::map<Timestamp, gtsam::Key> b_B;
};

// Encodes lighthouse information
struct LighthouseInfo
{
  // Static pose of this lighthouse
  gtsam::Key gTl;
};

// Helper to find the closest key lower that the supplied value. This is used to map
// pose corrections to their nearest IMU timestamp.
std::optional<gtsam::Key>
find_key_for_closest_stamp(std::map<Timestamp, gtsam::Key> & m, Timestamp t)
{
  std::map<Timestamp, gtsam::Key>::iterator it = m.lower_bound(t);
  it = it != m.begin() ? --it : m.end();
  if (it != m.end()) {
    return it->second;
  }
  return std::nullopt;
}

class PoserComponent : public rclcpp::Node
{
public:
  // Create a new metaposer class to track
  explicit PoserComponent(const rclcpp::NodeOptions & options);
  virtual ~PoserComponent();

private:
  // Raw measurements streaming in from the low-level driver
  void add_angle(const libsurvive_ros2::msg::Angle & msg);
  void add_tracker(const libsurvive_ros2::msg::Tracker & msg);
  void add_lighthouse(const libsurvive_ros2::msg::Lighthouse & msg);
  void add_imu(const sensor_msgs::msg::Imu & msg);

  // Estimates from the low-level driver, which we treat as priors
  void add_tracker_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void add_lighthouse_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  // Calculate and publish a solution
  void solution_callback();

  // Keep track of the next available symbol
  gtsam::Key next_available_key_;

  // Body information information
  std::unordered_map<std::string, BodyInfo> id_to_body_info_;

  // Tracker information
  std::unordered_map<std::string, TrackerInfo> id_to_tracker_info_;

  // Lighthouse information
  std::unordered_map<std::string, LighthouseInfo> id_to_lighthouse_info_;

  // This is the graphical model into which we'll store factors.
  gtsam::NonlinearFactorGraph graph_;

  // These are the initial set of values for the factors.
  gtsam::Values initial_values_;

  // This is an incremental smoothing and mapping algorithm for estimating.
  gtsam::ISAM2 isam2_;

  // Parent frame for all pose solutions.
  std::string tracking_frame_;

  // Topic subscribers.
  rclcpp::Subscription<libsurvive_ros2::msg::Tracker>::SharedPtr tracker_subscriber_;
  rclcpp::Subscription<libsurvive_ros2::msg::Lighthouse>::SharedPtr lighthouse_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<libsurvive_ros2::msg::Angle>::SharedPtr angle_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    tracker_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    lighthouse_pose_subscriber_;

  // Solution publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr body_pose_publisher_;

  // TF2 publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Solution timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace libsurvive_ros2

#endif  // LIBSURVIVE_ROS2__POSER_COMPONENT_HPP_
