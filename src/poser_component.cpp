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

// This is included as a reminder of the frame names.
//
//   g : the global frame
//   l : the lighthouse frame
//   s : the tracker "sensor" frame (the origin about which sensors are located)
//   h : the tracker "head" frame (the bolt on the back of the tracker)
//   i : the tracker "imu" frame (the IMU origin)
//   b : the origin of a rigid body to which a tracker is attached

#include "libsurvive_ros2/poser_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace libsurvive_ros2
{

// IMU integration uncertainties
const double kGravity = 9.81;
const double kDeltaT = 0.004;      // 4ms or 250Hz
const double kAccelSigma = 0.1;
const double kAccelDriftSigma = 0.1;
const double kOmegaSigma = 0.1;
const double kOmegaDriftSigma = 0.1;
const double kIntegrationSigma = 0.1;

// Time between bias states
const Timestamp kImuBiasTimeNs = 1e9;

// Helper functions

Timestamp gtsam_from_ros(const builtin_interfaces::msg::Time & msg)
{
  return rclcpp::Time(msg).nanoseconds();
}

builtin_interfaces::msg::Time ros_from_gtsam(Timestamp stamp)
{
  return rclcpp::Time(stamp);
}

gtsam::Pose3 gtsam_from_ros(const geometry_msgs::msg::Pose & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z),
    gtsam::Point3(
      pose.position.x,
      pose.position.y,
      pose.position.z));
}

geometry_msgs::msg::Pose ros_from_gtsam(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  msg.orientation.w = quaternion.w();
  msg.orientation.x = quaternion.x();
  msg.orientation.y = quaternion.y();
  msg.orientation.z = quaternion.z();
  return msg;
}

gtsam::noiseModel::Gaussian::shared_ptr gtsam_from_ros(const std::array<double, 36> & msg)
{
  gtsam::Matrix6 cov;
  cov << msg[21], msg[22], msg[23], msg[18], msg[19], msg[20],  // Rx
    msg[27], msg[28], msg[29], msg[24], msg[25], msg[26],       // Ry
    msg[33], msg[34], msg[35], msg[30], msg[31], msg[32],       // Rz
    msg[3], msg[4], msg[5], msg[0], msg[1], msg[2],             // Tx
    msg[9], msg[10], msg[11], msg[6], msg[7], msg[8],           // Ty
    msg[15], msg[16], msg[17], msg[12], msg[13], msg[14];       // Tz
  return gtsam::noiseModel::Gaussian::Covariance(cov);
}

std::array<double, 36> ros_from_gtsam(const gtsam::Matrix & cov)
{
  return {
    cov(3, 3), cov(3, 4), cov(3, 5), cov(3, 0), cov(3, 1), cov(3, 2),  // Tx
    cov(4, 3), cov(4, 4), cov(4, 5), cov(4, 0), cov(4, 1), cov(4, 2),  // Ty
    cov(5, 3), cov(5, 4), cov(5, 5), cov(5, 0), cov(5, 1), cov(5, 2),  // Tz
    cov(0, 3), cov(0, 4), cov(0, 5), cov(0, 0), cov(0, 1), cov(0, 2),  // Rx
    cov(1, 3), cov(1, 4), cov(1, 5), cov(1, 0), cov(1, 1), cov(1, 2),  // Ry
    cov(2, 3), cov(2, 4), cov(2, 5), cov(2, 0), cov(2, 1), cov(2, 2),  // Rz
  };
}

gtsam::Point3 gtsam_from_ros(const geometry_msgs::msg::Vector3 & msg)
{
  return gtsam::Point3(msg.x, msg.y, msg.z);
}

geometry_msgs::msg::Vector3 ros_from_gtsam(const gtsam::Point3 & point3)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = point3.x();
  msg.y = point3.y();
  msg.z = point3.z();
  return msg;
}

PoserComponent::PoserComponent(const rclcpp::NodeOptions & options)
: Node("libsurvive_ros2_poser_node", options),
  next_available_key_(0),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  std::string meta_config;
  this->declare_parameter("meta_config", "~/.config/libsurvive/poser.yaml");
  this->get_parameter("meta_config", meta_config);
  this->declare_parameter("tracking_frame", "libsurvive_frame");
  this->get_parameter("tracking_frame", tracking_frame_);

  // Example: poser.yaml
  // -------------------
  // rigid_bodies:
  //   - body_id: "rigid_body/wand"
  //     trackers:
  //       - tracker_id: LHR-74EFB987
  //         bTh: [0.0,  0.15, 0.0, 1.0, 0.0, 0.0, 0.0]
  //       - tracker_id: LHR-933F150A
  //         bTh: [0.0, -0.15, 0.0, 1.0, 0.0, 0.0, 0.0]
  YAML::Node config = YAML::LoadFile(meta_config);
  for (auto body : config["rigid_bodies"]) {
    std::string body_id = body["body_id"].as<std::string>();
    for (auto tracker : body["trackers"]) {
      std::string tracker_id = tracker["tracker_id"].as<std::string>();
      std::vector<double> bTh = tracker["bTh"].as<std::vector<double>>();
      id_to_body_info_[body_id].bTh[tracker_id] = gtsam::Pose3(
        gtsam::Rot3::Quaternion(bTh[3], bTh[4], bTh[5], bTh[6]),
        gtsam::Point3(bTh[0], bTh[1], bTh[2]));
    }
  }

  // TODO(asymingt) this might not play very well with composable node inter-process
  // communication. So, we might want to avoid it until everything else is ready.
  rclcpp::QoS qos_profile(10);
  // qos_profile.durability_volatile();

  // Setup low-level driver message subscribers.
  tracker_subscriber_ = this->create_subscription<libsurvive_ros2::msg::Tracker>(
    "tracker", qos_profile, std::bind(&PoserComponent::add_tracker, this, _1));
  lighthouse_subscriber_ = this->create_subscription<libsurvive_ros2::msg::Lighthouse>(
    "lighthouse", qos_profile, std::bind(&PoserComponent::add_lighthouse, this, _1));
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", qos_profile, std::bind(&PoserComponent::add_imu, this, _1));
  angle_subscriber_ = this->create_subscription<libsurvive_ros2::msg::Angle>(
    "angle", qos_profile, std::bind(&PoserComponent::add_angle, this, _1));
  tracker_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose/tracker", qos_profile, std::bind(&PoserComponent::add_tracker_pose, this, _1));
  lighthouse_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose/lighthouse", qos_profile, std::bind(&PoserComponent::add_lighthouse_pose, this, _1));

  // Setup refined body frame pose publisher
  body_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/body", qos_profile);

  // Add a timer to extract solution and publish pose
  timer_ = this->create_wall_timer(
    100ms, std::bind(&PoserComponent::solution_callback, this));
}

PoserComponent::~PoserComponent()
{
  RCLCPP_INFO(this->get_logger(), "Destructor called");
}

void PoserComponent::add_angle(const libsurvive_ros2::msg::Angle & msg)
{
  /* TODO fix this -- might need a CustomFactor?

  // Only accept angle measurements from trackers already inserted into the graph.
  const std::string & tracker_id = msg.header.frame_id;
  if (id_to_tracker_info_.count(tracker_id) == 0) {
    return;
  }
  const Timestamp stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
  auto & tracker_info = id_to_tracker_info_[tracker_id];
  auto & body_info = id_to_body_info_[tracker_info.body_id];

  // Find the closest time lower than the given stamp in the trajectory.
  auto gTb_iter = body_info.gTb.lower_bound(stamp);
  if (gTb == body_info.gTb.end()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "No matching timestamp found");
    return;
  }

  // Add a factor constraining the adjacent poses and velocities
  graph_.add(gtsam::BearingFactor<Pose3, Point3>(
    body_info.gTb[tracker_info.last_imu_stamp],          // last pose
    body_info.g_V[tracker_info.last_imu_stamp],          // last velocity
    tracker_info.preintegrator));                        // preintegrator

  */
}

void PoserComponent::add_imu(const sensor_msgs::msg::Imu & msg)
{
  // Only accept IMU measurements from trackers already inserted into the graph.
  const std::string & tracker_id = msg.header.frame_id;
  if (!id_to_tracker_info_.count(tracker_id)) {
    return;
  }
  const Timestamp stamp_curr = gtsam_from_ros(msg.header.stamp);
  auto & tracker_info = id_to_tracker_info_[tracker_id];
  auto & body_info = id_to_body_info_[tracker_info.body_id];

  // Extract the linear acceleration and correct for constant bias / scale errors.
  gtsam::Vector i_accel(3);
  i_accel <<
    tracker_info.accel_scale[0] * msg.linear_acceleration.x + tracker_info.accel_bias[0],
    tracker_info.accel_scale[1] * msg.linear_acceleration.y + tracker_info.accel_bias[1],
    tracker_info.accel_scale[2] * msg.linear_acceleration.z + tracker_info.accel_bias[2];

  // Now normalize to gravitational units
  i_accel *= kGravity;

  // Extract the angular velocity and correct for constant bias / scale errors.
  gtsam::Vector i_omega(3);
  i_omega <<
    tracker_info.omega_scale[0] * msg.angular_velocity.x + tracker_info.omega_bias[0],
    tracker_info.omega_scale[1] * msg.angular_velocity.y + tracker_info.omega_bias[1],
    tracker_info.omega_scale[2] * msg.angular_velocity.z + tracker_info.omega_bias[2];

  // Add the pre-integrated IMU measurement -- take note that the IMU is aware of
  // the bTi fixed transform and will correct for it as part of the model.
  tracker_info.preintegrator->integrateMeasurement(i_accel, i_omega, kDeltaT);

  // If bias hs not been set or needs to be reset, then lets add one
  if (!tracker_info.b_B.size() ||
    (stamp_curr - tracker_info.b_B.rbegin()->first) > kImuBiasTimeNs)
  {
    // Add a pose state, provided another IMU has not already set one for the
    // current timestamp. This is possible, but very, very unlikely.
    if (!body_info.gTb.count(stamp_curr)) {
      body_info.gTb[stamp_curr] = next_available_key_++;
      auto gTb_obs = gtsam::Pose3(
        gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
        gtsam::Point3(0.0, 0.0, 0.0));
      auto gTb_cov = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(100.0, 100.0, 100.0, 100.0, 100.0, 100.0));
      graph_.add(
        gtsam::PriorFactor<gtsam::Pose3>(
          body_info.gTb[stamp_curr], gTb_obs, gTb_cov));
      initial_values_.insert(body_info.gTb[stamp_curr], gTb_obs);
    }

    // Add a velocity state, provided another IMU has not already set one for the
    // current timestamp. This is possible, but very, very unlikely.
    if (!body_info.g_V.count(stamp_curr)) {
      body_info.g_V[stamp_curr] = next_available_key_++;
      auto g_V_obs = gtsam::Vector3(0.0, 0.0, 0.0);
      auto g_V_cov = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(10.0, 10.0, 10.0));
      graph_.add(
        gtsam::PriorFactor<gtsam::Vector3>(
          body_info.g_V[stamp_curr], g_V_obs, g_V_cov));
      initial_values_.insert(body_info.g_V[stamp_curr], g_V_obs);
    }

    // Add an IMU bias state. This will always need to be added, because each IMU
    // tracks its own bias internally.
    tracker_info.b_B[stamp_curr] = next_available_key_++;
    auto b_B_obs = gtsam::imuBias::ConstantBias();
    auto b_B_cov = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector6(
        kAccelDriftSigma, kAccelDriftSigma, kAccelDriftSigma,
        kOmegaDriftSigma, kOmegaDriftSigma, kOmegaDriftSigma));
    graph_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        tracker_info.b_B[stamp_curr], b_B_obs, b_B_cov));
    initial_values_.insert(tracker_info.b_B[stamp_curr], b_B_obs);

    // Get the last pose timestamp, if one exists for this body.
    std::optional<Timestamp> stamp_prev = std::nullopt;
    if (body_info.gTb.size()) {
      stamp_prev = body_info.gTb.rbegin()->first;
    }

    // Get the last bias timestamp, if one exists for this IMU.
    std::optional<Timestamp> stamp_bias = std::nullopt;
    if (tracker_info.b_B.size()) {
      stamp_bias = tracker_info.b_B.rbegin()->first;
    }

    // Only add between factors for IMU measurements
    if (stamp_prev && stamp_bias) {
      graph_.add(
        gtsam::ImuFactor(
          body_info.gTb[*stamp_prev],        // last pose
          body_info.g_V[*stamp_prev],        // last velocity
          body_info.gTb[stamp_curr],         // last pose
          body_info.g_V[stamp_curr],         // last velocity
          tracker_info.b_B[*stamp_bias],     // bias state
          *tracker_info.preintegrator));     // preintegrator
      graph_.add(
        gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
          tracker_info.b_B[*stamp_bias],
          tracker_info.b_B[stamp_curr],
          b_B_obs,
          b_B_cov));
    }

    // Reset the IMU integrator to reflect that we have a new preintgrattion period.
    tracker_info.preintegrator->resetIntegration();
  }
}

void PoserComponent::add_tracker(const libsurvive_ros2::msg::Tracker & msg)
{
  const std::string & tracker_id = msg.header.frame_id;

  // Don't re-add a tracker.
  if (id_to_tracker_info_.count(tracker_id)) {
    return;
  }

  // Find the extrinsics for this tracker
  std::optional<std::string> body_id;
  for (const auto & [id, info] : id_to_body_info_) {
    if (info.bTh.count(tracker_id)) {
      body_id = id;
    }
  }
  if (!body_id) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "No extrinsics for " << tracker_id);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Adding tracker " << tracker_id);

  // If we get here, then we can add this tracker because it has a body
  auto & tracker_info = id_to_tracker_info_[tracker_id];
  tracker_info.body_id = *body_id;

  // Get a reference to the body info
  auto & body_info = id_to_body_info_[tracker_info.body_id];

  // Fixed imu bias and scale terms.
  tracker_info.accel_scale = gtsam_from_ros(msg.accel_scale);
  tracker_info.accel_bias = gtsam_from_ros(msg.accel_bias);
  tracker_info.omega_scale = gtsam_from_ros(msg.omega_scale);
  tracker_info.omega_bias = gtsam_from_ros(msg.omega_bias);

  // Fixed transforms.
  tracker_info.tTi = gtsam_from_ros(msg.tracker_from_imu);
  tracker_info.tTh = gtsam_from_ros(msg.tracker_from_head);

  // Sensor locations and normals.
  for (size_t k = 0; k < msg.channels.size(); k++) {
    uint8_t channel = msg.channels[k];
    tracker_info.t_sensors[channel] = std::make_pair(
      gtsam_from_ros(msg.points[k]), gtsam_from_ros(msg.normals[k]));
  }

  // The IMU factor needs to know the pose of the IMU sensor in the body frame in
  // order to correct for centripetal and coriolis terms. Calculate this now.
  const auto & tTi = tracker_info.tTi;
  const auto & hTt = tracker_info.tTh.inverse();
  const auto & bTh = body_info.bTh[msg.header.frame_id];

  // Set up IMU pre-integrator for a Z-up world frame (eg. MakeSharedU).
  auto params = gtsam::PreintegrationParams::MakeSharedU(kGravity);
  params->setBodyPSensor(bTh * hTt * tTi);
  params->setAccelerometerCovariance(gtsam::I_3x3 * (kAccelSigma * kAccelSigma));
  params->setGyroscopeCovariance(gtsam::I_3x3 * (kOmegaSigma * kOmegaSigma));
  params->setIntegrationCovariance(gtsam::I_3x3 * (kIntegrationSigma * kIntegrationSigma));
  params->setUse2ndOrderCoriolis(false);
  params->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
  tracker_info.preintegrator =
    std::make_shared<gtsam::PreintegratedImuMeasurements>(params);
}

void PoserComponent::add_lighthouse(const libsurvive_ros2::msg::Lighthouse & msg)
{
  const std::string & lighthouse_id = msg.header.frame_id;
  if (id_to_lighthouse_info_.count(lighthouse_id)) {
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Adding lighthouse " << lighthouse_id);

  auto & lighthouse_info = id_to_lighthouse_info_[lighthouse_id];

  // This is out initial prior on bae station location given no observations.
  auto obs_gTl = gtsam::Pose3(
    gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
    gtsam::Point3(0.0, 0.0, 0.0));
  auto cov_gTl = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector6(100.0, 100.0, 100.0, 100.0, 100.0, 100.0));

  // Allocate a new variable.
  lighthouse_info.gTl = next_available_key_++;

  // Set its initial value.
  initial_values_.insert(lighthouse_info.gTl, obs_gTl);

  // Add a prior on the value.
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(lighthouse_info.gTl, obs_gTl, cov_gTl));
}

void PoserComponent::add_tracker_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const std::string & tracker_id = msg.header.frame_id;
  if (!id_to_tracker_info_.count(tracker_id)) {
    return;
  }
  const Timestamp stamp = gtsam_from_ros(msg.header.stamp);
  auto & tracker_info = this->id_to_tracker_info_[tracker_id];
  auto & body_info = this->id_to_body_info_[tracker_info.body_id];

  // We can't add an observation for a state that does not exist.
  auto gTb = find_key_for_closest_stamp(body_info.gTb, stamp);
  if (!gTb) {
    return;
  }

  // Add a prior factor on the pose state
  auto gTt_obs = gtsam_from_ros(msg.pose.pose);
  auto hTb = body_info.bTh[tracker_id].inverse();
  auto tTb = tracker_info.tTh * hTb;
  auto gTb_obs = gTt_obs * tTb;
  auto gTt_cov = gtsam_from_ros(msg.pose.covariance);
  auto gTb_cov = gTt_cov;
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(*gTb, gTb_obs, gTb_cov));
}

void PoserComponent::add_lighthouse_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const std::string & lighthouse_id = msg.header.frame_id;
  if (!id_to_lighthouse_info_.count(lighthouse_id)) {
    return;
  }
  auto & lighthouse_info = this->id_to_lighthouse_info_[lighthouse_id];

  // Collect observation
  auto obs_gTl = gtsam_from_ros(msg.pose.pose);
  auto cov_gTl = gtsam_from_ros(msg.pose.covariance);

  // Add the observation
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(lighthouse_info.gTl, obs_gTl, cov_gTl));
}

void PoserComponent::solution_callback()
{
  // Incremental update of the graph usng the latest values and factors.
  isam2_.update(graph_, initial_values_);

  // Print solution
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  for (const auto & [body_id, body_info] : id_to_body_info_) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Broadcasting body " << body_id);
    auto iter = body_info.gTb.rbegin();
    if (iter != body_info.gTb.rend()) {
      // Extract the solution
      gtsam::Pose3 pose = isam2_.calculateEstimate<gtsam::Pose3>(iter->second);
      gtsam::Matrix cov = isam2_.marginalCovariance(iter->second);

      // Create a message containing the transform
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = ros_from_gtsam(iter->first);
      msg.header.frame_id = body_id;
      msg.pose.pose = ros_from_gtsam(pose);
      msg.pose.covariance = ros_from_gtsam(cov);
      body_pose_publisher_->publish(msg);

      // Package up a transform
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = ros_from_gtsam(iter->first);
      transform_msg.header.frame_id = tracking_frame_;
      transform_msg.child_frame_id = body_id;
      transform_msg.transform.translation.x = msg.pose.pose.position.x;
      transform_msg.transform.translation.y = msg.pose.pose.position.y;
      transform_msg.transform.translation.z = msg.pose.pose.position.z;
      transform_msg.transform.rotation = msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform_msg);
    }
  }

  // Flush both the variables and factors, because they have been added
  graph_.resize(0);
  initial_values_.clear();
}

}  // namespace libsurvive_ros2

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(libsurvive_ros2::PoserComponent)
