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

#include <cmath>

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
const double kSensorPositionSigma = 0.0001;
const double kSigmaAngle = 0.001;
const double kUseImuData = true;

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

gtsam::Pose3 gtsam_from_ros(const geometry_msgs::msg::Transform & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(
      pose.rotation.w,
      pose.rotation.x,
      pose.rotation.y,
      pose.rotation.z),
    gtsam::Point3(
      pose.translation.x,
      pose.translation.y,
      pose.translation.z));
}

geometry_msgs::msg::Pose ros_from_gtsam(const gtsam::Pose3 & pose)
{
  gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  msg.orientation.w = quaternion.w();
  msg.orientation.x = quaternion.x();
  msg.orientation.y = quaternion.y();
  msg.orientation.z = quaternion.z();
  return msg;
}

geometry_msgs::msg::Transform ros_transform_from_gtsam(const gtsam::Pose3 & pose)
{
  gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  geometry_msgs::msg::Transform msg;
  msg.translation.x = pose.translation().x();
  msg.translation.y = pose.translation().y();
  msg.translation.z = pose.translation().z();
  msg.rotation.w = quaternion.w();
  msg.rotation.x = quaternion.x();
  msg.rotation.y = quaternion.y();
  msg.rotation.z = quaternion.z();
  return msg;
}

gtsam::noiseModel::Gaussian::shared_ptr gtsam_from_ros(const std::array<double, 36> & msg)
{
  //#
  gtsam::Matrix6 cov;
  cov <<  msg[21],  msg[22],  msg[23],  msg[18],  msg[19],  msg[20],  // Rx // NOLINT
          msg[27],  msg[28],  msg[29],  msg[24],  msg[25],  msg[26],  // Ry // NOLINT
          msg[33],  msg[34],  msg[35],  msg[30],  msg[31],  msg[32],  // Rz // NOLINT
          msg[3],   msg[4],   msg[5],   msg[0],   msg[1],   msg[2],   // Tx // NOLINT
          msg[9],   msg[10],  msg[11],  msg[6],   msg[7],   msg[8],   // Ty // NOLINT
          msg[15],  msg[16],  msg[17],  msg[12],  msg[13],  msg[14];  // Tz // NOLINT
  return gtsam::noiseModel::Gaussian::Covariance(cov);
}

std::array<double, 36> ros_from_gtsam(const gtsam::Matrix & cov)
{
  return {
    cov(3, 3), cov(3, 4), cov(3, 5), cov(3, 0), cov(3, 1), cov(3, 2),  // Tx // NOLINT
    cov(4, 3), cov(4, 4), cov(4, 5), cov(4, 0), cov(4, 1), cov(4, 2),  // Ty // NOLINT
    cov(5, 3), cov(5, 4), cov(5, 5), cov(5, 0), cov(5, 1), cov(5, 2),  // Tz // NOLINT
    cov(0, 3), cov(0, 4), cov(0, 5), cov(0, 0), cov(0, 1), cov(0, 2),  // Rx // NOLINT
    cov(1, 3), cov(1, 4), cov(1, 5), cov(1, 0), cov(1, 1), cov(1, 2),  // Ry // NOLINT
    cov(2, 3), cov(2, 4), cov(2, 5), cov(2, 0), cov(2, 1), cov(2, 2),  // Rz // NOLINT
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
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
  tf_static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this))
{
  std::string meta_config;
  this->declare_parameter("meta_config", "~/.config/libsurvive/poser.yaml");
  this->get_parameter("meta_config", meta_config);
  this->declare_parameter("tracking_frame", "libsurvive_frame");
  this->get_parameter("tracking_frame", tracking_frame_);

  // Example: example_config_poser.yaml
  // -------------------
  // rigid_bodies:
  //   - body_id: "world_frame"
  //     trackers:
  //       - tracker_id: LHR-74EFB987
  //         bTh: [0.0,  0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  //   - body_id: "body_frame"
  //     trackers:
  //       - tracker_id: LHR-933F150A
  //         bTh: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  // registration:
  //   - body_id: "world_frame"
  //     gTb: [0.0, 0.0, 10.0, 1.0, 0.0, 0.0, 0.0]
  YAML::Node config = YAML::LoadFile(meta_config);
  for (auto body : config["rigid_bodies"]) {
    std::string body_id = body["body_id"].as<std::string>();

    // If this is a static body, we need to only assign it a single state
    // the global frame, and set a prior on that state
    id_to_body_info_[body_id].is_static = body["static"].as<bool>();
    if (id_to_body_info_[body_id].is_static) {

      // Pose is unknown, but it start it off somewhere reasonable.
      id_to_body_info_[body_id].gTb[0] = next_available_key_++;
      auto gTb_obs = gtsam::Pose3(
        gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
        gtsam::Point3(0.0, 0.0, 0.0));
      auto gTb_cov = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector6(100.0, 100.0, 100.0, 100.0, 100.0, 100.0));
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
          id_to_body_info_[body_id].gTb[0], gTb_obs, gTb_cov));
      initial_values_.insert(id_to_body_info_[body_id].gTb[0], gTb_obs);
    }
    
    // Add tracker information
    for (auto tracker : body["trackers"]) {
      std::string tracker_id = tracker["tracker_id"].as<std::string>();
      std::vector<double> bTh = tracker["bTh"].as<std::vector<double>>();
      id_to_body_info_[body_id].bTh[tracker_id] = gtsam::Pose3(
        gtsam::Rot3::Quaternion(bTh[3], bTh[4], bTh[5], bTh[6]),
        gtsam::Point3(bTh[0], bTh[1], bTh[2]));
    }
  }

  // Add lighthouses to avoid needing to wait for OOTX packets.
  for (auto lighthouse : config["lighthouses"]) {
    libsurvive_ros2::msg::Lighthouse msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = lighthouse["lighthouse_id"].as<std::string>();
    msg.channel = static_cast<uint8_t>(lighthouse["channel"].as<int>());
    for (size_t i = 0; i < 2; i++) {
      msg.fcalcurve[i] = lighthouse["fcalcurve"][i].as<double>();
      msg.fcalgibmag[i] = lighthouse["fcalgibmag"][i].as<double>();
      msg.fcalgibpha[i] = lighthouse["fcalgibpha"][i].as<double>();
      msg.fcalogeemag[i] = lighthouse["fcalogeemag"][i].as<double>();
      msg.fcalogeephase[i] = lighthouse["fcalogeephase"][i].as<double>();
      msg.fcalphase[i] = lighthouse["fcalphase"][i].as<double>();
      msg.fcaltilt[i] = lighthouse["fcaltilt"][i].as<double>();
    }
    this->add_lighthouse(msg);
  }

  // Add registration information
  for (auto registration : config["registration"]) {
    std::string body_id = registration["body_id"].as<std::string>();
    if (id_to_body_info_.count(body_id) == 0) {
      RCLCPP_WARN_STREAM(this->get_logger(),
        "Unknown body frame '" << body_id << "' specified in registration");
      continue;
    }
    auto & body_info = id_to_body_info_[body_id];
    if (!body_info.is_static) {
      RCLCPP_WARN_STREAM(this->get_logger(),
        "Body frame '" << body_id << "' is not marked static");
      continue;
    }
    std::vector<double> gTb = registration["gTb"].as<std::vector<double>>();
    auto gTb_obs = gtsam::Pose3(
      gtsam::Rot3::Quaternion(gTb[3], gTb[4], gTb[5], gTb[6]),
      gtsam::Point3(gTb[0], gTb[1], gTb[2]));
    auto gTb_cov = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9));
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        body_info.gTb[0], gTb_obs, gTb_cov));
    initial_values_.update(body_info.gTb[0], gTb_obs);
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

  // Setup refined body frame pose publisher
  body_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/body", qos_profile);
  tracker_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/tracker", qos_profile);
  lighthouse_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/lighthouse", qos_profile);

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
  // Only accept angle measurements from trackers already inserted into the graph.
  const std::string & tracker_id = msg.header.frame_id;
  if (id_to_tracker_info_.count(tracker_id) == 0) {
    return;
  }
  auto & tracker_info = id_to_tracker_info_[tracker_id];
  auto & body_info = id_to_body_info_[tracker_info.body_id];

  // Only accept angle measurements from channels we have already seen
  if (channel_to_lighthouse_info_.count(msg.channel) == 0) {
    return;
  }
  auto & lighthouse_info = channel_to_lighthouse_info_[msg.channel];

  // Verify that the tracker has this sensor id
  if (tracker_info.t_points.count(msg.sensor_id) == 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Tracker " << tracker_id << " sensor " << int(msg.sensor_id) << " does not exist");
  }

  // Verify that we have a plane of 0 or 1
  if (msg.plane > 1) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Tracker " << tracker_id << " plane " << int(msg.plane) << " is not in  {0, 1}");
  }
  
  // Get the current timestamp
  const Timestamp stamp_curr = rclcpp::Time(msg.header.stamp).nanoseconds();
  
  // Get the last pose timestamp, if one exists for this body.
  std::optional<Timestamp> stamp_prev = std::nullopt;
  if (body_info.gTb.size()) {
    stamp_prev = body_info.gTb.rbegin()->first;
  }
  
  std::optional<gtsam::Key> gTb;
  if (body_info.is_static) {
    // If this is a static body, then we shouldn't keep adding states. Just use the
    // static single state and treat all observations.
    gTb = body_info.gTb[0];
  } else if (kUseImuData) {
    // If the IMU is being used and there is no pose state, we have to wait for one to
    // be inserted, so it makes more sense to return at this point.
    gTb = find_key_for_closest_stamp(body_info.gTb, stamp_curr);
    if (!gTb) {
      return;
    }
  } else {
    // If the IMU is ignored, then we need to insert a state to represent the pose of
    // body at the current time. But first, let's cover the case where we've received
    // an angle measurement from this body before.
    if (body_info.gTb.count(stamp_curr) == 0) {
      body_info.gTb[stamp_curr] = next_available_key_++;
      if (!stamp_prev) {
        RCLCPP_INFO_STREAM(this->get_logger(),
          "Adding prior pose for body " << tracker_info.body_id);
        auto gTb_obs = gtsam::Pose3(
          gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
          gtsam::Point3(0.0, 0.0, 0.0));
        auto gTb_cov = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector6(100.0, 100.0, 100.0, 100.0, 100.0, 100.0));
        graph_.add(
          gtsam::PriorFactor<gtsam::Pose3>(
            body_info.gTb[stamp_curr], gTb_obs, gTb_cov));
        initial_values_.insert(body_info.gTb[stamp_curr], gTb_obs);
      } else {
        auto gTb_obs = gtsam::Pose3(
          gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
          gtsam::Point3(0.0, 0.0, 0.0));
        auto gTb_cov = gtsam::noiseModel::Isotropic::Variance(6, 1e-3);
        graph_.add(
          gtsam::BetweenFactor<gtsam::Pose3>(
            body_info.gTb[*stamp_prev],
            body_info.gTb[stamp_curr],
            gTb_obs,
            gTb_cov));
        initial_values_.insert(body_info.gTb[stamp_curr], gTb_obs);
      }
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Pose state exists for tracker " << tracker_id);
    }
    // Use the keu for the current time stamp.
    gTb = body_info.gTb[stamp_curr];
  }

  // Transform sensor to body-frame. We can maybe optimize this at some point in the
  // future to prevent having to this for every angle measurement. Not sure what sort
  // of performance increase it would give us though.
  auto & t_sensor = tracker_info.t_points[msg.sensor_id];
  auto & bTh = body_info.bTh[tracker_id];
  auto & tTh = tracker_info.tTh;
  auto b_sensor = (bTh * tTh.inverse()).transformFrom(t_sensor);

  // Observed angle and error in that angle.
  auto angle_cov = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(kSigmaAngle));
  graph_.emplace_shared<gtsam::Gen2LightAngleFactor>(
      lighthouse_info.lTg,                  // lighthouse -> global frame
      *gTb,                                 // body -> global frame
      msg.angle,                            // observed angle
      angle_cov,                            // uncertainty in our observation
      msg.plane > 0,                        // is this the y-axis?
      b_sensor,                             // sensor location in the body frame
      lighthouse_info.bcal[msg.plane]);     // lighthouse calibration parameters
}

void PoserComponent::add_imu(const sensor_msgs::msg::Imu & msg)
{
  // If ignore IMU data is requested, respect this.
  if (!kUseImuData) {
    return;
  }

  // Only accept IMU measurements from trackers already inserted into the graph.
  const std::string & tracker_id = msg.header.frame_id;
  if (!id_to_tracker_info_.count(tracker_id)) {
    return;
  }
  auto & tracker_info = id_to_tracker_info_[tracker_id];
  auto & body_info = id_to_body_info_[tracker_info.body_id];

  // There is no point in adding IMU information about a static object, because all we'd
  // be doing is spending a lot of compute on tracking the IMU random walks. Perhaps at
  // some point in the future we might want to work out the gravitation vector and use
  // it to somehow adjust the pose, but that seems a little far-fetched.
  if (body_info.is_static) {
    return;
  }
  const Timestamp stamp_curr = gtsam_from_ros(msg.header.stamp);

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

  // If bias has not been set or needs to be reset, then lets add one
  if (!tracker_info.b_B.size() ||
    (stamp_curr - tracker_info.b_B.rbegin()->first) > kImuBiasTimeNs)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Bias state event for tracker " << tracker_id);

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

    // Add a pose state, provided another IMU has not already set one for the
    // current timestamp. This is possible, but very, very unlikely.
    if (!body_info.gTb.count(stamp_curr)) {
      body_info.gTb[stamp_curr] = next_available_key_++;
      if (!stamp_prev) {
        RCLCPP_INFO_STREAM(this->get_logger(),
          "Adding prior pose for body " << tracker_info.body_id);
        auto gTb_obs = gtsam::Pose3(
          gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
          gtsam::Point3(0.0, 0.0, 0.0));
        auto gTb_cov = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector6(100.0, 100.0, 100.0, 100.0, 100.0, 100.0));
        graph_.add(
          gtsam::PriorFactor<gtsam::Pose3>(
            body_info.gTb[stamp_curr], gTb_obs, gTb_cov));
        initial_values_.insert(body_info.gTb[stamp_curr], gTb_obs);
      } else {
        auto gTb_prev = isam2_.calculateEstimate<gtsam::Pose3>(body_info.gTb[*stamp_prev]);
        initial_values_.insert(body_info.gTb[stamp_curr], gTb_prev);
      }
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Pose state exists for tracker " << tracker_id);
    }

    // Add a velocity state, provided another IMU has not already set one for the
    // current timestamp. This is possible, but very, very unlikely.
    if (!body_info.g_V.count(stamp_curr)) {
      body_info.g_V[stamp_curr] = next_available_key_++;
      if (!stamp_prev) {
        RCLCPP_INFO_STREAM(this->get_logger(),
          "Adding prior velocity for body " << tracker_info.body_id);
        auto g_V_obs = gtsam::Vector3(0.0, 0.0, 0.0);
        auto g_V_cov = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3(10.0, 10.0, 10.0));
        graph_.add(
          gtsam::PriorFactor<gtsam::Vector3>(
            body_info.g_V[stamp_curr], g_V_obs, g_V_cov));
        initial_values_.insert(body_info.g_V[stamp_curr], g_V_obs);
      } else {
        auto g_V_prev = isam2_.calculateEstimate<gtsam::Vector3>(body_info.g_V[*stamp_prev]);
        initial_values_.insert(body_info.g_V[stamp_curr], g_V_prev);
      }
    } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Velocity state exists for tracker " << tracker_id);
    }

    // Add an IMU bias state. This will always need to be added, because each IMU
    // tracks its own bias internally.
    if (!tracker_info.b_B.count(stamp_curr)) {
      tracker_info.b_B[stamp_curr] = next_available_key_++;
      if (!stamp_bias) {
        RCLCPP_INFO_STREAM(this->get_logger(),
          "Adding prior bias for tracker " << tracker_id);
        auto b_B_obs = gtsam::imuBias::ConstantBias();
        auto b_B_cov = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector6(
            kAccelDriftSigma, kAccelDriftSigma, kAccelDriftSigma,
            kOmegaDriftSigma, kOmegaDriftSigma, kOmegaDriftSigma));
        graph_.add(
          gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
            tracker_info.b_B[stamp_curr], b_B_obs, b_B_cov));
        initial_values_.insert(tracker_info.b_B[stamp_curr], b_B_obs);
      } else {
        auto b_B_prev = isam2_.calculateEstimate<gtsam::imuBias::ConstantBias>(
          tracker_info.b_B[*stamp_bias]);
        initial_values_.insert(tracker_info.b_B[stamp_curr], b_B_prev);
      }
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Bias state exists for tracker " << tracker_id);
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
      auto b_B_obs = gtsam::imuBias::ConstantBias();
      auto b_B_cov = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(
          kAccelDriftSigma, kAccelDriftSigma, kAccelDriftSigma,
          kOmegaDriftSigma, kOmegaDriftSigma, kOmegaDriftSigma));
      graph_.add(
        gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
          tracker_info.b_B[*stamp_bias],
          tracker_info.b_B[stamp_curr],
          b_B_obs,
          b_B_cov));
      auto b_B_prev = isam2_.calculateEstimate<gtsam::imuBias::ConstantBias>(
        tracker_info.b_B[*stamp_bias]);
      tracker_info.preintegrator->resetIntegrationAndSetBias(b_B_prev);
    } else {
      tracker_info.preintegrator->resetIntegration();
    }
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

  // Sensor locations and normals.
  double x = 0.0;
  double y = 0.0;
  double n = 0.0;
  for (size_t k = 0; k < msg.channels.size(); k++) {
    const uint8_t channel = msg.channels[k];
    tracker_info.t_points[channel] = gtsam_from_ros(msg.points[k]);
    tracker_info.t_normals[channel] = gtsam_from_ros(msg.normals[k]);
    switch(channel) {
    case 0: case 8: case 18:
      x += msg.points[k].x;
      y += msg.points[k].y;
      n += 1.0;
      break;
    default:
      break;
    }
  }
  x /= n;
  y /= n;

  // Transform that moves from the IMU frame to the tracking reference frame, in
  // which the sensor coordinates are expressed.
  tracker_info.tTi = gtsam_from_ros(msg.tracker_from_imu);

  // I've looked at some plots in foxglove and the "head" location reported by the
  // tracker is not nearly where it should be relative to the tracking reference
  // frame ("light"). I am going to assume the reported "head" to "light" z offset 
  // is right, but not trust the X, Y or orientation. It seems like the light frame
  // is related to the head frame by two 90 degree rotations. I'm going to construct
  // the x and y offset by averaging three equidistant sensors spaced at 120deg to
  // find the center coordinate and use that as the bolt location. This transform
  // must be hard-coded somewhere in SteamVR to make things work.
  auto ccw_90deg_about_x = gtsam::Rot3::AxisAngle(gtsam::Point3(1.0, 0.0, 0.0), M_PI_2);
  auto ccw_90deg_about_y = gtsam::Rot3::AxisAngle(gtsam::Point3(0.0, 1.0, 0.0), M_PI_2);
  tracker_info.tTh = gtsam::Pose3(ccw_90deg_about_x * ccw_90deg_about_y,
    gtsam::Point3(x, y, -0.007));

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
  const Channel lighthouse_channel = msg.channel;
  const std::string & lighthouse_id = msg.header.frame_id;
  if (channel_to_lighthouse_info_.count(lighthouse_channel)) {
    const std::string & orig_lighthouse_id = channel_to_lighthouse_info_[lighthouse_channel].id;
    if (orig_lighthouse_id != lighthouse_id) {
      RCLCPP_ERROR_STREAM(this->get_logger(), 
        "New lighthouse " << lighthouse_id << " on channel " << int(lighthouse_channel)
          << " previously received from " << orig_lighthouse_id);
    }
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), 
    "Adding lighthouse " << lighthouse_id << " on channel " << int(lighthouse_channel));
  auto & lighthouse_info = channel_to_lighthouse_info_[lighthouse_channel];

  // Set the ID immediately.
  lighthouse_info.id = lighthouse_id;

  // Copy over all calibration params for this lighthouse.  
  for (size_t i = 0; i < 2; i++) {
    lighthouse_info.bcal[i].phase = msg.fcalphase[i];
    lighthouse_info.bcal[i].tilt = msg.fcaltilt[i];
    lighthouse_info.bcal[i].curve = msg.fcalcurve[i];
    lighthouse_info.bcal[i].gibpha = msg.fcalgibpha[i];
    lighthouse_info.bcal[i].gibmag = msg.fcalgibmag[i];
    lighthouse_info.bcal[i].ogeephase = msg.fcalogeephase[i];
    lighthouse_info.bcal[i].ogeemag = msg.fcalogeemag[i];
  }

  // Weak initial estimate of the global -> lighthouse pose. Take note that this is
  // the inverse representation of what you'd expect.
  auto obs_lTg = gtsam::Pose3(
    gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0),
    gtsam::Point3(0.0, 0.0, -1.0));
  auto cov_lTg = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector6(10.0, 10.0, 10.0, 10.0, 10.0, 10.0));

  // Allocate a new variable.
  lighthouse_info.lTg = next_available_key_++;

  // Set its initial value.
  initial_values_.insert(lighthouse_info.lTg, obs_lTg);

  // Add a prior on the value``.
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(lighthouse_info.lTg, obs_lTg, cov_lTg));
}

void PoserComponent::solution_callback()
{
  // Incremental update of the graph using the latest values and factors.
  graph_.print("Graph");
  initial_values_.print("Values");
  isam2_.update(graph_, initial_values_);

  // Static transforms get the current time
  const auto now = this->get_clock()->now();

  // Print bodies and trackers
  for (const auto & [body_id, body_info] : id_to_body_info_) {
    // Package up a transform for each tracker on the body
    for (const auto& [tracker_id, bTh] : body_info.bTh) {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = now;
      transform_msg.header.frame_id = body_id;
      transform_msg.child_frame_id = tracker_id;
      transform_msg.transform = ros_transform_from_gtsam(bTh);
      tf_static_broadcaster_->sendTransform(transform_msg);
      // If we have info about this tracker, lets send some transforms
      if (id_to_tracker_info_.count(tracker_id)) {
        auto & tracker_info = id_to_tracker_info_[tracker_id];
        transform_msg.header.stamp = now;
        transform_msg.header.frame_id = tracker_id;
        transform_msg.child_frame_id = tracker_id + "/light";
        transform_msg.transform = ros_transform_from_gtsam(tracker_info.tTh.inverse());
        tf_static_broadcaster_->sendTransform(transform_msg);
        // Send imu transform -- looks like there 
        transform_msg.header.stamp = now;
        transform_msg.header.frame_id = tracker_id + "/light";
        transform_msg.child_frame_id = tracker_id + "/imu";
        transform_msg.transform = ros_transform_from_gtsam(tracker_info.tTi);
        tf_static_broadcaster_->sendTransform(transform_msg);
        // Send sensor transforms
        for (const auto& [channel_id, t_point] : tracker_info.t_points) {
          transform_msg.header.stamp = now;
          transform_msg.header.frame_id = tracker_id + "/light";
          transform_msg.child_frame_id = tracker_id + "/" + std::to_string(channel_id);
          transform_msg.transform.translation.x = t_point.x();
          transform_msg.transform.translation.y = t_point.y();
          transform_msg.transform.translation.z = t_point.z();
          transform_msg.transform.rotation.w = 1.0;
          transform_msg.transform.rotation.x = 0.0;
          transform_msg.transform.rotation.y = 0.0;
          transform_msg.transform.rotation.z = 0.0;
          tf_static_broadcaster_->sendTransform(transform_msg);
        }
      }
    } 
    // Pack up all the timesteps
    auto iter = body_info.gTb.rbegin();
    if (iter != body_info.gTb.rend()) {
      // Extract the solution
      gtsam::Pose3 gTb = isam2_.calculateEstimate<gtsam::Pose3>(iter->second);
      gtsam::Matrix cov = isam2_.marginalCovariance(iter->second);
      // Create a message containing the transform
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = ros_from_gtsam(iter->first);
      msg.header.frame_id = body_id;
      msg.pose.pose = ros_from_gtsam(gTb);
      msg.pose.covariance = ros_from_gtsam(cov);
      body_pose_publisher_->publish(msg);
      // Package up a transform for the body, which may be static.
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = msg.header.stamp;
      transform_msg.header.frame_id = tracking_frame_;
      transform_msg.child_frame_id = body_id;
      transform_msg.transform = ros_transform_from_gtsam(gTb);
      if (body_info.is_static) {
        tf_static_broadcaster_->sendTransform(transform_msg);
      } else {
        tf_broadcaster_->sendTransform(transform_msg);
      }
    }
  }

  // Print lighthouses
  for (const auto & [channel, lighthouse_info] : channel_to_lighthouse_info_) {
    // Extract the solution
    gtsam::Pose3 lTg = isam2_.calculateEstimate<gtsam::Pose3>(lighthouse_info.lTg);
    gtsam::Matrix cov = isam2_.marginalCovariance(lighthouse_info.lTg);
    // Create a message containing the transform
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = lighthouse_info.id;
    msg.pose.pose = ros_from_gtsam(lTg.inverse());
    msg.pose.covariance = ros_from_gtsam(cov);
    lighthouse_pose_publisher_->publish(msg);
    // Package up a transform
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = tracking_frame_;
    transform_msg.child_frame_id = lighthouse_info.id;
    transform_msg.transform = ros_transform_from_gtsam(lTg.inverse());
    tf_static_broadcaster_->sendTransform(transform_msg);
  }

  // Flush both the variables and factors, because they have been added
  graph_.resize(0);
  initial_values_.clear();
}

}  // namespace libsurvive_ros2

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(libsurvive_ros2::PoserComponent)
