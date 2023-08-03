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

// C++ system
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// This project
#include "libsurvive_ros2/driver_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

// Hard-coded variances from libsurvive priors
const double kPosSigma_meters = 0.1;
const double kRotSigma_radians = 0.1;

// We can only ever load one version of the driver, so we store a pointer to the instance of the
// driver here, so the IMU callback can push data to it.
libsurvive_ros2::DriverComponent * _singleton = nullptr;

// Convert a libsurvive point to a geometry::msg::Vector3
static void ros_from_point(
  geometry_msgs::msg::Vector3 * const ros, const FLT * const arr)
{
  ros->x = arr[0];
  ros->y = arr[1];
  ros->z = arr[2];
}

// Convert a libsurvive pose to a geometry::msg::Transform
static void ros_from_pose(
  geometry_msgs::msg::Transform * const tx, const SurvivePose & pose)
{
  tx->translation.x = pose.Pos[0];
  tx->translation.y = pose.Pos[1];
  tx->translation.z = pose.Pos[2];
  tx->rotation.w = pose.Rot[0];
  tx->rotation.x = pose.Rot[1];
  tx->rotation.y = pose.Rot[2];
  tx->rotation.z = pose.Rot[3];
}

// Callback for inertial event
static void imu_func(
  SurviveObject * so, int mode, const FLT * accelgyromag, uint32_t rawtime, int id)
{
  survive_default_imu_process(so, mode, accelgyromag, rawtime, id);
  if (_singleton) {
    // Get precise time
    auto long_timecode = SurviveSensorActivations_long_timecode_imu(&so->activations, rawtime);
    FLT timecode = SurviveSensorActivations_runtime(&so->activations, long_timecode) / FLT(1e6);
    // Package up an inertial message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = _singleton->get_ros_time(timecode);
    imu_msg.header.frame_id = std::string(so->serial_number);
    ros_from_point(&imu_msg.linear_acceleration, &accelgyromag[0]);
    ros_from_point(&imu_msg.angular_velocity, &accelgyromag[3]);
    _singleton->publish_imu(imu_msg);
  }
}

// Callback for button event
static void button_func(
  SurviveObject * so,
  enum SurviveInputEvent eventType,
  enum SurviveButton buttonId,
  const enum SurviveAxis * axisIds,
  const SurviveAxisVal_t * axisVals)
{
  survive_default_button_process(so, eventType, buttonId, axisIds, axisVals);
  if (_singleton) {
    // Get approximate time
    FLT timecode = survive_run_time_since_epoch(so->ctx);
    // Package up a button message
    sensor_msgs::msg::Joy button_msg;
    button_msg.header.stamp = _singleton->get_ros_time(timecode);
    button_msg.header.frame_id = so->serial_number;
    button_msg.buttons.resize(SURVIVE_BUTTON_MAX * 2);
    for (int i = 0; i < SURVIVE_BUTTON_MAX; i++) {
      button_msg.buttons[i] = (buttonId == i ? eventType : 0);
    }
    button_msg.axes.resize(SURVIVE_MAX_AXIS_COUNT);
    for (int i = 0; i < SURVIVE_MAX_AXIS_COUNT; i++) {
      button_msg.axes[axisIds[i]] = axisVals[i];
    }
    _singleton->publish_button(button_msg);
  }
}

// Callback for lighthouse event
static void ootx_received_func(
  struct SurviveContext * ctx, uint8_t idx)
{
  survive_default_ootx_received_process(ctx, idx);
  if (_singleton) {
    printf("IDX: index:%d == channel:%d\n", idx, ctx->bsd[idx].mode);
    // Get serial number of lighthouse.
    char serial[16];
    snprintf(serial, sizeof(serial), "LHB-%X", (unsigned)ctx->bsd[idx].BaseStationID);
    // Get approximate time
    FLT timecode = survive_run_time_since_epoch(ctx);
    // Package up a lighthouse message
    libsurvive_ros2::msg::Lighthouse lighthouse_msg;
    lighthouse_msg.header.stamp = _singleton->get_ros_time(timecode);
    lighthouse_msg.header.frame_id = serial;
    lighthouse_msg.index = idx;
    lighthouse_msg.channel = ctx->bsd[idx].mode;
    lighthouse_msg.unlock_count = ctx->bsd[idx].sys_unlock_count;
    ros_from_point(&lighthouse_msg.accel, ctx->bsd[idx].accel);
    for (size_t i = 0; i < 2; i++) {
      lighthouse_msg.fcalphase[i] = ctx->bsd[idx].fcal[i].phase;
      lighthouse_msg.fcaltilt[i] = ctx->bsd[idx].fcal[i].tilt;
      lighthouse_msg.fcalcurve[i] = ctx->bsd[idx].fcal[i].curve;
      lighthouse_msg.fcalgibpha[i] = ctx->bsd[idx].fcal[i].gibpha;
      lighthouse_msg.fcalgibmag[i] = ctx->bsd[idx].fcal[i].gibmag;
      lighthouse_msg.fcalogeephase[i] = ctx->bsd[idx].fcal[i].ogeephase;
      lighthouse_msg.fcalogeemag[i] = ctx->bsd[idx].fcal[i].ogeemag;
    }
    lighthouse_msg.ootx_set = ctx->bsd[idx].OOTXSet;
    lighthouse_msg.position_set = ctx->bsd[idx].PositionSet;
    // Add or update the lighthouse in our internal data structure. It will
    // be published at a fixed cadence for all to see.
    _singleton->add_or_update_lighthouse(lighthouse_msg);
  }
}

// Callback for tracker event
static int config_func(struct SurviveObject * so, char * ct0conf, int len)
{
  const int res = survive_default_config_process(so, ct0conf, len);
  if (_singleton) {
    // Get the approximate time
    FLT timecode = survive_run_time_since_epoch(so->ctx);
    // Package up tracker config data packet
    libsurvive_ros2::msg::Tracker tracker_msg;
    tracker_msg.header.stamp = _singleton->get_ros_time(timecode);
    tracker_msg.header.frame_id = std::string(so->serial_number);
    tracker_msg.tracker_from_imu.position.x = so->imu2trackref.Pos[0];
    tracker_msg.tracker_from_imu.position.y = so->imu2trackref.Pos[1];
    tracker_msg.tracker_from_imu.position.z = so->imu2trackref.Pos[2];
    tracker_msg.tracker_from_imu.orientation.w = so->imu2trackref.Rot[0];
    tracker_msg.tracker_from_imu.orientation.x = so->imu2trackref.Rot[1];
    tracker_msg.tracker_from_imu.orientation.y = so->imu2trackref.Rot[2];
    tracker_msg.tracker_from_imu.orientation.z = so->imu2trackref.Rot[3];
    tracker_msg.tracker_from_head.position.x = so->head2trackref.Pos[0];
    tracker_msg.tracker_from_head.position.y = so->head2trackref.Pos[1];
    tracker_msg.tracker_from_head.position.z = so->head2trackref.Pos[2];
    tracker_msg.tracker_from_head.orientation.w = so->head2trackref.Rot[0];
    tracker_msg.tracker_from_head.orientation.x = so->head2trackref.Rot[1];
    tracker_msg.tracker_from_head.orientation.y = so->head2trackref.Rot[2];
    tracker_msg.tracker_from_head.orientation.z = so->head2trackref.Rot[3];
    ros_from_point(&tracker_msg.accel_bias, so->acc_bias);
    ros_from_point(&tracker_msg.accel_scale, so->acc_scale);
    ros_from_point(&tracker_msg.omega_bias, so->gyro_bias);
    ros_from_point(&tracker_msg.omega_scale, so->gyro_scale);
    tracker_msg.channels.resize(so->sensor_ct);
    tracker_msg.normals.resize(so->sensor_ct);
    tracker_msg.points.resize(so->sensor_ct);
    for (uint8_t k = 0; k < so->sensor_ct; k++) {
      tracker_msg.channels[k] = so->channel_map[k];
      ros_from_point(&tracker_msg.points[k], &so->sensor_locations[k * 3]);
      ros_from_point(&tracker_msg.normals[k], &so->sensor_normals[k * 3]);
    }
    // Add or update the lighthouse in our internal data structure. It will
    // be published at a fixed cadence for all to see.
    _singleton->add_or_update_tracker(tracker_msg);
  }
  return res;
}

// Callback for angle event
static void sweep_angle_func(
  SurviveObject * so,
  uint8_t channel, int sensor_id, uint32_t rawtime, int8_t plane, FLT angle)
{
  survive_default_sweep_angle_process(so, channel, sensor_id, rawtime, plane, angle);
  if (_singleton) {
    // Get the approximate time
    auto long_timecode = SurviveSensorActivations_long_timecode_light(&so->activations, rawtime);
    FLT timecode = SurviveSensorActivations_runtime(&so->activations, long_timecode) / FLT(1e6);
    // Package up an angle measurement
    libsurvive_ros2::msg::Angle angle_msg;
    angle_msg.header.stamp = _singleton->get_ros_time(timecode);
    angle_msg.header.frame_id = std::string(so->serial_number);
    angle_msg.sensor_id = sensor_id;
    angle_msg.channel = channel;
    angle_msg.plane = plane;
    angle_msg.angle = angle;
    _singleton->publish_angle(angle_msg);
  }
}

// Callback for new lighthouse pose
static void lighthouse_pose_func(
  SurviveContext * ctx, uint8_t lighthouse, const SurvivePose * lighthouse_pose)
{
  survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose);
  if (_singleton && lighthouse_pose) {
    // Get serial number of lighthouse.
    char serial[16];
    snprintf(serial, sizeof(serial), "LHB-%X", (unsigned)ctx->bsd[lighthouse].BaseStationID);
    // Get the approximate time
    FLT timecode = survive_run_time_since_epoch(ctx);
    // Package up a transform
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = _singleton->get_ros_time(timecode);
    transform_msg.header.frame_id = _singleton->get_tracking_frame();
    transform_msg.child_frame_id = serial;
    ros_from_pose(&transform_msg.transform, *lighthouse_pose);
    _singleton->publish_tf(transform_msg, /* is_static= */ true);
    // Publish a separate pose message with covariance
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = transform_msg.header.stamp;
    pose_msg.header.frame_id = serial;
    pose_msg.pose.pose.orientation = transform_msg.transform.rotation;
    pose_msg.pose.pose.position.x = transform_msg.transform.translation.x;
    pose_msg.pose.pose.position.y = transform_msg.transform.translation.y;
    pose_msg.pose.pose.position.z = transform_msg.transform.translation.z;
    pose_msg.pose.covariance.fill(0.0);
    pose_msg.pose.covariance[0] = kPosSigma_meters * kPosSigma_meters;
    pose_msg.pose.covariance[7] = kPosSigma_meters * kPosSigma_meters;
    pose_msg.pose.covariance[14] = kPosSigma_meters * kPosSigma_meters;
    pose_msg.pose.covariance[21] = kRotSigma_radians * kRotSigma_radians;
    pose_msg.pose.covariance[28] = kRotSigma_radians * kRotSigma_radians;
    pose_msg.pose.covariance[35] = kRotSigma_radians * kRotSigma_radians;
    _singleton->publish_lighthouse_pose(pose_msg);
  }
}

// Callback for tracked object pose
static void tracker_pose_func(
  SurviveObject * so, survive_long_timecode rawtime, const SurvivePose * pose)
{
  survive_default_pose_process(so, rawtime, pose);
  if (_singleton && pose) {
    // Get the precise time
    FLT timecode = SurviveSensorActivations_runtime(
      &so->activations, so->OutPose_timecode) / FLT(1e6);
    // Package up a transform
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = _singleton->get_ros_time(timecode);
    transform_msg.header.frame_id = _singleton->get_tracking_frame();
    transform_msg.child_frame_id = std::string(so->serial_number);
    ros_from_pose(&transform_msg.transform, *pose);
    _singleton->publish_tf(transform_msg, /* is_static= */ false);
    // Publish a separate pose message with covariance
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = transform_msg.header.stamp;
    pose_msg.header.frame_id = std::string(so->serial_number);
    pose_msg.pose.pose.orientation = transform_msg.transform.rotation;
    pose_msg.pose.pose.position.x = transform_msg.transform.translation.x;
    pose_msg.pose.pose.position.y = transform_msg.transform.translation.y;
    pose_msg.pose.pose.position.z = transform_msg.transform.translation.z;
    pose_msg.pose.covariance.fill(0.0);
    pose_msg.pose.covariance[0] = kPosSigma_meters * kPosSigma_meters;
    pose_msg.pose.covariance[7] = kPosSigma_meters * kPosSigma_meters;
    pose_msg.pose.covariance[14] = kPosSigma_meters * kPosSigma_meters;
    pose_msg.pose.covariance[21] = kRotSigma_radians * kRotSigma_radians;
    pose_msg.pose.covariance[28] = kRotSigma_radians * kRotSigma_radians;
    pose_msg.pose.covariance[35] = kRotSigma_radians * kRotSigma_radians;
    _singleton->publish_tracker_pose(pose_msg);
  }
}

namespace libsurvive_ros2
{

DriverComponent::DriverComponent(const rclcpp::NodeOptions & options)
: Node("libsurvive_ros2_driver_node", options),
  ctx_(nullptr),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
  tf_static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this))
{
  // Store the instance globally to be used by a C callback.
  _singleton = this;

  // Low-level driver
  this->declare_parameter("driver_args", "");
  this->get_parameter("driver_args", driver_args_);
  this->declare_parameter("driver_config_in", "~/.config/libsurvive/config.json");
  this->get_parameter("driver_config_in", driver_config_in_);
  this->declare_parameter("driver_config_out", "~/.config/libsurvive/config.json");
  this->get_parameter("driver_config_out", driver_config_out_);

  // General options
  this->declare_parameter("tracking_frame", "libsurvive_frame");
  this->get_parameter("tracking_frame", tracking_frame_);
  this->declare_parameter("recalibrate", false);
  this->get_parameter("recalibrate", recalibrate_);

  // TODO(asymingt) this might not play very well with composable node inter-process
  // communication. So, we might want to avoid it until everything else is ready.
  rclcpp::QoS qos_profile(10);
  // qos_profile.durability_volatile();

  // Setup topics for publishing
  imu_publisher_ =
    this->create_publisher<sensor_msgs::msg::Imu>("imu", qos_profile);
  button_publisher_ =
    this->create_publisher<sensor_msgs::msg::Joy>("button", qos_profile);
  angle_publisher_ =
    this->create_publisher<libsurvive_ros2::msg::Angle>("angle", qos_profile);
  lighthouse_publisher_ =
    this->create_publisher<libsurvive_ros2::msg::Lighthouse>("lighthouse", qos_profile);
  tracker_publisher_ =
    this->create_publisher<libsurvive_ros2::msg::Tracker>("tracker", qos_profile);
  tracker_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/pose/tracker",
    qos_profile);
  lighthouse_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/pose/lighthouse",
    qos_profile);

  // Callback timer for republishing lighthouse and tracker information. We do this
  // because composable nodes don't support transient local topics, and so if the
  // poser joins after the tracker info is published, it will be lost.
  timer_ = this->create_wall_timer(1s, std::bind(&DriverComponent::timer_callback, this));

  // Setup driver parameters.
  std::vector<std::string> args;
  args.emplace_back(this->get_name());
  args.emplace_back("--v");
  args.emplace_back("100");
  args.emplace_back("--lighthouse-gen");
  args.emplace_back("2");
  args.emplace_back("--init-configfile");
  args.emplace_back(driver_config_in_);
  args.emplace_back("--configfile");
  args.emplace_back(driver_config_out_);
  if (recalibrate_) {
    args.emplace_back("--force-calibrate");
    args.emplace_back("1");
  } else {
    args.emplace_back("--force-calibrate");
    args.emplace_back("0");
  }
  std::stringstream driver_ss(driver_args_);
  std::string token;
  while (getline(driver_ss, token, ' ')) {
    args.emplace_back(token);
  }

  // Do an argument conversion, initialize, survive and clean up memory immediately.
  int argc = args.size();
  char **argv = new char*[args.size()];
  for (int i = 0; i < argc; i++) {
    argv[i] = new char[args[i].length()];
    std::strcpy(argv[i], args[i].c_str());
  }
  ctx_ = survive_init(argc, argv);
  for (int i = 0; i < argc; i++) {
    delete argv[i];
  }
  delete[] argv;
  if (ctx_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "Could not initialize the libsurvive context");
    return;
  }

  // Initialize the driver library.
  survive_startup(ctx_);

  // Setup callback for reading button data.
  survive_install_button_fn(ctx_, button_func);

  // Setup callbacks for tracked objects and lighthouse poses
  survive_install_pose_fn(ctx_, tracker_pose_func);
  survive_install_lighthouse_pose_fn(ctx_, lighthouse_pose_func);

  // Setup callbacks for raw data.
  survive_install_imu_fn(ctx_, imu_func);
  survive_install_sweep_angle_fn(ctx_, sweep_angle_func);
  survive_install_ootx_received_fn(ctx_, ootx_received_func);
  survive_install_config_fn(ctx_, config_func);

  // Start the work thread
  worker_thread_ = std::thread(&DriverComponent::work, this);
}

DriverComponent::~DriverComponent()
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up.");
  worker_thread_.join();

  RCLCPP_INFO(this->get_logger(), "Shutting down libsurvive driver");
  if (ctx_) {
    survive_close(ctx_);
  }

  RCLCPP_INFO(this->get_logger(), "Clearing singleton instance");
  _singleton = nullptr;
}

void DriverComponent::add_or_update_lighthouse(const libsurvive_ros2::msg::Lighthouse & msg)
{
  this->lighthouses_.emplace(msg.header.frame_id, msg);
}

void DriverComponent::add_or_update_tracker(const libsurvive_ros2::msg::Tracker & msg)
{
  this->trackers_.emplace(msg.header.frame_id, msg);
}

void DriverComponent::timer_callback()
{
  for (const auto & [name, msg] : this->lighthouses_) {
    this->publish_lighthouse(msg);
  }
  for (const auto & [name, msg] : this->trackers_) {
    this->publish_tracker(msg);
  }
}

rclcpp::Time DriverComponent::get_ros_time(FLT timecode)
{
  return rclcpp::Time(0, 0) + rclcpp::Duration(std::chrono::duration<double>(timecode));
}

void DriverComponent::publish_imu(const sensor_msgs::msg::Imu & msg)
{
  if (imu_publisher_) {
    imu_publisher_->publish(msg);
  }
}

void DriverComponent::publish_angle(const libsurvive_ros2::msg::Angle & msg)
{
  if (angle_publisher_) {
    angle_publisher_->publish(msg);
  }
}

void DriverComponent::publish_lighthouse(const libsurvive_ros2::msg::Lighthouse & msg)
{
  if (lighthouse_publisher_) {
    lighthouse_publisher_->publish(msg);
  }
}

void DriverComponent::publish_tracker(const libsurvive_ros2::msg::Tracker & msg)
{
  if (tracker_publisher_) {
    tracker_publisher_->publish(msg);
  }
}

void DriverComponent::publish_button(const sensor_msgs::msg::Joy & msg)
{
  if (button_publisher_) {
    button_publisher_->publish(msg);
  }
}

void DriverComponent::publish_tracker_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  if (tracker_pose_publisher_) {
    tracker_pose_publisher_->publish(msg);
  }
}

void DriverComponent::publish_lighthouse_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  if (lighthouse_pose_publisher_) {
    lighthouse_pose_publisher_->publish(msg);
  }
}

void DriverComponent::publish_tf(const geometry_msgs::msg::TransformStamped & msg, bool is_static)
{
  if (is_static) {
    if (tf_static_broadcaster_) {
      tf_static_broadcaster_->sendTransform(msg);
    }
  } else {
    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(msg);
    }
  }
}

const std::string & DriverComponent::get_tracking_frame() const
{
  return tracking_frame_;
}

void DriverComponent::work()
{
  RCLCPP_INFO(this->get_logger(), "Start listening for events..");
  while (rclcpp::ok() && (survive_poll(ctx_) == 0)) {}
}

}  // namespace libsurvive_ros2

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(libsurvive_ros2::DriverComponent)
