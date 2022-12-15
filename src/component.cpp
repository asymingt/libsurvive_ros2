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

#include <libsurvive_ros2/component.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace libsurvive_ros2 {

Component::Component(const rclcpp::NodeOptions& options)
  : Node("libsurvive_ros2", options)
  , actx_(nullptr)
  , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  , tf_static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this)) {

  // Convert the ROS parameters to
  std::string driver_args;
  this->declare_parameter("driver_args", "--force-recalibrate 1"); 
  this->get_parameter("driver_args", driver_args);
  std::vector<const char*> args;
  std::stringstream driver_ss(driver_args);
  std::string token;
  while (getline(driver_ss, token, ' ')) {
      args.emplace_back(token.c_str());
  }
 
  // Try and initialize survive with the arguments supplied
  actx_ = survive_simple_init(args.size(), const_cast<char **>(args.data()));
  if (actx_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "Could not initialize the libsurvive context");
    return;
  }

  // Initialize the survive thread.
  survive_simple_start_thread(actx_);

  // Start the work thread
  worker_thread_ = std::thread(&Component::work, this);
}

Component::~Component() {
  RCLCPP_INFO(this->get_logger(), "Cleaning up.");
  worker_thread_.join();

  RCLCPP_INFO(this->get_logger(), "Shutting down libsurvive driver");
  if (actx_) {
    survive_simple_close(actx_);
  }
}

void Component::work() {
  
  // Poll for events.
  RCLCPP_INFO(this->get_logger(), "Start listening for events..");
  struct SurviveSimpleEvent event = {};
  while (survive_simple_wait_for_event(actx_, &event) != SurviveSimpleEventType_Shutdown && rclcpp::ok()) {
    switch (event.event_type) {

    // Pose change events:
    case SurviveSimpleEventType_PoseUpdateEvent: {
      const struct SurviveSimplePoseUpdatedEvent *pose_event = survive_simple_get_pose_updated_event(&event);
      if (survive_simple_object_get_type(pose_event->object) != SurviveSimpleObject_LIGHTHOUSE) {
        SurvivePose pose = {};
        auto timecode = survive_simple_object_get_latest_pose(pose_event->object, &pose);
        if (timecode > 0) {
          geometry_msgs::msg::TransformStamped pose_msg;
          pose_msg.header.stamp = this->get_clock()->now();
          pose_msg.header.frame_id = "libsurvive_world";
          pose_msg.child_frame_id = survive_simple_serial_number(pose_event->object);
          pose_msg.transform.translation.x = pose.Pos[0];
          pose_msg.transform.translation.y = pose.Pos[1];
          pose_msg.transform.translation.z = pose.Pos[2];
          pose_msg.transform.rotation.w = pose.Rot[0];
          pose_msg.transform.rotation.x = pose.Rot[1];
          pose_msg.transform.rotation.y = pose.Rot[2];
          pose_msg.transform.rotation.z = pose.Rot[3];
          tf_broadcaster_->sendTransform(pose_msg);
        }
      }
      break;
    }

    // Button press events:
    case SurviveSimpleEventType_ButtonEvent: {
      RCLCPP_WARN(this->get_logger(), "Button events are not yet implemented");
      break;
    }
    
    // Configuration events
    case SurviveSimpleEventType_ConfigEvent: {
      RCLCPP_WARN(this->get_logger(), "Configuration events are not yet implemented");
      break;
    }

    // Unknown events
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown event");
      break;
    }

    // Publish base stations:
    auto time_now = this->get_clock()->now();
    if (time_now.seconds() - last_base_station_update_.seconds() > 0.25) {
      last_base_station_update_ = time_now;
      for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx_); it != 0;
        it = survive_simple_get_next_object(actx_, it)) {
        if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
          SurvivePose pose = {};
          auto timecode = survive_simple_object_get_latest_pose(it, &pose);
          if (timecode > 0) {
            geometry_msgs::msg::TransformStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = "libsurvive_world";
            pose_msg.child_frame_id = survive_simple_serial_number(it);
            pose_msg.transform.translation.x = pose.Pos[0];
            pose_msg.transform.translation.y = pose.Pos[1];
            pose_msg.transform.translation.z = pose.Pos[2];
            pose_msg.transform.rotation.w = pose.Rot[0];
            pose_msg.transform.rotation.x = pose.Rot[1];
            pose_msg.transform.rotation.y = pose.Rot[2];
            pose_msg.transform.rotation.z = pose.Rot[3];
            tf_static_broadcaster_->sendTransform(pose_msg);
          }
        }
      }
    }
  }
}

}  // namespace libsurvive_ros2

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(libsurvive_ros2::Component)