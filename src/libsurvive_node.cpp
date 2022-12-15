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

// STL libraries
#include <mutex>
#include <cstdint>
#include <thread>
#include <vector>

// ROS2 core
#include <rclcpp/rclcpp.hpp>

// ROS2 libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ROS2 messages
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Libsurvive libraries
extern "C" {
#include <os_generic.h>
#include <survive.h>
#include <survive_api.h>
}

// Node to manage interaction with libsurvive
class SurviveNode : public rclcpp::Node
{
public:
  SurviveNode(SurviveSimpleContext *actx)
    : Node("libsurvive_ros2")
    , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
    , tf_static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this))
    , worker_thread_(&SurviveNode::work, this, actx) {
      RCLCPP_INFO(this->get_logger(), "Node initialized");
    }
  ~SurviveNode() {
    worker_thread_.join();
  }

private:
  void work(SurviveSimpleContext *actx) {
    RCLCPP_INFO(this->get_logger(), "Start listening for events..");
    struct SurviveSimpleEvent event = {};
    while (survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && rclcpp::ok()) {
      switch (event.event_type) {
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
      case SurviveSimpleEventType_ButtonEvent: {
        RCLCPP_WARN(this->get_logger(), "Button events are not yet implemented");
        break;
      }
      case SurviveSimpleEventType_ConfigEvent: {
        RCLCPP_WARN(this->get_logger(), "Configuration events are not yet implemented");
        break;
      }
      default:
        break;
      }
      // Publish base stations
      auto time_now = this->get_clock()->now();
      if (time_now.seconds() - last_base_station_update_.seconds() > 0.25) {
        last_base_station_update_ = time_now;
        for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
          it = survive_simple_get_next_object(actx, it)) {
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
    RCLCPP_INFO(this->get_logger(), "Exiting listener.");
  }
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::thread worker_thread_;
  rclcpp::Time last_base_station_update_;
};

int main(int argc, char *argv[])
{
  // Try and initialize survive with the arguments supplied
  SurviveSimpleContext *actx = survive_simple_init(argc, argv);
	if (actx == nullptr) {
	  return 0;
  }
  
  // Initialize survive
  survive_simple_start_thread(actx);
  
  // Start the ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurviveNode>(actx));
  rclcpp::shutdown();

  // Clean up the driver
	survive_simple_close(actx);
  return 0;
}
