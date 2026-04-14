#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class WorldAlignNode final : public rclcpp::Node
{
public:
  WorldAlignNode()
  : rclcpp::Node{"libsurvive_world_align_node"},
    joy_topic_{declare_parameter<std::string>("joy_topic", "/libsurvive/joy")},
    tracking_frame_{declare_parameter<std::string>("tracking_frame", "libsurvive_world")},
    world_frame_{declare_parameter<std::string>("world_frame", "world")},
    button_index_{static_cast<int>(declare_parameter<int>("button_index", 3))},
    lookup_timeout_{
      rclcpp::Duration::from_seconds(declare_parameter<double>("lookup_timeout_sec", 0.1))},
    tf_buffer_{get_clock()},
    tf_listener_{tf_buffer_},
    static_broadcaster_{std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)}
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) { OnJoy(msg); });

    RCLCPP_INFO(
      get_logger(),
      "World align node started. joy_topic=%s tracking_frame=%s world_frame=%s button_index=%d",
      joy_topic_.c_str(),
      tracking_frame_.c_str(),
      world_frame_.c_str(),
      button_index_);
  }

private:
  static tf2::Transform TrackerToWorldTransform(const tf2::Transform & tracker_wrt_libsurvive)
  {
    constexpr double kMinNorm = 1e-6;
    const tf2::Matrix3x3 basis_tracker_wrt_libsurvive = tracker_wrt_libsurvive.getBasis();

    tf2::Vector3 x_world_wrt_libsurvive = -basis_tracker_wrt_libsurvive.getColumn(1);
    x_world_wrt_libsurvive.setZ(0.0);
    if (x_world_wrt_libsurvive.length2() < kMinNorm) {
      x_world_wrt_libsurvive = -basis_tracker_wrt_libsurvive.getColumn(0);
      x_world_wrt_libsurvive.setZ(0.0);
    }
    if (x_world_wrt_libsurvive.length2() < kMinNorm) {
      x_world_wrt_libsurvive = tf2::Vector3(1.0, 0.0, 0.0);
    }
    x_world_wrt_libsurvive.normalize();

    const double yaw = std::atan2(x_world_wrt_libsurvive.y(), x_world_wrt_libsurvive.x());
    tf2::Quaternion world_quat_wrt_libsurvive;
    world_quat_wrt_libsurvive.setRPY(0.0, 0.0, yaw);
    world_quat_wrt_libsurvive.normalize();
    return tf2::Transform(world_quat_wrt_libsurvive, tracker_wrt_libsurvive.getOrigin());
  }

  void OnJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (
      msg->header.frame_id.empty() || button_index_ < 0 ||
      static_cast<size_t>(button_index_) >= msg->buttons.size())
    {
      return;
    }

    const bool pressed = (msg->buttons[button_index_] != 0);
    bool & prev_pressed = last_button_state_[msg->header.frame_id];
    if (!pressed || prev_pressed) {
      prev_pressed = pressed;
      return;
    }
    prev_pressed = true;

    try {
      const auto tf_msg = tf_buffer_.lookupTransform(
        tracking_frame_, msg->header.frame_id, msg->header.stamp, lookup_timeout_);

      tf2::Quaternion q_lt(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w);
      q_lt.normalize();
      const tf2::Transform t_lt(
        q_lt,
        tf2::Vector3(
          tf_msg.transform.translation.x,
          tf_msg.transform.translation.y,
          tf_msg.transform.translation.z));

      const tf2::Transform t_lw = TrackerToWorldTransform(t_lt);
      const tf2::Transform t_wl = t_lw.inverse();

      geometry_msgs::msg::TransformStamped world_to_lib;
      world_to_lib.header.stamp = msg->header.stamp;
      world_to_lib.header.frame_id = world_frame_;
      world_to_lib.child_frame_id = tracking_frame_;
      world_to_lib.transform.translation.x = t_wl.getOrigin().x();
      world_to_lib.transform.translation.y = t_wl.getOrigin().y();
      world_to_lib.transform.translation.z = t_wl.getOrigin().z();
      world_to_lib.transform.rotation.x = t_wl.getRotation().x();
      world_to_lib.transform.rotation.y = t_wl.getRotation().y();
      world_to_lib.transform.rotation.z = t_wl.getRotation().z();
      world_to_lib.transform.rotation.w = t_wl.getRotation().w();
      static_broadcaster_->sendTransform(world_to_lib);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to align world frame: %s", ex.what());
    }
  }

  std::string joy_topic_;
  std::string tracking_frame_;
  std::string world_frame_;
  int button_index_{3};
  rclcpp::Duration lookup_timeout_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::unordered_map<std::string, bool> last_button_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldAlignNode>());
  rclcpp::shutdown();
  return 0;
}
