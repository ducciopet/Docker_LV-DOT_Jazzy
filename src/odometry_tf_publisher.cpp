#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdometryTfPublisher : public rclcpp::Node
{
public:
  OdometryTfPublisher()
  : Node("odometry_tf_publisher")
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");

    // RELIABLE to match robot_localization EKF publisher QoS
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10),
      std::bind(&OdometryTfPublisher::odomCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(),
      "Broadcasting odom->base_link TF from '%s'", odom_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = msg->header;
    tf_msg.child_frame_id = msg->child_frame_id;
    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
