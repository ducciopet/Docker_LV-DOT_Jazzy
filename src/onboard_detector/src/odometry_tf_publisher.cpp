#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

class OdometryTfPublisher : public rclcpp::Node
{
public:
  OdometryTfPublisher()
  : Node("odometry_tf_publisher")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/glim_ros/odom_corrected");
    odom_output_topic_ = this->declare_parameter<std::string>("odom_output_topic", "/glim_ros/odom_corrected_to_base_link");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu_link");
    base_link_frame_ = this->declare_parameter<std::string>("base_link_frame", "base_link");

    // Use SensorDataQoS (BEST_EFFORT) to match GLIM's publisher QoS
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryTfPublisher::odomCallback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(
      this->get_logger(),
      "Listening for odometry on '%s', publishing corrected odometry to '%s', and broadcasting TF from '%s' to '%s'",
      odom_topic_.c_str(), odom_output_topic_.c_str(), odom_frame_.c_str(), base_link_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string parent_frame = odom_frame_.empty() ? msg->header.frame_id : odom_frame_;
    const std::string input_child_frame = msg->child_frame_id.empty() ? imu_frame_ : msg->child_frame_id;
    const std::string output_child_frame = base_link_frame_;

    if (parent_frame.empty() || input_child_frame.empty() || output_child_frame.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Odometry message or configuration missing required frames: header.frame_id='%s', child_frame_id='%s'",
        parent_frame.c_str(), input_child_frame.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped imu_to_base_tf;
    try {
      // T_imu_base: lookupTransform(imu_link, base_link) so that
      // T_odom_base = T_odom_imu * T_imu_base
      imu_to_base_tf = tf_buffer_.lookupTransform(
        input_child_frame, output_child_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unable to look up transform from '%s' to '%s': %s",
        output_child_frame.c_str(), input_child_frame.c_str(), ex.what());
      return;
    }

    // T_odom_imu: pose of imu_link in odom frame (from GLIM)
    tf2::Transform odom_to_imu;
    tf2::fromMsg(msg->pose.pose, odom_to_imu);

    // T_imu_base: pose of base_link in imu_link frame (static)
    tf2::Transform imu_to_base;
    tf2::fromMsg(imu_to_base_tf.transform, imu_to_base);

    // Redefine odom so that base_link starts at the origin:
    //   T_odom'_base = T_base_imu * T_odom_imu * T_imu_base
    // At startup T_odom_imu = I → T_base_imu * T_imu_base = I  ✓
    tf2::Transform base_to_imu = imu_to_base.inverse();
    tf2::Transform odom_to_base = base_to_imu * odom_to_imu * imu_to_base;
    const auto odom_to_base_transform = tf2::toMsg(odom_to_base);

    nav_msgs::msg::Odometry output_odom;
    output_odom.header.stamp = msg->header.stamp;
    output_odom.header.frame_id = parent_frame;
    output_odom.child_frame_id = output_child_frame;
    output_odom.pose.pose.position.x = odom_to_base_transform.translation.x;
    output_odom.pose.pose.position.y = odom_to_base_transform.translation.y;
    output_odom.pose.pose.position.z = odom_to_base_transform.translation.z;
    output_odom.pose.pose.orientation = odom_to_base_transform.rotation;
    output_odom.pose.covariance = msg->pose.covariance;

    // Rotate velocity from imu_link to base_link frame: R_base_imu = R_imu_base^{-1}
    const tf2::Quaternion base_from_imu_quat = imu_to_base.getRotation().inverse();
    const tf2::Vector3 linear(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const tf2::Vector3 angular(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    const tf2::Vector3 rotated_linear = tf2::quatRotate(base_from_imu_quat, linear);
    const tf2::Vector3 rotated_angular = tf2::quatRotate(base_from_imu_quat, angular);

    output_odom.twist.twist.linear.x = rotated_linear.x();
    output_odom.twist.twist.linear.y = rotated_linear.y();
    output_odom.twist.twist.linear.z = rotated_linear.z();
    output_odom.twist.twist.angular.x = rotated_angular.x();
    output_odom.twist.twist.angular.y = rotated_angular.y();
    output_odom.twist.twist.angular.z = rotated_angular.z();
    output_odom.twist.covariance = msg->twist.covariance;

    odom_publisher_->publish(output_odom);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = output_child_frame;
    tf_msg.transform.translation.x = output_odom.pose.pose.position.x;
    tf_msg.transform.translation.y = output_odom.pose.pose.position.y;
    tf_msg.transform.translation.z = output_odom.pose.pose.position.z;
    tf_msg.transform.rotation = output_odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_topic_;
  std::string odom_output_topic_;
  std::string odom_frame_;
  std::string imu_frame_;
  std::string base_link_frame_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
