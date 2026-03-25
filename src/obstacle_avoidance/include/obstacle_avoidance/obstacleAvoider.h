/*
    FILE: obstacleAvoider.h
    ---------------------------------
    header file of obstacle avoider
*/
#ifndef OBSTACLE_AVOIDANCE_OBSTACLEAVOIDER_H
#define OBSTACLE_AVOIDANCE_OBSTACLEAVOIDER_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>


namespace obstacle_avoidance {
class ObstacleAvoider {
public:
    ObstacleAvoider(rclcpp::Node::SharedPtr node);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_bbox_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr history_traj_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr oriented_bbox_pub_;

    std::string filtered_pointcloud_topic_;
    std::string filtered_bboxes_topic_;
    std::string dynamic_bboxes_topic_;
    std::string history_trajectory_topic_;
    std::string oriented_bboxes_topic_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    visualization_msgs::msg::MarkerArray::SharedPtr latest_bboxes_;
    visualization_msgs::msg::MarkerArray::SharedPtr latest_dynamic_bboxes_;
    visualization_msgs::msg::MarkerArray::SharedPtr latest_history_traj_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void dynamic_bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void history_traj_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void process();
};
}

#endif

