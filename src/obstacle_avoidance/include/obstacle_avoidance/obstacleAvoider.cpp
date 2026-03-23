/*
    FILE: obstacleAvoider.cpp
    ---------------------------------
    function implementation of obstacle avoider
*/
#include <obstacle_avoidance/obstacleAvoider.h>

namespace obstacle_avoidance {

ObstacleAvoider::ObstacleAvoider(rclcpp::Node::SharedPtr node) : node_(node) {
    filtered_pointcloud_topic_ = node_->declare_parameter<std::string>("filtered_pointcloud_topic", "/filtered_points");
    filtered_bboxes_topic_ = node_->declare_parameter<std::string>("filtered_bboxes_topic", "/filtered_bboxes");
    oriented_bboxes_topic_ = node_->declare_parameter<std::string>("oriented_bboxes_topic", "/oriented_bboxes");

    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        filtered_pointcloud_topic_, 10,
        std::bind(&ObstacleAvoider::pointcloud_callback, this, std::placeholders::_1));
    bbox_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        filtered_bboxes_topic_, 10,
        std::bind(&ObstacleAvoider::bbox_callback, this, std::placeholders::_1));
    oriented_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        oriented_bboxes_topic_, 10);
}

void ObstacleAvoider::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    latest_cloud_ = msg;
    process();
}

void ObstacleAvoider::bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    latest_bboxes_ = msg;
    process();
}

void ObstacleAvoider::process() {
    if (!latest_cloud_ || !latest_bboxes_) return;

    // Convert PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*latest_cloud_, *cloud);

    visualization_msgs::msg::MarkerArray oriented_bboxes;

    for (const auto& marker : latest_bboxes_->markers) {
        // Extract box pose and scale
        Eigen::Vector3f box_center(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
        Eigen::Quaternionf box_orientation(
            marker.pose.orientation.w,
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z);
        Eigen::Vector3f box_scale(marker.scale.x, marker.scale.y, marker.scale.z);

        // Collect points inside the bounding box
        std::vector<Eigen::Vector3f> inside_points;
        for (const auto& pt : cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            // Transform point to box frame
            Eigen::Vector3f p_local = box_orientation.inverse() * (p - box_center);
            if (std::abs(p_local.x()) <= box_scale.x() / 2 &&
                std::abs(p_local.y()) <= box_scale.y() / 2 &&
                std::abs(p_local.z()) <= box_scale.z() / 2) {
                inside_points.push_back(p);
            }
        }

        if (!inside_points.empty()) {
            // Compute oriented bounding box for inside_points
            // (Here, for simplicity, we just reuse the original box. You can add PCA for a true OBB.)
            visualization_msgs::msg::Marker obb = marker;
            obb.header.stamp = node_->now();
            obb.ns = "oriented_bboxes";
            obb.color.r = 0.0;
            obb.color.g = 1.0;
            obb.color.b = 0.0;
            obb.color.a = 0.5;
            oriented_bboxes.markers.push_back(obb);
        }
    }
    oriented_bbox_pub_->publish(oriented_bboxes);
}

} // namespace obstacle_avoidance