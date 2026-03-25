/*
    FILE: obstacleAvoider.cpp
    ---------------------------------
    function implementation of obstacle avoider
*/
#include <obstacle_avoidance/obstacleAvoider.h>

namespace obstacle_avoidance {

    ObstacleAvoider::ObstacleAvoider(rclcpp::Node::SharedPtr node) : node_(node) {
        filtered_pointcloud_topic_ = node_->declare_parameter<std::string>("filtered_pointcloud_topic", "/onboard_detector/filtered_point_cloud");
        filtered_bboxes_topic_ = node_->declare_parameter<std::string>("filtered_bboxes_topic", "/onboard_detector/filtered_bboxes");
        dynamic_bboxes_topic_ = node_->declare_parameter<std::string>("dynamic_bboxes_topic", "/onboard_detector/dynamic_bboxes");
        history_trajectory_topic_ = node_->declare_parameter<std::string>("history_trajectory_topic", "/onboard_detector/history_trajectory");
        oriented_bboxes_topic_ = node_->declare_parameter<std::string>("oriented_bboxes_topic", "/obstacle_avoidance/oriented_bboxes");

        pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            filtered_pointcloud_topic_, 10,
            std::bind(&ObstacleAvoider::pointcloud_callback, this, std::placeholders::_1));
        bbox_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            filtered_bboxes_topic_, 10,
            std::bind(&ObstacleAvoider::bbox_callback, this, std::placeholders::_1));
        dynamic_bbox_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            dynamic_bboxes_topic_, 10,
            std::bind(&ObstacleAvoider::dynamic_bbox_callback, this, std::placeholders::_1));
        history_traj_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            history_trajectory_topic_, 10,
            std::bind(&ObstacleAvoider::history_traj_callback, this, std::placeholders::_1));
        oriented_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            oriented_bboxes_topic_, 10);
    }
    
    void ObstacleAvoider::dynamic_bbox_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        latest_dynamic_bboxes_ = msg;
        // Optionally process or fuse with other data
    }

    void ObstacleAvoider::history_traj_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        latest_history_traj_ = msg;
        // Optionally process or fuse with other data
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
        int id = 0;
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
            std::vector<Eigen::Vector3f> box_points;
            Eigen::Matrix3f rot = box_orientation.toRotationMatrix();
            Eigen::Matrix3f rot_inv = rot.transpose();
            for (const auto& pt : cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f p_local = rot_inv * (p - box_center);
                if (std::abs(p_local.x()) <= box_scale.x() * 0.5 &&
                    std::abs(p_local.y()) <= box_scale.y() * 0.5 &&
                    std::abs(p_local.z()) <= box_scale.z() * 0.5) {
                    box_points.push_back(p);
                }
            }

            if (box_points.size() < 10) continue; // skip small clusters

            // Compute OBB using PCA
            Eigen::MatrixXf mat(box_points.size(), 3);
            for (size_t i = 0; i < box_points.size(); ++i) {
                mat.row(i) = box_points[i];
            }
            Eigen::Vector3f mean = mat.colwise().mean();
            Eigen::MatrixXf centered = mat.rowwise() - mean.transpose();
            Eigen::Matrix3f cov = centered.transpose() * centered / float(mat.rows() - 1);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
            Eigen::Matrix3f eigvecs = eig.eigenvectors();
            // Ensure right-handed coordinate system
            if (eigvecs.col(0).cross(eigvecs.col(1)).dot(eigvecs.col(2)) < 0) {
                eigvecs.col(2) *= -1;
            }

            // Project points to eigenvector axes to get extents
            Eigen::MatrixXf proj = centered * eigvecs;
            Eigen::Vector3f min_proj = proj.colwise().minCoeff();
            Eigen::Vector3f max_proj = proj.colwise().maxCoeff();
            Eigen::Vector3f obb_center_local = 0.5f * (min_proj + max_proj);
            Eigen::Vector3f obb_extents = max_proj - min_proj;
            Eigen::Vector3f obb_center = mean + eigvecs * obb_center_local;

            // Create Marker for OBB
            visualization_msgs::msg::Marker obb_marker;
            obb_marker.header = marker.header;
            obb_marker.header.stamp = node_->now();
            obb_marker.ns = "oriented_bboxes";
            obb_marker.id = id++;
            obb_marker.type = visualization_msgs::msg::Marker::CUBE;
            obb_marker.action = visualization_msgs::msg::Marker::ADD;
            obb_marker.pose.position.x = obb_center.x();
            obb_marker.pose.position.y = obb_center.y();
            obb_marker.pose.position.z = obb_center.z();
            Eigen::Quaternionf obb_q(eigvecs);
            obb_marker.pose.orientation.x = obb_q.x();
            obb_marker.pose.orientation.y = obb_q.y();
            obb_marker.pose.orientation.z = obb_q.z();
            obb_marker.pose.orientation.w = obb_q.w();
            obb_marker.scale.x = obb_extents.x();
            obb_marker.scale.y = obb_extents.y();
            obb_marker.scale.z = obb_extents.z();
            obb_marker.color.r = 0.0;
            obb_marker.color.g = 1.0;
            obb_marker.color.b = 0.0;
            obb_marker.color.a = 0.5;
            obb_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            obb_marker.frame_locked = true;
            oriented_bboxes.markers.push_back(obb_marker);
        }

        if (oriented_bbox_pub_ && !oriented_bboxes.markers.empty()) {
            oriented_bbox_pub_->publish(oriented_bboxes);
        }
    }

} // namespace obstacle_avoidance