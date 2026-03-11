#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <open3d/Open3D.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class CalibrationICPNode : public rclcpp::Node {
public:
    CalibrationICPNode() : Node("calibration_icp_node") {
        depth_topic_ = this->declare_parameter(
            "depth_topic", std::string("/camera1/rs1/depth/image_rect_raw"));
        lidar_topic_ = this->declare_parameter(
            "velodyne_topic", std::string("/velodyne_points"));

        std::vector<double> depth_intrinsics = this->declare_parameter(
            "onboard_detector.depth_intrinsics",
            std::vector<double>{436.9647521972656, 436.9647521972656, 431.9205627441406, 240.13380432128906});

        depth_scale_ = this->declare_parameter("onboard_detector.depth_scale_factor", 1000.0);
        depth_min_ = this->declare_parameter("onboard_detector.depth_min_value", 0.5);
        depth_max_ = this->declare_parameter("onboard_detector.depth_max_value", 5.0);
        depth_skip_ = this->declare_parameter("onboard_detector.depth_skip_pixel", 2);

        lidar_frame_ = this->declare_parameter("lidar_frame", std::string("velodyne"));
        camera_frame_ = this->declare_parameter("camera_frame", std::string("rs1_link"));
        refined_camera_frame_ = this->declare_parameter("refined_camera_frame", camera_frame_ + "_refined");

        icp_max_corr_dist_ = this->declare_parameter("icp_max_correspondence_distance", 0.2);
        icp_max_iter_ = this->declare_parameter("icp_max_iteration", 100);
        icp_voxel_size_ = this->declare_parameter("icp_voxel_size", 0.05);
        icp_use_point_to_plane_ = this->declare_parameter("icp_use_point_to_plane", true);
        icp_normal_radius_ = this->declare_parameter("icp_normal_radius", 0.15);
        icp_normal_max_nn_ = this->declare_parameter("icp_normal_max_nn", 30);
        min_overlap_points_ = this->declare_parameter("min_overlap_points", 100);
        overlap_fov_margin_px_ = this->declare_parameter("overlap_fov_margin_px", 120);

        fx_ = depth_intrinsics[0];
        fy_ = depth_intrinsics[1];
        cx_ = depth_intrinsics[2];
        cy_ = depth_intrinsics[3];

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        rclcpp::QoS qos(1);
        qos.reliable();
        qos.transient_local();

        pub_lidar_overlap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/calibration/velodyne_points", qos);
        pub_depth_aligned_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/calibration/depth_cloud_in_velodyne", qos);

        sub_lidar_.subscribe(this, lidar_topic_);
        sub_depth_.subscribe(this, depth_topic_);

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::Image>;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(20), sub_lidar_, sub_depth_);
        sync_->registerCallback(&CalibrationICPNode::onData, this);

        RCLCPP_INFO(this->get_logger(), "Calibration ICP node ready (one-shot)");
        RCLCPP_INFO(this->get_logger(), "  depth topic: %s", depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  lidar topic: %s", lidar_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  initial TF from: %s -> %s", lidar_frame_.c_str(), camera_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  refined TF target child: %s", refined_camera_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  overlap FOV margin: %d px", overlap_fov_margin_px_);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_overlap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_depth_aligned_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2,
            sensor_msgs::msg::Image>>> sync_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::string depth_topic_;
    std::string lidar_topic_;
    std::string lidar_frame_;
    std::string camera_frame_;
    std::string refined_camera_frame_;

    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    double depth_scale_{1000.0};
    double depth_min_{0.5};
    double depth_max_{5.0};
    int depth_skip_{2};

    double icp_max_corr_dist_{0.2};
    int icp_max_iter_{100};
    double icp_voxel_size_{0.05};
    bool icp_use_point_to_plane_{true};
    double icp_normal_radius_{0.15};
    int icp_normal_max_nn_{30};
    int min_overlap_points_{100};
    int overlap_fov_margin_px_{120};

    bool processed_{false};
    Eigen::Matrix4d T_lidar_camera_init_{Eigen::Matrix4d::Identity()};

    static std::string matrixToString(const Eigen::Matrix4d& m) {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(6);
        for (int r = 0; r < 4; ++r) {
            oss << "[ ";
            for (int c = 0; c < 4; ++c) {
                oss << m(r, c);
                if (c < 3) {
                    oss << ", ";
                }
            }
            oss << " ]";
            if (r < 3) {
                oss << "\n";
            }
        }
        return oss.str();
    }

    static Eigen::Matrix4d transformMsgToEigen(const geometry_msgs::msg::Transform& tf_msg) {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(tf_msg.rotation.w, tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z);
        m.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        m(0, 3) = tf_msg.translation.x;
        m(1, 3) = tf_msg.translation.y;
        m(2, 3) = tf_msg.translation.z;
        return m;
    }

    static geometry_msgs::msg::Transform eigenToTransformMsg(const Eigen::Matrix4d& m) {
        geometry_msgs::msg::Transform out;
        Eigen::Quaterniond q(m.block<3, 3>(0, 0));
        out.translation.x = m(0, 3);
        out.translation.y = m(1, 3);
        out.translation.z = m(2, 3);
        out.rotation.w = q.w();
        out.rotation.x = q.x();
        out.rotation.y = q.y();
        out.rotation.z = q.z();
        return out;
    }

    static Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& rot) {
        Eigen::Quaterniond q(rot);
        tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
        tf2::Matrix3x3 tf_m(tf_q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf_m.getRPY(roll, pitch, yaw);
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    bool fetchInitialGuessFromTF(const builtin_interfaces::msg::Time& stamp) {
        try {
            auto tf_stamped = tf_buffer_->lookupTransform(
                lidar_frame_, camera_frame_, rclcpp::Time(stamp), std::chrono::milliseconds(200));
            T_lidar_camera_init_ = transformMsgToEigen(tf_stamped.transform);
            RCLCPP_INFO(this->get_logger(), "Initial TF guess from /tf (%s -> %s):\n%s",
                        lidar_frame_.c_str(), camera_frame_.c_str(), matrixToString(T_lidar_camera_init_).c_str());
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup at stamp failed: %s. Trying latest TF.", ex.what());
        }

        try {
            auto tf_stamped = tf_buffer_->lookupTransform(
                lidar_frame_, camera_frame_, tf2::TimePointZero, std::chrono::milliseconds(200));
            T_lidar_camera_init_ = transformMsgToEigen(tf_stamped.transform);
            RCLCPP_INFO(this->get_logger(), "Initial TF guess from latest /tf (%s -> %s):\n%s",
                        lidar_frame_.c_str(), camera_frame_.c_str(), matrixToString(T_lidar_camera_init_).c_str());
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read initial guess from /tf: %s", ex.what());
            return false;
        }
    }

    std::shared_ptr<open3d::geometry::PointCloud> depthToPointCloud(const cv::Mat& depth_image) const {
        auto cloud = std::make_shared<open3d::geometry::PointCloud>();
        cloud->points_.reserve((depth_image.rows / depth_skip_) * (depth_image.cols / depth_skip_));

        const bool is_u16 = depth_image.type() == CV_16UC1;
        const bool is_f32 = depth_image.type() == CV_32FC1;
        if (!is_u16 && !is_f32) {
            RCLCPP_ERROR(this->get_logger(), "Unsupported depth image encoding (need 16UC1 or 32FC1)");
            return cloud;
        }

        for (int y = 0; y < depth_image.rows; y += depth_skip_) {
            for (int x = 0; x < depth_image.cols; x += depth_skip_) {
                double z = 0.0;
                if (is_u16) {
                    const uint16_t d = depth_image.at<uint16_t>(y, x);
                    if (d == 0) {
                        continue;
                    }
                    z = static_cast<double>(d) / depth_scale_;
                } else {
                    const float d = depth_image.at<float>(y, x);
                    if (!std::isfinite(d) || d <= 0.0f) {
                        continue;
                    }
                    z = static_cast<double>(d);
                }

                if (z < depth_min_ || z > depth_max_) {
                    continue;
                }

                const double X = (static_cast<double>(x) - cx_) * z / fx_;
                const double Y = (static_cast<double>(y) - cy_) * z / fy_;
                cloud->points_.emplace_back(X, Y, z);
            }
        }

        return cloud;
    }

    std::shared_ptr<open3d::geometry::PointCloud> extractLidarOverlapWithDepthFov(
        const pcl::PointCloud<pcl::PointXYZ>& lidar_cloud,
        uint32_t image_width,
        uint32_t image_height,
        const Eigen::Matrix4d& T_lidar_camera) const {

        auto overlap = std::make_shared<open3d::geometry::PointCloud>();
        overlap->points_.reserve(lidar_cloud.points.size());

        const Eigen::Matrix4d T_camera_lidar = T_lidar_camera.inverse();

        for (const auto& pt : lidar_cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            Eigen::Vector4d p_lidar(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d p_camera = T_camera_lidar * p_lidar;

            const double z = p_camera(2);
            if (z <= depth_min_ || z >= depth_max_) {
                continue;
            }

            const double u = fx_ * p_camera(0) / z + cx_;
            const double v = fy_ * p_camera(1) / z + cy_;
            const double min_u = -static_cast<double>(overlap_fov_margin_px_);
            const double min_v = -static_cast<double>(overlap_fov_margin_px_);
            const double max_u = static_cast<double>(image_width) + static_cast<double>(overlap_fov_margin_px_);
            const double max_v = static_cast<double>(image_height) + static_cast<double>(overlap_fov_margin_px_);
            if (u < min_u || v < min_v || u >= max_u || v >= max_v) {
                continue;
            }

            overlap->points_.emplace_back(pt.x, pt.y, pt.z);
        }

        return overlap;
    }

    sensor_msgs::msg::PointCloud2 toROSPointCloud2(
        const open3d::geometry::PointCloud& cloud,
        const builtin_interfaces::msg::Time& stamp,
        const std::string& frame,
        uint8_t r,
        uint8_t g,
        uint8_t b) const {

        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl_cloud.points.reserve(cloud.points_.size());
        for (const auto& p : cloud.points_) {
            pcl::PointXYZRGB q;
            q.x = static_cast<float>(p(0));
            q.y = static_cast<float>(p(1));
            q.z = static_cast<float>(p(2));
            q.r = r;
            q.g = g;
            q.b = b;
            pcl_cloud.points.push_back(q);
        }

        pcl_cloud.width = static_cast<uint32_t>(pcl_cloud.points.size());
        pcl_cloud.height = 1;

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(pcl_cloud, out);
        out.header.stamp = stamp;
        out.header.frame_id = frame;
        return out;
    }

    void publishRefinedStaticTF(const Eigen::Matrix4d& T_lidar_camera_refined,
                                const builtin_interfaces::msg::Time& stamp) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = lidar_frame_;
        tf_msg.child_frame_id = refined_camera_frame_;
        tf_msg.transform = eigenToTransformMsg(T_lidar_camera_refined);
        static_tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(), "Published refined TF: %s -> %s",
                    lidar_frame_.c_str(), refined_camera_frame_.c_str());
    }

    void printCopyPasteTransform(const Eigen::Matrix4d& T_lidar_camera_refined) {
        const Eigen::Vector3d rpy = rotationToRPY(T_lidar_camera_refined.block<3, 3>(0, 0));
        const double x = T_lidar_camera_refined(0, 3);
        const double y = T_lidar_camera_refined(1, 3);
        const double z = T_lidar_camera_refined(2, 3);

        RCLCPP_INFO(this->get_logger(), "========== READY-TO-PASTE TF ==========");
        RCLCPP_INFO(this->get_logger(),
                    "ros2 run tf2_ros static_transform_publisher --x %.9f --y %.9f --z %.9f --roll %.9f --pitch %.9f --yaw %.9f --frame-id %s --child-frame-id %s",
                    x, y, z, rpy(0), rpy(1), rpy(2), lidar_frame_.c_str(), camera_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "=======================================");
    }

    void onData(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        if (processed_) {
            return;
        }

        std::string status_message;
        if (!runCalibration(lidar_msg, depth_msg, status_message)) {
            RCLCPP_WARN(this->get_logger(), "Calibration attempt failed: %s", status_message.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Published one-shot clouds: lidar overlap + aligned depth");
    }

    bool runCalibration(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                        std::string& status_message) {
        if (!fetchInitialGuessFromTF(depth_msg->header.stamp)) {
            status_message = "Cannot start calibration without initial TF guess velodyne->rs1_link";
            RCLCPP_ERROR(this->get_logger(), "%s", status_message.c_str());
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*lidar_msg, *lidar_cloud);

        cv::Mat depth_image;
        try {
            auto depth_cv = cv_bridge::toCvShare(depth_msg);
            depth_image = depth_cv->image;
        } catch (const std::exception& ex) {
            status_message = std::string("Depth cv_bridge conversion failed: ") + ex.what();
            RCLCPP_ERROR(this->get_logger(), "%s", status_message.c_str());
            return false;
        }

        auto depth_cloud_camera = depthToPointCloud(depth_image);
        if (depth_cloud_camera->points_.empty()) {
            status_message = "Depth point cloud is empty";
            RCLCPP_ERROR(this->get_logger(), "%s", status_message.c_str());
            return false;
        }

        auto lidar_overlap = extractLidarOverlapWithDepthFov(
            *lidar_cloud, depth_msg->width, depth_msg->height, T_lidar_camera_init_);
        if (static_cast<int>(lidar_overlap->points_.size()) < min_overlap_points_) {
            RCLCPP_WARN(this->get_logger(),
                        "LiDAR overlap too small (%zu points, min=%d). Using full LiDAR cloud as ICP target.",
                        lidar_overlap->points_.size(), min_overlap_points_);
            lidar_overlap = std::make_shared<open3d::geometry::PointCloud>();
            lidar_overlap->points_.reserve(lidar_cloud->points.size());
            for (const auto& pt : lidar_cloud->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                    continue;
                }
                lidar_overlap->points_.emplace_back(pt.x, pt.y, pt.z);
            }
        }

        auto source = depth_cloud_camera;
        auto target = lidar_overlap;

        if (icp_voxel_size_ > 0.0) {
            source = source->VoxelDownSample(icp_voxel_size_);
            target = target->VoxelDownSample(icp_voxel_size_);
            if (source->points_.empty()) {
                source = depth_cloud_camera;
            }
            if (target->points_.empty()) {
                target = lidar_overlap;
            }
        }

        if (icp_use_point_to_plane_) {
            target->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(icp_normal_radius_, icp_normal_max_nn_));
        }

        open3d::pipelines::registration::ICPConvergenceCriteria criteria;
        criteria.max_iteration_ = icp_max_iter_;

        open3d::pipelines::registration::RegistrationResult reg_result;
        if (icp_use_point_to_plane_) {
            reg_result = open3d::pipelines::registration::RegistrationICP(
                *source,
                *target,
                icp_max_corr_dist_,
                T_lidar_camera_init_,
                open3d::pipelines::registration::TransformationEstimationPointToPlane(),
                criteria);
        } else {
            reg_result = open3d::pipelines::registration::RegistrationICP(
                *source,
                *target,
                icp_max_corr_dist_,
                T_lidar_camera_init_,
                open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
                criteria);
        }

        const Eigen::Matrix4d T_lidar_camera_refined = reg_result.transformation_;

        RCLCPP_INFO(this->get_logger(), "ICP done. fitness=%.6f rmse=%.6f",
                    reg_result.fitness_, reg_result.inlier_rmse_);
        RCLCPP_INFO(this->get_logger(), "Refined %s->%s:\n%s",
                lidar_frame_.c_str(), refined_camera_frame_.c_str(),
                    matrixToString(T_lidar_camera_refined).c_str());

        auto depth_aligned = std::make_shared<open3d::geometry::PointCloud>(*source);
        depth_aligned->Transform(T_lidar_camera_refined);

        auto lidar_msg_out = toROSPointCloud2(*lidar_overlap, depth_msg->header.stamp, lidar_frame_, 0, 255, 0);
        auto depth_msg_out = toROSPointCloud2(*depth_aligned, depth_msg->header.stamp, lidar_frame_, 255, 80, 80);

        pub_lidar_overlap_->publish(lidar_msg_out);
        pub_depth_aligned_->publish(depth_msg_out);

        publishRefinedStaticTF(T_lidar_camera_refined, depth_msg->header.stamp);
        printCopyPasteTransform(T_lidar_camera_refined);

        processed_ = true;
        status_message = "Calibration completed and refined TF published";
        return true;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationICPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
