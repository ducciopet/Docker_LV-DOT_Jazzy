/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <onboard_detector/dynamicDetector.h>

namespace onboardDetector{
    // Ensure nextTrackId_ is initialized in the constructor
    dynamicDetector::dynamicDetector() : nextTrackId_(0) {
        this->ns_ = "onboard_detector";
        this->hint_ = "[onboardDetector]";
    }

    dynamicDetector::dynamicDetector(const rclcpp::Node::SharedPtr& nh){
        this->ns_ = "onboard_detector";
        this->hint_ = "[onboardDetector]";
        this->nh_ = nh;
        this->tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->nh_->get_clock());
        this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer_, this->nh_, false);
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void dynamicDetector::initDetector(const rclcpp::Node::SharedPtr& nh){
        this->nh_ = nh;
        this->tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->nh_->get_clock());
        this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer_, this->nh_, false);
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void dynamicDetector::initParam(){
                // Voxel size for filtering
                auto pname_voxel = [&](const std::string &p){ return this->ns_.empty() ? p : this->ns_ + "." + p; };
                if (!this->nh_->get_parameter(pname_voxel("voxel_size"), this->voxelSize_)) {
                    this->voxelSize_ = 0.1;
                    RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: voxel_size not set, using default 0.1");
                } else {
                    RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: voxel_size set to %f", this->voxelSize_);
                }
        // helper lambda to build parameter names
        auto pname = [&](const std::string &p){ return this->ns_.empty() ? p : this->ns_ + "." + p; };

        this->useTfPose_ = true;
        this->nh_->get_parameter(pname("use_tf_pose"), this->useTfPose_);

        if (!this->nh_->get_parameter(pname("tf_map_frame"), this->tfMapFrame_)) {
            this->tfMapFrame_ = "map";
            RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: tf_map_frame not set, using default '%s'", this->tfMapFrame_.c_str());
        }

        if (!this->nh_->get_parameter(pname("tf_lidar_frame"), this->tfLidarFrame_)) {
            this->tfLidarFrame_ = "velodyne";
            RCLCPP_WARN(this->nh_->get_logger(), "[dynamicDetector]: tf_lidar_frame not set, using default '%s'", this->tfLidarFrame_.c_str());
        }

        if (!this->nh_->get_parameter(pname("tf_depth_frame"), this->tfDepthFrame_)) {
            this->tfDepthFrame_.clear();
            RCLCPP_ERROR(this->nh_->get_logger(), "[dynamicDetector]: tf_depth_frame not set. Configure it in YAML.");
        }

        if (!this->nh_->get_parameter(pname("tf_color_frame"), this->tfColorFrame_)) {
            this->tfColorFrame_.clear();
            RCLCPP_ERROR(this->nh_->get_logger(), "[dynamicDetector]: tf_color_frame not set. Configure it in YAML.");
        }

        RCLCPP_INFO(this->nh_->get_logger(),
                    "[dynamicDetector]: TF pose mode: %s (map=%s, lidar=%s, depth=%s, color=%s)",
                    this->useTfPose_ ? "enabled" : "disabled",
                    this->tfMapFrame_.c_str(),
                    this->tfLidarFrame_.c_str(),
                    this->tfDepthFrame_.c_str(),
                    this->tfColorFrame_.c_str());

        // localization mode
        if (!this->nh_->get_parameter(pname("localization_mode"), this->localizationMode_)){
            this->localizationMode_ = 0;
            std::cout << this->hint_ << ": No localization mode option. Use default: pose" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Localization mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << std::endl;
        }

        // depth topic name
        if (!this->nh_->get_parameter(pname("depth_image_topic"), this->depthTopicName_)){
            this->depthTopicName_ = "/camera/depth/image_raw";
            std::cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << std::endl;
        }

        // color topic name
        if (!this->nh_->get_parameter(pname("color_image_topic"), this->colorImgTopicName_)){
            this->colorImgTopicName_ = "/camera/color/image_raw";
            std::cout << this->hint_ << ": No color image topic name. Use default: /camera/color/image_raw" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Color image topic: " << this->colorImgTopicName_ << std::endl;
        }

        // lidar topic name
        if (!this->nh_->get_parameter(pname("lidar_pointcloud_topic"), this->lidarTopicName_)){
            this->lidarTopicName_ = "/cloud_registered";
            std::cout << this->hint_ << ": No lidar pointcloud topic name. Use default: /cloud_registered" << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Lidar pointcloud topic: " << this->lidarTopicName_ << std::endl;
        }

        if (this->localizationMode_ == 0){
            if (!this->nh_->get_parameter(pname("pose_topic"), this->poseTopicName_)){
                this->poseTopicName_ = "/pose";
                std::cout << this->hint_ << ": No pose topic name. Use default: /pose" << std::endl;
            }
            else{
                std::cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << std::endl;
            }
        }

        if (this->localizationMode_ == 1){
            if (!this->nh_->get_parameter(pname("odom_topic"), this->odomTopicName_)){
                this->odomTopicName_ = "/odom";
                std::cout << this->hint_ << ": No odom topic name. Use default: /odom" << std::endl;
            }
            else{
                std::cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << std::endl;
            }
        }

        // depth intrinsics
        std::vector<double> depthIntrinsics (4);
        if (!this->nh_->get_parameter(pname("depth_intrinsics"), depthIntrinsics)){
            std::cout << this->hint_ << ": Please check camera intrinsics!" << std::endl;
            exit(0);
        }
        else{
            this->fx_ = depthIntrinsics[0];
            this->fy_ = depthIntrinsics[1];
            this->cx_ = depthIntrinsics[2];
            this->cy_ = depthIntrinsics[3];
            std::cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << std::endl;
        }

        // color intrinsics
        std::vector<double> colorIntrinsics (4);
        if (!this->nh_->get_parameter(pname("color_intrinsics"), colorIntrinsics)){
            std::cout << this->hint_ << ": Please check camera intrinsics!" << std::endl;
            exit(0);
        }
        else{
            this->fxC_ = colorIntrinsics[0];
            this->fyC_ = colorIntrinsics[1];
            this->cxC_ = colorIntrinsics[2];
            this->cyC_ = colorIntrinsics[3];
            std::cout << this->hint_ << ": fxC, fyC, cxC, cyC: " << "["  << this->fxC_ << ", " << this->fyC_  << ", " << this->cxC_ << ", "<< this->cyC_ << "]" << std::endl;
        }

        // depth scale factor
        if (!this->nh_->get_parameter(pname("depth_scale_factor"), this->depthScale_)){
            this->depthScale_ = 1000.0;
            std::cout << this->hint_ << ": No depth scale factor. Use default: 1000." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << std::endl;
        }

        // depth min value
        if (!this->nh_->get_parameter(pname("depth_min_value"), this->depthMinValue_)){
            this->depthMinValue_ = 0.2;
            std::cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << std::endl;
        }

        // depth max value
        if (!this->nh_->get_parameter(pname("depth_max_value"), this->depthMaxValue_)){
            this->depthMaxValue_ = 5.0;
            this->raycastMaxLength_ = 5.0;
            std::cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << std::endl;
        }
        else{
            this->raycastMaxLength_ = this->depthMaxValue_;
            std::cout << this->hint_ << ": Depth max value: " << this->depthMaxValue_ << std::endl;
        }

        // depth filter margin
        if (!this->nh_->get_parameter(pname("depth_filter_margin"), this->depthFilterMargin_)){
            this->depthFilterMargin_ = 0;
            std::cout << this->hint_ << ": No depth filter margin. Use default: 0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << std::endl;
        }

        // depth skip pixel
        if (!this->nh_->get_parameter(pname("depth_skip_pixel"), this->skipPixel_)){
            this->skipPixel_ = 1;
            std::cout << this->hint_ << ": No depth skip pixel. Use default: 1." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << std::endl;
        }

        // ------------------------------------------------------------------------------------
        // depth image columns
        if (!this->nh_->get_parameter(pname("image_cols"), this->imgCols_)){
            this->imgCols_ = 640;
            std::cout << this->hint_ << ": No depth image columns. Use default: 640." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << std::endl;
        }

        // depth rows
        if (!this->nh_->get_parameter(pname("image_rows"), this->imgRows_)){
            this->imgRows_ = 480;
            std::cout << this->hint_ << ": No depth image rows. Use default: 480." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << std::endl;
        }
        this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        this->pointsDepth_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        // ------------------------------------------------------------------------------------


        RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: Sensor extrinsic matrices are not loaded from YAML. Using TF-only pose computation.");

        // time step
        if (!this->nh_->get_parameter(pname("time_step"), this->dt_)){
            this->dt_ = 0.033;
            std::cout << this->hint_ << ": No time step parameter found. Use default: 0.033." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Time step for the system is set to: " << this->dt_ << std::endl;
        }  

        // ground height
        if (!this->nh_->get_parameter(pname("ground_height"), this->groundHeight_)){
            this->groundHeight_ = 0.1;
            std::cout << this->hint_ << ": No ground height parameter. Use default: 0.1m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << std::endl;
        }

        // roof height
        if (!this->nh_->get_parameter(pname("roof_height"), this->roofHeight_)){
            this->roofHeight_ = 2.0;
            std::cout << this->hint_ << ": No roof height parameter. Use default: 2.0m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Roof height is set to: " << this->roofHeight_ << std::endl;
        }

        // min num of points for a voxel to be occupied in voxel filter
        if (!this->nh_->get_parameter(pname("voxel_occupied_thresh"), this->voxelOccThresh_)){
            this->voxelOccThresh_ = 10;
            std::cout << this->hint_ << ": No voxel_occupied_threshold. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": min num of points for a voxel to be occupied in voxel filter is set to be: " << this->voxelOccThresh_ << std::endl;
        }

        // minimum number of points in each cluster
        if (!this->nh_->get_parameter(pname("dbscan_min_points_cluster"), this->dbMinPointsCluster_)){
            this->dbMinPointsCluster_ = 18;
            std::cout << this->hint_ << ": No DBSCAN minimum point in each cluster parameter. Use default: 18." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN Minimum point in each cluster is set to: " << this->dbMinPointsCluster_ << std::endl;
        }

        // search range
        if (!this->nh_->get_parameter(pname("dbscan_search_range_epsilon"), this->dbEpsilon_)){
            this->dbEpsilon_ = 0.3;
            std::cout << this->hint_ << ": No DBSCAN epsilon parameter. Use default: 0.3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN epsilon is set to: " << this->dbEpsilon_ << std::endl;
        }

        // lidar dbscan min points
        if (!this->nh_->get_parameter(pname("lidar_DBSCAN_min_points"), this->lidarDBMinPoints_)){
            this->lidarDBMinPoints_ = 10;
            std::cout << this->hint_ << ": No lidar DBSCAN minimum point in each cluster parameter. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Lidar DBSCAN Minimum point in each cluster is set to: " << this->lidarDBMinPoints_ << std::endl;
        }

        // lidar dbscan search range
        if (!this->nh_->get_parameter(pname("lidar_DBSCAN_epsilon"), this->lidarDBEpsilon_)){
            this->lidarDBEpsilon_ = 0.2;
            std::cout << this->hint_ << ": No lidar DBSCAN epsilon parameter. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Lidar DBSCAN epsilon is set to: " << this->lidarDBEpsilon_ << std::endl;
        }

        // lidar points downsample threshold
        if(!this->nh_->get_parameter(pname("downsample_threshold"), this->downSampleThresh_)){
            this->downSampleThresh_ = 4000;
            std::cout << this->hint_ << ": No downsample threshold parameter found. Use default: 4000." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Downsample threshold is set to: " << this->downSampleThresh_ << std::endl;
        }

        // gaussian downsample rate
        if (!this->nh_->get_parameter(pname("gaussian_downsample_rate"), this->gaussianDownSampleRate_)){
            this->gaussianDownSampleRate_ = 2;
            std::cout << this->hint_ << ": No gaussian downsample rate parameter found. Use default: 2." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Gaussian downsample rate is set to: " << this->gaussianDownSampleRate_ << std::endl;
        }

        // DBSCAN refinement enable
        if (!this->nh_->get_parameter(pname("dbscan_refinement_enable"), this->dbscanRefinementEnable_)){
            this->dbscanRefinementEnable_ = true;
            std::cout << this->hint_ << ": No DBSCAN refinement enable. Use default: true." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refinement enable: " << this->dbscanRefinementEnable_ << std::endl;
        }

        // DBSCAN refine max diagonal
        if (!this->nh_->get_parameter(pname("dbscan_refine_max_diagonal"), this->dbscanRefineMaxDiagonal_)){
            this->dbscanRefineMaxDiagonal_ = 2.0;
            std::cout << this->hint_ << ": No DBSCAN refine max diagonal. Use default: 2.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine max diagonal: " << this->dbscanRefineMaxDiagonal_ << std::endl;
        }

        // DBSCAN refine min density
        if (!this->nh_->get_parameter(pname("dbscan_refine_min_density"), this->dbscanRefineMinDensity_)){
            this->dbscanRefineMinDensity_ = 80.0;
            std::cout << this->hint_ << ": No DBSCAN refine min density. Use default: 80.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine min density: " << this->dbscanRefineMinDensity_ << std::endl;
        }

        // DBSCAN refine split min pts
        if (!this->nh_->get_parameter(pname("dbscan_refine_split_min_pts"), this->dbscanRefineSplitMinPts_)){
            this->dbscanRefineSplitMinPts_ = 8;
            std::cout << this->hint_ << ": No DBSCAN refine split min pts. Use default: 8." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine split min pts: " << this->dbscanRefineSplitMinPts_ << std::endl;
        }

        // DBSCAN refine split eps
        if (!this->nh_->get_parameter(pname("dbscan_refine_split_eps"), this->dbscanRefineSplitEps_)){
            this->dbscanRefineSplitEps_ = 0.08;
            std::cout << this->hint_ << ": No DBSCAN refine split eps. Use default: 0.08." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine split eps: " << this->dbscanRefineSplitEps_ << std::endl;
        }

        // DBSCAN refine min subcluster pts
        if (!this->nh_->get_parameter(pname("dbscan_refine_min_subcluster_pts"), this->dbscanRefineMinSubclusterPts_)){
            this->dbscanRefineMinSubclusterPts_ = 10;
            std::cout << this->hint_ << ": No DBSCAN refine min subcluster pts. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine min subcluster pts: " << this->dbscanRefineMinSubclusterPts_ << std::endl;
        }

        // DBSCAN refine axis slice width
        if (!this->nh_->get_parameter(pname("dbscan_refine_axis_slice_width"), this->dbscanRefineAxisSliceWidth_)){
            this->dbscanRefineAxisSliceWidth_ = 0.40;
            std::cout << this->hint_ << ": No DBSCAN refine axis slice width. Use default: 0.40." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine axis slice width: " << this->dbscanRefineAxisSliceWidth_ << std::endl;
        }

        // DBSCAN refine max depth
        if (!this->nh_->get_parameter(pname("dbscan_refine_max_depth"), this->dbscanRefineMaxDepth_)){
            this->dbscanRefineMaxDepth_ = 2;
            std::cout << this->hint_ << ": No DBSCAN refine max depth. Use default: 2." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine max depth: " << this->dbscanRefineMaxDepth_ << std::endl;
        }

        // DBSCAN refine recursive
        if (!this->nh_->get_parameter(pname("dbscan_refine_recursive"), this->dbscanRefineRecursive_)){
            this->dbscanRefineRecursive_ = true;
            std::cout << this->hint_ << ": No DBSCAN refine recursive. Use default: true." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine recursive: " << this->dbscanRefineRecursive_ << std::endl;
        }

        // DBSCAN refine min box volume
        if (!this->nh_->get_parameter(pname("dbscan_refine_min_box_volume"), this->dbscanRefineMinBoxVolume_)){
            this->dbscanRefineMinBoxVolume_ = 0.0001;
            std::cout << this->hint_ << ": No DBSCAN refine min box volume. Use default: 0.0001." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DBSCAN refine min box volume: " << this->dbscanRefineMinBoxVolume_ << std::endl;
        }

        // Visual IOU threshold
        if (!this->nh_->get_parameter(pname("visual_filtering_BBox_IOU_threshold"), this->visualboxIOUThresh_)){
            this->visualboxIOUThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for boununding box IOU filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for boununding box IOU filtering is set to: " << this->visualboxIOUThresh_ << std::endl;
        }
        
        // Visual IOV threshold
        if (!this->nh_->get_parameter(pname("visual_filtering_BBox_IOV_threshold"), this->visualboxIOVThresh_)){
            this->visualboxIOVThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for boununding box IOV filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for boununding box IOV filtering is set to: " << this->visualboxIOVThresh_ << std::endl;
        }

        // LiDAR Visual IOU threshold
        if (!this->nh_->get_parameter(pname("lidar_visual_filtering_BBox_IOU_threshold"), this->lidarVisualboxIOUThresh_)){
            this->lidarVisualboxIOUThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for LiDAR bounding box IOU filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for LiDAR bounding box IOU filtering is set to: " << this->lidarVisualboxIOUThresh_ << std::endl;
        }
        
        // LiDAR Visual IOV threshold
        if (!this->nh_->get_parameter(pname("lidar_visual_filtering_BBox_IOV_threshold"), this->lidarVisualboxIOVThresh_)){
            this->lidarVisualboxIOVThresh_ = 0.5;
            std::cout << this->hint_ << ": No threshold for LiDAR bounding box IOV filtering parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for LiDAR bounding box IOV filtering is set to: " << this->lidarVisualboxIOVThresh_ << std::endl;
        }

        // Visual Merging Flag
        if (!this->nh_->get_parameter(pname("visual_merging_flag"), this->visualmergingFlag_)){
            this->visualmergingFlag_ = "smaller";
            std::cout << this->hint_ << ": No visual merging flag parameter found. Use default: smaller." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Visual merging flag is set to: " << this->visualmergingFlag_ << std::endl;
        }

        // LiDAR Visual Merging Flag
        if (!this->nh_->get_parameter(pname("lidar_visual_merging_flag"), this->lidarVisualmergingFlag_)){
            this->lidarVisualmergingFlag_ = "smaller";
            std::cout << this->hint_ << ": No LiDAR visual merging flag parameter found. Use default: smaller." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": LiDAR visual merging flag is set to: " << this->lidarVisualmergingFlag_ << std::endl;
        }

        // UV unmerged Flag
        if (!this->nh_->get_parameter(pname("uv_unmerged_flag"), this->uvUnmergedFlag_)){
            this->uvUnmergedFlag_ = false;
            std::cout << this->hint_ << ": No UV unmerged flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": UV unmerged flag is set to: " << (this->uvUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // DB unmerged Flag
        if (!this->nh_->get_parameter(pname("db_unmerged_flag"), this->dbUnmergedFlag_)){
            this->dbUnmergedFlag_ = false;
            std::cout << this->hint_ << ": No DB unmerged flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": DB unmerged flag is set to: " << (this->dbUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // LiDAR unmerged Flag
        if (!this->nh_->get_parameter(pname("lidar_unmerged_flag"), this->lidarUnmergedFlag_)){
            this->lidarUnmergedFlag_ = true;
            std::cout << this->hint_ << ": No LiDAR unmerged flag parameter found. Use default: true." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": LiDAR unmerged flag is set to: " << (this->lidarUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // Same group IOV Threshold
        if (!this->nh_->get_parameter(pname("samegroupIOV_threshold"), this->samegroupIOVThresh_)){
            this->samegroupIOVThresh_ = 0.6;
            std::cout << this->hint_ << ": No same group IOV threshold parameter found. Use default: 0.6." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Same group IOV threshold is set to: " << this->samegroupIOVThresh_ << std::endl;
        }

        // Same group IOU Threshold
        if (!this->nh_->get_parameter(pname("samegroupIOU_threshold"), this->samegroupIOUThresh_)){
            this->samegroupIOUThresh_ = 0.3;
            std::cout << this->hint_ << ": No same group IOU threshold parameter found. Use default: 0.3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Same group IOU threshold is set to: " << this->samegroupIOUThresh_ << std::endl;
        }

        // Visual unmerged Flag
        if (!this->nh_->get_parameter(pname("visual_unmerged_flag"), this->visualUnmergedFlag_)){
            this->visualUnmergedFlag_ = false;
            std::cout << this->hint_ << ": No visual unmerged flag parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Visual unmerged flag is set to: " << (this->visualUnmergedFlag_ ? "true" : "false") << std::endl;
        }

        // maximum match range
        if (!this->nh_->get_parameter(pname("max_match_range"), this->maxMatchRange_)){
            this->maxMatchRange_ = 0.5;
            std::cout << this->hint_ << ": No max match range parameter found. Use default: 0.5m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max match range is set to: " << this->maxMatchRange_  << "m." << std::endl;
        }

        // maximum size difference for matching
        if (!this->nh_->get_parameter(pname("max_size_diff_range"), this->maxMatchSizeRange_)){
            this->maxMatchSizeRange_ = 0.5;
            std::cout << this->hint_ << ": No max size difference range for matching parameter found. Use default: 0.5m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Max size difference range for matching is set to: " << this->maxMatchSizeRange_ << "m." << std::endl;
        }

        // feature weight
        std::vector<double> tempWeights;
        if (!this->nh_->get_parameter(pname("feature_weight"), tempWeights)) {
            this->featureWeights_ = Eigen::VectorXd(10);
            this->featureWeights_ << 3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0, 0, 0;
            std::cout << this->hint_ << ": No feature weights parameter found. Using default feature weights: [3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0, 0, 0]." << std::endl;
        }
        else {
            this->featureWeights_ = Eigen::Map<Eigen::VectorXd>(tempWeights.data(), tempWeights.size());
            std::cout <<  this->hint_ << ": Feature weights are set to: [";
            for (size_t i = 0; i < tempWeights.size(); ++i) {
                std::cout << tempWeights[i];
                if (i != tempWeights.size()-1){
                    std::cout << ", ";
                }
            }
            std::cout << "]." << std::endl;
        }

        // tracking history size
        if (!this->nh_->get_parameter(pname("history_size"), this->histSize_)){
            this->histSize_ = 5;
            std::cout << this->hint_ << ": No tracking history size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": History for tracking is set to: " << this->histSize_ << std::endl;
        }

        // history threshold for fixing box size
        if (!this->nh_->get_parameter(pname("fix_size_history_threshold"), this->fixSizeHistThresh_)){
            this->fixSizeHistThresh_ = 10;
            std::cout << this->hint_ << ": No history threshold for fixing size parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": History threshold for fixing size parameter is set to: " << this->fixSizeHistThresh_ << std::endl;
        }

        // dimension threshold for fixing box size
        if (!this->nh_->get_parameter(pname("fix_size_dimension_threshold"), this->fixSizeDimThresh_)){
            this->fixSizeDimThresh_ = 0.4;
            std::cout << this->hint_ << ": No dimension threshold for fixing size parameter found. Use default: 0.4." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Dimension threshold for fixing size parameter is set to: " << this->fixSizeDimThresh_ << std::endl;
        }

        // kalman filter parameters
        std::vector<double> kalmanFilterParams;
        if (!this->nh_->get_parameter(pname("kalman_filter_param"), kalmanFilterParams)){
            this->eP_ = 0.5;
            this->eQPos_ = 0.5; // pos prediction noise
            this->eQVel_ = 0.5; // vel prediction noise
            this->eQAcc_ = 0.5; // acc prediction noise
            this->eRPos_ = 0.5; // pos measurement noise
            this->eRVel_ = 0.5; // vel measurement noise
            this->eRAcc_ = 0.5; // acc measurement noise
            std::cout << this->hint_ << ": No kalman filter parameter found. Use default: 0.5." << std::endl;
        }
        else{
            this->eP_ = kalmanFilterParams[0];
            this->eQPos_ = kalmanFilterParams[1]; // pos prediction noise
            this->eQVel_ = kalmanFilterParams[2]; // vel prediction noise
            this->eQAcc_ = kalmanFilterParams[3]; // acc prediction noise
            this->eRPos_ = kalmanFilterParams[4]; // pos measurement noise
            this->eRVel_ = kalmanFilterParams[5]; // vel measurement noise
            this->eRAcc_ = kalmanFilterParams[6]; // acc measurement noise
            std::cout << this->hint_ << ": Kalman filter parameter is set to: [";
            for (int i=0; i<int(kalmanFilterParams.size()); ++i){
                double param = kalmanFilterParams[i];
                if (i != int(kalmanFilterParams.size())-1){
                    std::cout << param << ", ";
                }
                else{
                    std::cout << param;
                }
            }
            std::cout << "]." << std::endl;
        }

        // num of frames used in KF for observation
        if (!this->nh_->get_parameter(pname("kalman_filter_averaging_frames"), this->kfAvgFrames_)){
            this->kfAvgFrames_ = 10;
            std::cout << this->hint_ << ": No number of frames used in KF for observation parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Number of frames used in KF for observation is set to: " << this->kfAvgFrames_ << std::endl;
        }

        // num of max missed frames for KF matching
        if (!this->nh_->get_parameter(pname("max_missed_frames"), this->maxMissedFrames_)){
            this->maxMissedFrames_ = 5;
            std::cout << this->hint_ << ": No number of max missed frames for KF matching parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Number of frames used in KF for observation is set to: " << this->kfAvgFrames_ << std::endl;
        }
        
        // weights for matching
        if (!this->nh_->get_parameter(pname("match_feat_weight"), this->matchFeatWeight_)){
            this->matchFeatWeight_ = 1.0;
            std::cout << this->hint_ << ": No match feature weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match feature weight is set to: " << this->matchFeatWeight_ << std::endl;
        }

        // weight for position difference in matching
        if (!this->nh_->get_parameter(pname("match_pos_weight"), this->matchPosWeight_)){
            this->matchPosWeight_ = 1.5;
            std::cout << this->hint_ << ": No match position weight parameter found. Use default: 1.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match position weight is set to: " << this->matchPosWeight_ << std::endl;
        }

        // weight for point cloud difference in matching
        if (!this->nh_->get_parameter(pname("match_pc_weight"), this->matchPcWeight_)){
            this->matchPcWeight_ = 1.0;
            std::cout << this->hint_ << ": No match point cloud weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match point cloud weight is set to: " << this->matchPcWeight_ << std::endl;
        }

        // weight for size difference in matching
        if (!this->nh_->get_parameter(pname("match_size_weight"), this->matchSizeWeight_)){
            this->matchSizeWeight_ = 1.0;
            std::cout << this->hint_ << ": No match size weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match size weight is set to: " << this->matchSizeWeight_ << std::endl;
        }

        // weight for IOU in matching
        if (!this->nh_->get_parameter(pname("match_iou_weight"), this->matchIouWeight_)){
            this->matchIouWeight_ = 1.0;
            std::cout << this->hint_ << ": No match IOU weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match IOU weight is set to: " << this->matchIouWeight_ << std::endl;
        }

        // minimum match score for matching
        if (!this->nh_->get_parameter(pname("min_match_score"), this->minMatchScore_)){
            this->minMatchScore_ = -2.5;
            std::cout << this->hint_ << ": No minimum match score parameter found. Use default: -2.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Minimum match score is set to: " << this->minMatchScore_ << std::endl;
        }

        // weight for velocity difference in matching
        if (!this->nh_->get_parameter(pname("match_vel_weight"), this->matchVelWeight_)){
            this->matchVelWeight_ = 1.0;
            std::cout << this->hint_ << ": No match velocity weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match velocity weight is set to: " << this->matchVelWeight_ << std::endl;
        }

        // thresholds for new track initialization
        if (!this->nh_->get_parameter(pname("new_track_min_dist"), this->newTrackMinDist_)){
            this->newTrackMinDist_ = 0.8;
            std::cout << this->hint_ << ": No minimum distance for new track parameter found. Use default: 0.8." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Minimum distance for new track is set to: " << this->newTrackMinDist_ << std::endl;
        }

        // minimum point cloud distance for new track initialization
        if (!this->nh_->get_parameter(pname("new_track_min_pc_dist"), this->newTrackMinPcDist_)){
            this->newTrackMinPcDist_ = 1.0;
            std::cout << this->hint_ << ": No minimum point cloud distance for new track parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Minimum point cloud distance for new track is set to: " << this->newTrackMinPcDist_ << std::endl;
        }

        // maximum velocity difference for matching
        if (!this->nh_->get_parameter(pname("max_match_vel_diff"), this->maxMatchVelDiff_)){
            this->maxMatchVelDiff_ = 3.0;
            std::cout << this->hint_ << ": No maximum velocity difference for matching parameter found. Use default: 3.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Maximum velocity difference for matching is set to: " << this->maxMatchVelDiff_ << std::endl;
        }

        // weight for cluster consistency in matching
        if (!this->nh_->get_parameter(pname("match_cluster_weight"), this->matchClusterWeight_)){
            this->matchClusterWeight_ = 1.0;
            std::cout << this->hint_ << ": No match cluster weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Match cluster weight is set to: " << this->matchClusterWeight_ << std::endl;
        }   

        // skip frame for classification
        if (!this->nh_->get_parameter(pname("frame_skip"), this->skipFrame_)){
            this->skipFrame_ = 5;
            std::cout << this->hint_ << ": No skip frame parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Frames skiped in classification when comparing two point cloud is set to: " << this->skipFrame_ << std::endl;
        }

        // velocity threshold for dynamic classification
        if (!this->nh_->get_parameter(pname("dynamic_velocity_threshold"), this->dynaVelThresh_)){
            this->dynaVelThresh_ = 0.35;
            std::cout << this->hint_ << ": No dynamic velocity threshold parameter found. Use default: 0.35." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Velocity threshold for dynamic classification is set to: " << this->dynaVelThresh_ << std::endl;
        }

        // voting threshold for dynamic classification
        if (!this->nh_->get_parameter(pname("dynamic_voting_threshold"), this->dynaVoteThresh_)){
            this->dynaVoteThresh_ = 0.8;
            std::cout << this->hint_ << ": No dynamic voting threshold parameter found. Use default: 0.8." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Voting threshold for dynamic classification is set to: " << this->dynaVoteThresh_ << std::endl;
        }

        // frames to force dynamic
        if (!this->nh_->get_parameter(pname("frames_force_dynamic"), this->forceDynaFrames_)){
            this->forceDynaFrames_ = 20;
            std::cout << this->hint_ << ": No range of searching dynamic obstacles in box history found. Use default: 20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Range of searching dynamic obstacles in box history is set to: " << this->forceDynaFrames_ << std::endl;
        }

        if (!this->nh_->get_parameter(pname("frames_force_dynamic_check_range"), this->forceDynaCheckRange_)){
            this->forceDynaCheckRange_ = 30;
            std::cout << this->hint_ << ": No threshold for forcing dynamic obstacles found. Use default: 30." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for forcing dynamic obstacles is set to: " << this->forceDynaCheckRange_ << std::endl;
        }

        // dynamic consistency check
        if (!this->nh_->get_parameter(pname("dynamic_consistency_threshold"), this->dynamicConsistThresh_)){
            this->dynamicConsistThresh_ = 3;
            std::cout << this->hint_ << ": No threshold for dynamic-consistency check found. Use default: 3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for dynamic consistency check is set to: " << this->dynamicConsistThresh_ << std::endl;
        }

        if ( this->histSize_ < this->forceDynaCheckRange_+1){
            RCLCPP_ERROR(this->nh_->get_logger(), "history length is too short to perform force-dynamic");
        }

        // constrain target object size
        if (!this->nh_->get_parameter(pname("target_constrain_size"), this->constrainSize_)){
            this->constrainSize_ = false;
            std::cout << this->hint_ << ": No target object constrain size param found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Target object constrain is set to: " << this->constrainSize_ << std::endl;
        }

        // target object  sizes
        std::vector<double> targetObjectSizeTemp;
        if (!this->nh_->get_parameter(pname("target_object_size"), targetObjectSizeTemp)){
            std::cout << this->hint_ << ": No target object size found. Do not apply target object size." << std::endl;
        }
        else{
            for (size_t i=0; i<targetObjectSizeTemp.size(); i+=3){
                Eigen::Vector3d targetSize (targetObjectSizeTemp[i+0], targetObjectSizeTemp[i+1], targetObjectSizeTemp[i+2]);
                this->targetObjectSize_.push_back(targetSize);
                std::cout << this->hint_ << ": target object size is set to: [" << targetObjectSizeTemp[i+0]  << ", " 
                << targetObjectSizeTemp[i+1] << ", " <<  targetObjectSizeTemp[i+2] << "]." << std::endl;
            }

        }

        // max object size
        std::vector<double> maxObjectSizeTemp;
        if(!this->nh_->get_parameter(pname("max_object_size"), maxObjectSizeTemp)){
            this->maxObjectSize_ = Eigen::Vector3d (2.0, 2.0, 2.0);
            std::cout << this->hint_ << ": No max object size threshold parameter found. Use default: [2.0, 2.0, 2.0]." << std::endl;
        }
        else{
            this->maxObjectSize_(0) = maxObjectSizeTemp[0];
            this->maxObjectSize_(1) = maxObjectSizeTemp[1];
            this->maxObjectSize_(2) = maxObjectSizeTemp[2];
            std::cout <<  this->hint_ << ": Max object size threshold is set to: [";
            for (size_t i = 0; i < maxObjectSizeTemp.size(); ++i) {
                std::cout << maxObjectSizeTemp[i];
                if (i != maxObjectSizeTemp.size()-1){
                    std::cout << ", ";
                }
            }
            std::cout << "]." << std::endl;
        }

        // Duplicate track distance threshold
        if (!this->nh_->get_parameter(pname("duplicate_track_dist_threshold"), this->duplicateTrackDistThresh_)){
            this->duplicateTrackDistThresh_ = 0.5;
            std::cout << this->hint_ << ": No duplicate track distance threshold parameter found.   Use default: 0.5." << std::endl;        
        }
        else{
            std::cout << this->hint_ << ": Duplicate track distance threshold is set to: " << this->duplicateTrackDistThresh_ << std::endl;
        }

        // Duplicate track IOU threshold
        if (!this->nh_->get_parameter(pname("duplicate_track_iou_threshold"), this->duplicateTrackIou2DThresh_)){
            this->duplicateTrackIou2DThresh_ = 0.3;
            std::cout << this->hint_ << ": No duplicate track IOU threshold parameter found. Use default: 0.3." << std::endl;        
        }
        else{
            std::cout << this->hint_ << ": Duplicate track IOU threshold is set to: " << this->duplicateTrackIou2DThresh_ << std::endl;
        }

        // Duplicate Size Rel Threshold
        if (!this->nh_->get_parameter(pname("duplicate_size_rel_threshold"), this->duplicateSizeRelThresh_)){
            this->duplicateSizeRelThresh_ = 0;
            std::cout << this->hint_ << ": No duplicate size relative threshold parameter found. Use default: 0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Duplicate size relative threshold is set to: " << this->duplicateSizeRelThresh_ << std::endl;
        }

        // Shape Score Weight:
        if (!this->nh_->get_parameter(pname("shape_score_weight"), this->shapeScoreWeight_)){
            this->shapeScoreWeight_ = 0.2;
            std::cout << this->hint_ << ": No shape score weight parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Shape score weight is set to: " << this->shapeScoreWeight_ << std::endl;
        }

        // Enable Tracking Debug Logs
        if (!this->nh_->get_parameter(pname("enable_tracking_debug_logs"), this->enableTrackingDebugLogs_)){
            this->enableTrackingDebugLogs_ = false;
            std::cout << this->hint_ << ": No enable tracking debug logs parameter found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Enable tracking debug logs is set to: " << (this->enableTrackingDebugLogs_ ? "true" : "false") << std::endl;
        }

    }

    bool dynamicDetector::lookupTfMatrix(const std::string& targetFrame, const std::string& sourceFrame, Eigen::Matrix4d& transformMatrix){
        if (!this->tfBuffer_) {
            return false;
        }

        try {
            geometry_msgs::msg::TransformStamped tfStamped =
                this->tfBuffer_->lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);

            const auto& t = tfStamped.transform.translation;
            const auto& q = tfStamped.transform.rotation;

            Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
            Eigen::Matrix3d rot = quat.toRotationMatrix();

            transformMatrix.setIdentity();
            transformMatrix.block<3, 3>(0, 0) = rot;
            transformMatrix(0, 3) = t.x;
            transformMatrix(1, 3) = t.y;
            transformMatrix(2, 3) = t.z;

            return true;
        }
        catch (const tf2::TransformException&) {
            return false;
        }
    }

    void dynamicDetector::registerPub(){
        // uv detector depth map pub
        this->uvDepthMapPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/detected_depth_map" : this->ns_ + "/detected_depth_map", 10);

        // uv detector u depth map pub
        this->uDepthMapPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/detected_u_depth_map" : this->ns_ + "/detected_u_depth_map", 10);

        // uv detector bird view pub
        this->uvBirdViewPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/u_depth_bird_view" : this->ns_ + "/u_depth_bird_view", 10);

        // color 2D bounding boxes pub
        this->detectedColorImgPub_ = this->nh_->create_publisher<sensor_msgs::msg::Image>(
            this->ns_.empty() ? "/detected_color_image" : this->ns_ + "/detected_color_image", 10);

        // uv detector bounding box pub
        this->uvBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/uv_bboxes" : this->ns_ + "/uv_bboxes", 10);

        // uv detector filtered bounding box pub
        this->uvBBoxesFilteredPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/uv_bboxes_filtered" : this->ns_ + "/uv_bboxes_filtered", 10);

        // DBSCAN bounding box pub
        this->dbBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/dbscan_bboxes" : this->ns_ + "/dbscan_bboxes", 10);
        
        //DBSCAN filtered bounding box pub
        this->dbBBoxesFilteredPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/dbscan_bboxes_filtered" : this->ns_ + "/dbscan_bboxes_filtered", 10);

        // visual bboxes pub
        this->visualBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/visual_bboxes" : this->ns_ + "/visual_bboxes", 10);

        // lidar bbox pub
        this->lidarBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/lidar_bboxes" : this->ns_ + "/lidar_bboxes", 10);

        // lidar filtered bounding box pub
        this->lidarBBoxesFilteredPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/lidar_bboxes_filtered" : this->ns_ + "/lidar_bboxes_filtered", 10);

        // filtered bounding box before YOLO pub
        this->filteredBBoxesBeforeYoloPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/filtered_before_yolo_bboxes" : this->ns_ + "/filtered_before_yolo_bboxes", 10);

        // filtered bounding box pub
        this->filteredBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/filtered_bboxes" : this->ns_ + "/filtered_bboxes", 10);

        // tracked bounding box pub
        this->trackedBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/tracked_bboxes" : this->ns_ + "/tracked_bboxes", 10);

        // dynamic bounding box pub
        this->dynamicBBoxesPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->ns_.empty() ? "/dynamic_bboxes" : this->ns_ + "/dynamic_bboxes", 10);

        // filtered depth pointcloud pub
        this->filteredDepthPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/filtered_depth_cloud" : this->ns_ + "/filtered_depth_cloud", 10);

        // lidar cluster pub
        this->lidarClustersPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/lidar_clusters" : this->ns_ + "/lidar_clusters", 10);

        // filtered pointcloud pub
        this->filteredPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/filtered_point_cloud" : this->ns_ + "/filtered_point_cloud", 10);

        // dynamic pointcloud pub
        this->dynamicPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/dynamic_point_cloud" : this->ns_ + "/dynamic_point_cloud", 10);

        // raw dynamic pointcloud pub
        this->rawDynamicPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/raw_dynamic_point_cloud" : this->ns_ + "/raw_dynamic_point_cloud", 10);

        // downsample points visualization pub
        this->downSamplePointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/downsampled_point_cloud" : this->ns_ + "/downsampled_point_cloud", 10);

        // raw LiDAR points visualization pub
        this->rawLidarPointsPub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/raw_lidar_point_cloud" : this->ns_ + "/raw_lidar_point_cloud", 10);

        // history trajectory pub
        this->historyTrajPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(this->ns_.empty() ? "/history_trajectories" : this->ns_ + "/history_trajectories", 10);

        // velocity visualization pub
        this->velVisPub_ = this->nh_->create_publisher<visualization_msgs::msg::MarkerArray>(this->ns_.empty() ? "/velocity_visualizaton" : this->ns_ + "/velocity_visualizaton", 10);
        
        // filtered lidar points in velodyne frame pub
        this->filteredLidarVelodynePub_ = this->nh_->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_.empty() ? "/filtered_lidar_points_velodyne_frame" : this->ns_ + "/filtered_lidar_points_velodyne_frame", 10);
    }   

    void dynamicDetector::registerCallback(){
        // depth pose callback
        this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(this->nh_, this->depthTopicName_));
        this->lidarCloudSub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this->nh_, this->lidarTopicName_));
        if (this->localizationMode_ == 0){
            this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::msg::PoseStamped>(this->nh_, this->poseTopicName_));
            // ApproximateTime with large queue for rosbag playback
            this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(1000), *this->depthSub_, *this->poseSub_));
            this->depthPoseSync_->registerCallback(std::bind(&dynamicDetector::depthPoseCB, this, std::placeholders::_1, std::placeholders::_2));
            
            this->lidarPoseSync_.reset(new message_filters::Synchronizer<lidarPoseSync>(lidarPoseSync(1000), *this->lidarCloudSub_, *this->poseSub_));
            this->lidarPoseSync_->registerCallback(std::bind(&dynamicDetector::lidarPoseCB, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: POSE mode - ApproximateTime sync (queue=1000)");
        }
        else if (this->localizationMode_ == 1){
            this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this->nh_, this->odomTopicName_));
            // ApproximateTime with large queue for rosbag playback
            this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(1000), *this->depthSub_, *this->odomSub_));
            this->depthOdomSync_->registerCallback(std::bind(&dynamicDetector::depthOdomCB, this, std::placeholders::_1, std::placeholders::_2));
            
            this->lidarOdomSync_.reset(new message_filters::Synchronizer<lidarOdomSync>(lidarOdomSync(1000), *this->lidarCloudSub_, *this->odomSub_));
            this->lidarOdomSync_->registerCallback(std::bind(&dynamicDetector::lidarOdomCB, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->nh_->get_logger(), "[dynamicDetector]: ODOMETRY mode - ApproximateTime sync (queue=1000)");
        }
        else{
            RCLCPP_ERROR(this->nh_->get_logger(), "[dynamicDetector]: Invalid localization mode!");
            exit(0);
        }

        // color image subscriber
        this->colorImgSub_ = this->nh_->create_subscription<sensor_msgs::msg::Image>(
            this->colorImgTopicName_, 10,
            std::bind(&dynamicDetector::colorImgCB, this, std::placeholders::_1));

        // yolo detection results subscriber
        this->yoloDetectionSub_ = this->nh_->create_subscription<vision_msgs::msg::Detection2DArray>(
            "yolo_detector/detected_bounding_boxes", 10,
            std::bind(&dynamicDetector::yoloDetectionCB, this, std::placeholders::_1));

        // detection timer
        this->detectionTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::detectionCB, this));

        // lidar detection timer
        this->lidarDetectionTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::lidarDetectionCB, this));

        // tracking timer
        this->trackingTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::trackingCB, this));

        // classification timer
        this->classificationTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::classificationCB, this));
    
        // visualization timer
        this->visTimer_ = this->nh_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ * 1000.0)),
            std::bind(&dynamicDetector::visCB, this));

        // get dynamic obstacle service
        this->getDynamicObstacleServer_ = this->nh_->create_service<onboard_detector::srv::GetDynamicObstacles>(
            "onboard_detector/get_dynamic_obstacles",
            [this](const std::shared_ptr<onboard_detector::srv::GetDynamicObstacles::Request> req,
                   std::shared_ptr<onboard_detector::srv::GetDynamicObstacles::Response> res) -> void {
                this->getDynamicObstacles(req, res);
            });
    }

    bool dynamicDetector::getDynamicObstacles(const std::shared_ptr<onboard_detector::srv::GetDynamicObstacles::Request> req,
                                              std::shared_ptr<onboard_detector::srv::GetDynamicObstacles::Response> res) {
        // Get the current robot position
        Eigen::Vector3d currPos = Eigen::Vector3d (req->current_position.x, req->current_position.y, req->current_position.z);

        // Vector to store obstacles along with their distances
        std::vector<std::pair<double, onboardDetector::box3D>> obstaclesWithDistances;

        // Go through all obstacles and calculate distances
        for (const onboardDetector::box3D& bbox : this->dynamicBBoxes_) {
            Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
            Eigen::Vector3d diff = currPos - obsPos;
            diff(2) = 0.;
            double distance = diff.norm();
            if (distance <= req->range) {
                obstaclesWithDistances.push_back(std::make_pair(distance, bbox));
            }
        }

        // Sort obstacles by distance in ascending order
        std::sort(obstaclesWithDistances.begin(), obstaclesWithDistances.end(), 
                [](const std::pair<double, onboardDetector::box3D>& a, const std::pair<double, onboardDetector::box3D>& b) {
                    return a.first < b.first;
                });

        // Push sorted obstacles into the response
        for (const auto& item : obstaclesWithDistances) {
            const onboardDetector::box3D& bbox = item.second;

            geometry_msgs::msg::Vector3 pos;
            geometry_msgs::msg::Vector3 vel;
            geometry_msgs::msg::Vector3 size;

            pos.x = bbox.x;
            pos.y = bbox.y;
            pos.z = bbox.z;

            vel.x = bbox.Vx;
            vel.y = bbox.Vy;
            vel.z = 0.;

            size.x = bbox.x_width;
            size.y = bbox.y_width;
            size.z = bbox.z_width;

            res->position.push_back(pos);
            res->velocity.push_back(vel);
            res->size.push_back(size);
        }

        return true;
    }

    void dynamicDetector::depthPoseCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose){
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseDepthMatrix, camPoseColorMatrix;
        this->getCameraPose(pose, camPoseDepthMatrix, camPoseColorMatrix);

        this->position_(0) = pose->pose.position.x;
        this->position_(1) = pose->pose.position.y;
        this->position_(2) = pose->pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionDepth_(0) = camPoseDepthMatrix(0, 3);
        this->positionDepth_(1) = camPoseDepthMatrix(1, 3);
        this->positionDepth_(2) = camPoseDepthMatrix(2, 3);
        this->orientationDepth_ = camPoseDepthMatrix.block<3, 3>(0, 0);

        this->positionColor_(0) = camPoseColorMatrix(0, 3);
        this->positionColor_(1) = camPoseColorMatrix(1, 3);
        this->positionColor_(2) = camPoseColorMatrix(2, 3);
        this->orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::depthOdomCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const nav_msgs::msg::Odometry::ConstSharedPtr& odom){
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseDepthMatrix, camPoseColorMatrix;
        this->getCameraPose(odom, camPoseDepthMatrix, camPoseColorMatrix);

        this->position_(0) = odom->pose.pose.position.x;
        this->position_(1) = odom->pose.pose.position.y;
        this->position_(2) = odom->pose.pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionDepth_(0) = camPoseDepthMatrix(0, 3);
        this->positionDepth_(1) = camPoseDepthMatrix(1, 3);
        this->positionDepth_(2) = camPoseDepthMatrix(2, 3);
        this->orientationDepth_ = camPoseDepthMatrix.block<3, 3>(0, 0);

        this->positionColor_(0) = camPoseColorMatrix(0, 3);
        this->positionColor_(1) = camPoseColorMatrix(1, 3);
        this->positionColor_(2) = camPoseColorMatrix(2, 3);
        this->orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::lidarPoseCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose){
        // for visualization
        this->latestCloud_ = cloudMsg;

        // local cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *tempCloud);

        // filter and downsample pointcloud
        // Create a filtered cloud pointer to store intermediate results
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply a pass-through filter to limit points to the local sensor range in X, Y, and Z axes
        pcl::PassThrough<pcl::PointXYZ> pass;

        // Filter for X axis
        pass.setInputCloud(tempCloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-this->localLidarRange_.x(), this->localLidarRange_.x());
        pass.filter(*filteredCloud);

        // Filter for Y axis
        pass.setInputCloud(filteredCloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-this->localLidarRange_.y(), this->localLidarRange_.y());
        pass.filter(*filteredCloud);

        int sigma = this->gaussianDownSampleRate_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(new pcl::PointCloud<pcl::PointXYZ>());
        preTransformCloud->reserve(filteredCloud->size());

        for (pcl::PointXYZ &pt : filteredCloud->points) {
            double dist = pow(pow(pt.x, 2) + pow(pt.y, 2), 0.5);
            double p = std::exp(-(dist * dist) / (2 * sigma * sigma));

            double r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            if (r < p) {
                preTransformCloud->push_back(pt);
            }
        }

        // Publish preTransformCloud in velodyne frame
        sensor_msgs::msg::PointCloud2 preTransformCloudMsg;
        pcl::toROSMsg(*preTransformCloud, preTransformCloudMsg);
        preTransformCloudMsg.header.frame_id = this->tfLidarFrame_;
        preTransformCloudMsg.header.stamp = cloudMsg->header.stamp;
        this->filteredLidarVelodynePub_->publish(preTransformCloudMsg);

        // transform
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = this->orientationLidar_;
        transform.translation() = this->positionLidar_;

        // map cloud
        // Create an empty point cloud to store the transformed data
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply the transformation
        pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

        // filter roof and ground 
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(transformedCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(this->groundHeight_, this->roofHeight_);
        pass.filter(*groundRoofFilterCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud = groundRoofFilterCloud;
        // Create the VoxelGrid filter object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        // sor.setInputCloud(filteredCloud);
        sor.setInputCloud(groundRoofFilterCloud);

        // Set the leaf size (adjust to control the downsampling)
        sor.setLeafSize(0.1f, 0.1f, 0.1f); // Try different values based on your point cloud density

        // If the downsampled cloud has more than certain points, further increase the leaf size
        while (int(downsampledCloud->size()) > this->downSampleThresh_) {
            double leafSize = sor.getLeafSize().x() * 1.1f; // Increase the leaf size to reduce point count
            sor.setLeafSize(leafSize, leafSize, leafSize);
            sor.filter(*downsampledCloud);
        }

        this->lidarCloud_ = downsampledCloud;
        sensor_msgs::msg::PointCloud2 outputCloud;
        pcl::toROSMsg(*this->lidarCloud_, outputCloud); // Convert to ROS message
        outputCloud.header.frame_id = "map";    // Set appropriate frame ID
        this->downSamplePointsPub_->publish(outputCloud);

        // store current position and orientation
        Eigen::Matrix4d lidarPoseMatrix;
        this->getLidarPose(pose, lidarPoseMatrix);

        this->position_(0) = pose->pose.position.x;
        this->position_(1) = pose->pose.position.y;
        this->position_(2) = pose->pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionLidar_(0) = lidarPoseMatrix(0, 3);
        this->positionLidar_(1) = lidarPoseMatrix(1, 3);
        this->positionLidar_(2) = lidarPoseMatrix(2, 3);
        this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::lidarOdomCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom){
        // for visualization
        this->latestCloud_ = cloudMsg;

        // local cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *tempCloud);

        // filter and downsample pointcloud
        // Create a filtered cloud pointer to store intermediate results
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply a pass-through filter to limit points to the local sensor range in X, Y, and Z axes
        pcl::PassThrough<pcl::PointXYZ> pass;

        // Filter for X axis
        pass.setInputCloud(tempCloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-this->localLidarRange_.x(), this->localLidarRange_.x());
        pass.filter(*filteredCloud);

        // Filter for Y axis
        pass.setInputCloud(filteredCloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-this->localLidarRange_.y(), this->localLidarRange_.y());
        pass.filter(*filteredCloud);

        int sigma = this->gaussianDownSampleRate_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(new pcl::PointCloud<pcl::PointXYZ>());
        preTransformCloud->reserve(filteredCloud->size());

        for (pcl::PointXYZ &pt : filteredCloud->points) {
            double dist = pow(pow(pt.x, 2) + pow(pt.y, 2), 0.5);
            double p = std::exp(-(dist * dist) / (2 * sigma * sigma));

            double r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            if (r < p) {
                preTransformCloud->push_back(pt);
            }
        }

        // Publish preTransformCloud in velodyne frame
        sensor_msgs::msg::PointCloud2 preTransformCloudMsg;
        pcl::toROSMsg(*preTransformCloud, preTransformCloudMsg);
        preTransformCloudMsg.header.frame_id = this->tfLidarFrame_;
        preTransformCloudMsg.header.stamp = cloudMsg->header.stamp;
        this->filteredLidarVelodynePub_->publish(preTransformCloudMsg);

        // transform
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = this->orientationLidar_;
        transform.translation() = this->positionLidar_;

        // map cloud
        // Create an empty point cloud to store the transformed data
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());

        // Apply the transformation
        pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

        // filter roof and ground 
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(transformedCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(this->groundHeight_, this->roofHeight_);
        pass.filter(*groundRoofFilterCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud = groundRoofFilterCloud;
        // Create the VoxelGrid filter object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        // sor.setInputCloud(filteredCloud);
        sor.setInputCloud(groundRoofFilterCloud);

        // Set the leaf size (adjust to control the downsampling)
        sor.setLeafSize(0.1f, 0.1f, 0.1f); // Try different values based on your point cloud density

        // If the downsampled cloud has more than certain points, further increase the leaf size
        while (int(downsampledCloud->size()) > this->downSampleThresh_) {
            double leafSize = sor.getLeafSize().x() * 1.1f; // Increase the leaf size to reduce point count
            sor.setLeafSize(leafSize, leafSize, leafSize);
            sor.filter(*downsampledCloud);
        }

        this->lidarCloud_ = downsampledCloud;
        sensor_msgs::msg::PointCloud2 outputCloud;
        pcl::toROSMsg(*this->lidarCloud_, outputCloud); // Convert to ROS message
        outputCloud.header.frame_id = "map";    // Set appropriate frame ID
        this->downSamplePointsPub_->publish(outputCloud);
        
        // store current position and orientation
        Eigen::Matrix4d lidarPoseMatrix;
        this->getLidarPose(odom, lidarPoseMatrix);

        this->position_(0) = odom->pose.pose.position.x;
        this->position_(1) = odom->pose.pose.position.y;
        this->position_(2) = odom->pose.pose.position.z;
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();
        this->orientation_ = rot;

        this->positionLidar_(0) = lidarPoseMatrix(0, 3);
        this->positionLidar_(1) = lidarPoseMatrix(1, 3);
        this->positionLidar_(2) = lidarPoseMatrix(2, 3);
        this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::colorImgCB(const sensor_msgs::msg::Image::ConstSharedPtr& img){
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        imgPtr->image.copyTo(this->detectedColorImage_);
    }

    void dynamicDetector::yoloDetectionCB(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections){
        this->yoloDetectionResults_ = *detections;
    }

   
    void dynamicDetector::lidarDetectionCB(){
        this->lidarDetect();
    }

    void dynamicDetector::detectionCB(){
        // detection thread
        this->dbscanDetect();
        this->uvDetect();
        this->filterLVBBoxes();
        this->newDetectFlag_ = true; // get a new detection
    }

    void dynamicDetector::trackingCB(){
        std::vector<int> bestMatch;

        this->boxAssociation(bestMatch);

        if (!this->boxHist_.empty() || !this->filteredBBoxes_.empty()){
            this->kalmanFilterAndUpdateHist(bestMatch);
        }
        else{
            this->boxHist_.clear();
            this->pcHist_.clear();
            this->pcCenterHist_.clear();
            this->filters_.clear();
            this->missedFrames_.clear();
            this->trackedBBoxes_.clear();
        }
    }

    void dynamicDetector::classificationCB(){
        // Identification thread
        std::vector<onboardDetector::box3D> dynamicBBoxesTemp;

        // Iterate through all pointcloud/bounding boxes history (note that yolo's pointclouds are dummy pointcloud (empty))
        // NOTE: There are 3 cases which we don't need to perform dynamic obstacle identification.
        for (size_t i=0; i<this->pcHist_.size() ; ++i){
            // ===================================================================================
            // CASE I: yolo recognized as dynamic dynamic obstacle
            if (this->boxHist_[i][0].is_human){
                dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                continue;
            }
            // ===================================================================================


            // ===================================================================================
            // CASE II: history length is not enough to run classification
            int curFrameGap;
            if (int(this->pcHist_[i].size()) < this->skipFrame_+1){
                curFrameGap = this->pcHist_[i].size() - 1;
            }
            else{
                curFrameGap = this->skipFrame_;
            }
            // ===================================================================================


            // ==================================================================================
            // CASE III: Force Dynamic (if the obstacle is classifed as dynamic for several time steps)
            int dynaFrames = 0;
            if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_){
                for (int j=1 ; j<this->forceDynaCheckRange_+1 ; ++j){
                    if (this->boxHist_[i][j].is_dynamic){
                        ++dynaFrames;
                    }
                }
            }

            if (dynaFrames >= this->forceDynaFrames_){
                this->boxHist_[i][0].is_dynamic = true;
                dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                continue;
            }
            // ===================================================================================

            std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
            std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];
            Eigen::Vector3d Vcur(0.,0.,0.); // single point velocity 
            Eigen::Vector3d Vbox(0.,0.,0.); // bounding box velocity 
            Eigen::Vector3d Vkf(0.,0.,0.);  // velocity estimated from kalman filter
            int numPoints = currPc.size(); // it changes within loop
            int votes = 0;

            Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x)/(this->dt_*curFrameGap);
            Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y)/(this->dt_*curFrameGap);
            Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z)/(this->dt_*curFrameGap);
            Vkf(0) = this->boxHist_[i][0].Vx;
            Vkf(1) = this->boxHist_[i][0].Vy;

            // find nearest neighbor
            for (size_t j=0 ; j<currPc.size() ; ++j){
                double minDist = 2;
                Eigen::Vector3d nearestVect;
                for (size_t k=0 ; k<prevPc.size() ; k++){ // find the nearest point in the previous pointcloud
                    double dist = (currPc[j]-prevPc[k]).norm();
                    if (abs(dist) < minDist){
                        minDist = dist;
                        nearestVect = currPc[j]-prevPc[k];
                    }
                }
                Vcur = nearestVect/(this->dt_*curFrameGap); Vcur(2) = 0;
                double velSim = Vcur.dot(Vbox)/(Vcur.norm()*Vbox.norm());

                if (velSim < 0){
                    --numPoints;
                }
                else{
                    if (Vcur.norm()>this->dynaVelThresh_){
                        ++votes;
                    }
                }
            }
            
            
            // update dynamic boxes
            double voteRatio = (numPoints>0)?double(votes)/double(numPoints):0;
            double velNorm = Vkf.norm();

            // voting and velocity threshold
            // 1. point cloud voting ratio.
            // 2. velocity (from kalman filter) 
            if (voteRatio>=this->dynaVoteThresh_ && velNorm>=this->dynaVelThresh_){
                this->boxHist_[i][0].is_dynamic_candidate = true;
                // dynamic-consistency check
                int dynaConsistCount = 0;
                if (int(this->boxHist_[i].size()) >= this->dynamicConsistThresh_){
                    for (int j=0 ; j<this->dynamicConsistThresh_; ++j){
                        if (this->boxHist_[i][j].is_dynamic_candidate or this->boxHist_[i][j].is_human or this->boxHist_[i][j].is_dynamic){
                            ++dynaConsistCount;
                        }
                    }
                }            
                if (dynaConsistCount == this->dynamicConsistThresh_){
                    // set as dynamic and push into history
                    this->boxHist_[i][0].is_dynamic = true;
                    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);    
                }
            }
        }

        // filter the dynamic obstacles based on the target sizes
        if (this->constrainSize_){
            std::vector<onboardDetector::box3D> dynamicBBoxesBeforeConstrain = dynamicBBoxesTemp;
            dynamicBBoxesTemp.clear();

            for (onboardDetector::box3D ob : dynamicBBoxesBeforeConstrain){
                bool findMatch = false;
                for (Eigen::Vector3d targetSize : this->targetObjectSize_){
                    double xdiff = std::abs(ob.x_width - targetSize(0));
                    double ydiff = std::abs(ob.y_width - targetSize(1));
                    double zdiff = std::abs(ob.z_width - targetSize(2)); 
                    if (xdiff < 0.8 and ydiff < 0.8 and zdiff < 1.0){
                        findMatch = true;
                    }
                }

                if (findMatch){
                    dynamicBBoxesTemp.push_back(ob);
                }
            }
        }

        this->dynamicBBoxes_ = dynamicBBoxesTemp;
    }

    void dynamicDetector::visCB(){
        this->publishUVImages();
        this->publishColorImages();
        
        // Publish 3D visualization markers for all bounding box stages
        this->publish3dBox(this->uvBBoxes_, this->uvBBoxesPub_, 0, 1, 0);
        //this->publish3dBox(this->uvBBoxesFiltered_, this->uvBBoxesFilteredPub_, 0, 1, 0.5);
        this->publish3dBox(this->dbBBoxes_, this->dbBBoxesPub_, 1, 0, 0);
        //this->publish3dBox(this->dbBBoxesFiltered_, this->dbBBoxesFilteredPub_, 1, 0.5, 0);
        this->publish3dBox(this->visualBBoxes_, this->visualBBoxesPub_, 0.3, 0.8, 1.0);
        this->publish3dBox(this->lidarBBoxes_, this->lidarBBoxesPub_, 0.5, 0.5, 0.5);
        //this->publish3dBox(this->lidarBBoxesFiltered_, this->lidarBBoxesFilteredPub_, 0.5, 0, 0.5);
        this->publish3dBox(this->filteredBBoxesBeforeYolo_, this->filteredBBoxesBeforeYoloPub_, 0, 1, 0.5);
        this->publish3dBox(this->filteredBBoxes_, this->filteredBBoxesPub_, 0, 1, 1);
        this->publish3dBox(this->trackedBBoxes_, this->trackedBBoxesPub_, 1, 1, 0);
        // Publish track IDs as text markers above each tracked bbox
        visualization_msgs::msg::MarkerArray trackIdMarkers;
        for (size_t i = 0; i < this->trackedBBoxes_.size(); ++i) {
            visualization_msgs::msg::Marker idMarker;
            idMarker.header.frame_id = "map";
            idMarker.header.stamp = this->nh_->now();
            idMarker.ns = "track_id";
            idMarker.id = i;
            idMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            idMarker.action = visualization_msgs::msg::Marker::ADD;
            // Position marker just above the bbox, slightly offset
            idMarker.pose.position.x = this->trackedBBoxes_[i].x;
            idMarker.pose.position.y = this->trackedBBoxes_[i].y;
            idMarker.pose.position.z = this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width/2. + 0.05;
            // Make text smaller
            idMarker.scale.z = 0.15;
            idMarker.scale.x = 0.15;
            idMarker.scale.y = 0.15;
            // ID and velocity norm together
            // Only declare these variables once per marker
            idMarker.text = "ID: " + std::to_string(static_cast<int>(this->trackedBBoxes_[i].id)) + " | |V|: " + std::to_string(std::sqrt(this->trackedBBoxes_[i].Vx * this->trackedBBoxes_[i].Vx + this->trackedBBoxes_[i].Vy * this->trackedBBoxes_[i].Vy));
            idMarker.color.a = 1.0;
            idMarker.color.r = 0.0;
            idMarker.color.g = 1.0;
            idMarker.color.b = 0.0;
            // Show the track id (cast to int for display)
            int track_id = static_cast<int>(this->trackedBBoxes_[i].id);
            double vx = this->trackedBBoxes_[i].Vx;
            double vy = this->trackedBBoxes_[i].Vy;
            double vnorm = std::sqrt(vx*vx + vy*vy);
            idMarker.text = "ID: " + std::to_string(track_id) + " | |V|: " + std::to_string(vnorm);
            idMarker.lifetime = rclcpp::Duration::from_seconds(0.1);
            trackIdMarkers.markers.push_back(idMarker);
        }
        this->trackedBBoxesPub_->publish(trackIdMarkers);
        this->publish3dBox(this->dynamicBBoxes_, this->dynamicBBoxesPub_, 0, 0, 1);

        this->publishLidarClusters(); // colored clusters
        this->publishFilteredPoints();
        std::vector<Eigen::Vector3d> dynamicPoints;
        this->getDynamicPc(dynamicPoints);
        this->publishPoints(dynamicPoints, this->dynamicPointsPub_);
        this->publishPoints(this->filteredDepthPoints_, this->filteredDepthPointsPub_);
        this->publishRawDynamicPoints();

        this->publishHistoryTraj();
        this->publishVelVis();
    }

    void dynamicDetector::uvDetect(){
        // initialization
        if (this->uvDetector_ == NULL){
            this->uvDetector_.reset(new UVdetector ());
            this->uvDetector_->fx = this->fx_;
            this->uvDetector_->fy = this->fy_;
            this->uvDetector_->px = this->cx_;
            this->uvDetector_->py = this->cy_;
            this->uvDetector_->depthScale_ = this->depthScale_; 
            this->uvDetector_->max_dist = this->raycastMaxLength_ * 1000;
        }

        // detect from depth mapcalBox
        if (not this->depthImage_.empty()){
            this->uvDetector_->depth = this->depthImage_;
            this->uvDetector_->detect();
            this->uvDetector_->extract_3Dbox();

            this->uvDetector_->display_U_map();
            this->uvDetector_->display_bird_view();
            this->uvDetector_->display_depth();

            // transform to the world frame (recalculate the bounding boxes)
            std::vector<onboardDetector::box3D> uvBBoxes;
            this->transformUVBBoxes(uvBBoxes);
            this->uvBBoxes_ = uvBBoxes;
        }
    }

    void dynamicDetector::dbscanDetect(){
        // 1. get pointcloud
        this->projectDepthImage();

        // 2. filter points
        this->filterPoints(this->projPoints_, this->filteredDepthPoints_);

        // 3. cluster points and get bounding boxes
        this->clusterPointsAndBBoxes(this->filteredDepthPoints_, this->dbBBoxes_, this->pcClustersVisual_, 
                                     this->pcClusterCentersVisual_, this->pcClusterStdsVisual_);
    }


    void dynamicDetector::lidarDetect(){
        if (this->lidarDetector_ == nullptr){
            this->lidarDetector_.reset(new lidarDetector());
        }

        // aggiorna sempre i parametri, anche se il detector esiste già
        this->lidarDetector_->setParams(
            this->lidarDBEpsilon_,
            this->lidarDBMinPoints_,
            this->dbscanRefinementEnable_,
            this->dbscanRefineMaxDiagonal_,
            this->dbscanRefineMinDensity_,
            this->dbscanRefineSplitMinPts_,
            this->dbscanRefineSplitEps_,
            this->dbscanRefineMinSubclusterPts_,
            this->dbscanRefineAxisSliceWidth_,
            this->dbscanRefineMaxDepth_,
            this->dbscanRefineRecursive_,
            this->dbscanRefineMinBoxVolume_);

        if (this->lidarCloud_ != nullptr){
            this->lidarDetector_->getPointcloud(this->lidarCloud_);
            this->lidarDetector_->lidarDBSCAN();

            std::vector<onboardDetector::Cluster> lidarClustersRaw = this->lidarDetector_->getClusters();
            std::vector<onboardDetector::Cluster> lidarClustersFiltered;
            std::vector<onboardDetector::box3D> lidarBBoxesRaw = this->lidarDetector_->getBBoxes();
            std::vector<onboardDetector::box3D> lidarBBoxesFiltered;

            for (size_t i = 0; i < lidarBBoxesRaw.size(); ++i){
                const onboardDetector::box3D& lidarBBox = lidarBBoxesRaw[i];

                if (lidarBBox.x_width > this->maxObjectSize_(0) ||
                    lidarBBox.y_width > this->maxObjectSize_(1) ||
                    lidarBBox.z_width > this->maxObjectSize_(2)){
                    continue;
                }

                lidarBBoxesFiltered.push_back(lidarBBox);
                lidarClustersFiltered.push_back(lidarClustersRaw[i]);
            }

            this->lidarBBoxes_ = lidarBBoxesFiltered;
            this->lidarClusters_ = lidarClustersFiltered;
        }
    }

    void dynamicDetector::BboxesMerger(
                                    const std::vector<onboardDetector::box3D>& group1BBoxes_,
                                    const std::vector<onboardDetector::box3D>& group2BBoxes_,

                                    const std::vector<std::vector<Eigen::Vector3d>>& group1pcClusters_,
                                    const std::vector<Eigen::Vector3d>& group1pcClusterCenters_,
                                    const std::vector<Eigen::Vector3d>& group1pcClusterStds_,

                                    const std::vector<std::vector<Eigen::Vector3d>>& group2pcClusters_,
                                    const std::vector<Eigen::Vector3d>& group2pcClusterCenters_,
                                    const std::vector<Eigen::Vector3d>& group2pcClusterStds_,

                                    std::vector<onboardDetector::box3D>& BBoxesTemp,
                                    std::vector<std::vector<Eigen::Vector3d>>& PcClustersTemp,
                                    std::vector<Eigen::Vector3d>& PcClusterCentersTemp,
                                    std::vector<Eigen::Vector3d>& PcClusterStdsTemp,

                                    std::string merging_style,
                                    bool flag_group1,
                                    bool flag_group2,
                                    double boxIOUThresh_,
                                    double boxIOVThresh_)
    {
        size_t M = group1BBoxes_.size();
        size_t N = group2BBoxes_.size();

        Eigen::MatrixXd IOU = Eigen::MatrixXd::Zero(M, N);
        Eigen::MatrixXd IOV_g1 = Eigen::MatrixXd::Zero(M, N);
        Eigen::MatrixXd IOV_g2 = Eigen::MatrixXd::Zero(M, N);

        std::vector<bool> usedGroup1(M, false);
        std::vector<bool> usedGroup2(N, false);

        double eps = 1e-6;
        std::vector<onboardDetector::Edge> edges;

        // =========================
        // STEP 0: MATRICES
        // =========================
        for (size_t i = 0; i < M; ++i)
        {
            for (size_t j = 0; j < N; ++j)
            {
                IOU(i,j) = this->calBoxIOU(group1BBoxes_[i], group2BBoxes_[j]);
                IOV_g1(i,j) = this->calBoxIOV(group1BBoxes_[i], group2BBoxes_[j]);
                IOV_g2(i,j) = this->calBoxIOV(group2BBoxes_[j], group1BBoxes_[i]);
            }
        }

        // =========================
        // STEP 1: MUTUAL IOU MATCH
        // =========================
        for (size_t i = 0; i < M; ++i)
        {
            if (usedGroup1[i]) continue;

            double bestIOU = -1;
            int best_j = -1;

            for (size_t j = 0; j < N; ++j)
            {
                if (usedGroup2[j]) continue;

                if (IOU(i,j) > bestIOU)
                {
                    bestIOU = IOU(i,j);
                    best_j = j;
                }
            }

            if (best_j == -1) continue;

            double bestIOU_back = -1;
            int best_i = -1;

            for (size_t k = 0; k < M; ++k)
            {
                if (usedGroup1[k]) continue;

                if (IOU(k, best_j) > bestIOU_back)
                {
                    bestIOU_back = IOU(k, best_j);
                    best_i = k;
                }
            }

            if (best_i == (int)i &&
                bestIOU > boxIOUThresh_ &&
                bestIOU_back > boxIOUThresh_)
            {
                auto b1 = group1BBoxes_[i];
                auto b2 = group2BBoxes_[best_j];

                double xmax = std::max(b1.x+b1.x_width/2, b2.x+b2.x_width/2);
                double xmin = std::min(b1.x-b1.x_width/2, b2.x-b2.x_width/2);
                double ymax = std::max(b1.y+b1.y_width/2, b2.y+b2.y_width/2);
                double ymin = std::min(b1.y-b1.y_width/2, b2.y-b2.y_width/2);
                double zmax = std::max(b1.z+b1.z_width/2, b2.z+b2.z_width/2);
                double zmin = std::min(b1.z-b1.z_width/2, b2.z-b2.z_width/2);

                onboardDetector::box3D bbox;
                bbox.x = (xmin+xmax)/2;
                bbox.y = (ymin+ymax)/2;
                bbox.z = (zmin+zmax)/2;
                bbox.x_width = xmax-xmin;
                bbox.y_width = ymax-ymin;
                bbox.z_width = zmax-zmin;
                bbox.Vx = 0;
                bbox.Vy = 0;

                // 👉 CLUSTER = group2 (come codice originale!)
                auto cluster = group2pcClusters_[best_j];
                auto center  = group2pcClusterCenters_[best_j];
                auto stddev  = group2pcClusterStds_[best_j];

                BBoxesTemp.push_back(bbox);
                PcClustersTemp.push_back(cluster);
                PcClusterCentersTemp.push_back(center);
                PcClusterStdsTemp.push_back(stddev);

                usedGroup1[i] = true;
                usedGroup2[best_j] = true;
            }
        }

        // =========================
        // STEP 2: BUILD GRAPH (IOV)
        // =========================
        for (size_t i = 0; i < M; ++i)
        {
            if (usedGroup1[i]) continue;

            for (size_t j = 0; j < N; ++j)
            {
                if (usedGroup2[j]) continue;

                double iov1 = IOV_g1(i,j);
                double iov2 = IOV_g2(i,j);

                if (iov1 < eps && iov2 < eps) continue;

                onboardDetector::Edge e;

                if (iov1 > iov2)
                {
                    e.parent = { (int)j, false };
                    e.child  = { (int)i, true };
                    e.weight = iov1;
                }
                else
                {
                    e.parent = { (int)i, true };
                    e.child  = { (int)j, false };
                    e.weight = iov2;
                }

                if (e.weight >= boxIOVThresh_)
                    edges.push_back(e);
            }
        }

        // =========================
        // STEP 3: BEST PARENT
        // =========================
        std::map<std::pair<int,bool>, onboardDetector::Edge> bestParent;

        for (const auto& e : edges)
        {
            auto key = std::make_pair(e.child.idx, e.child.is_group1);

            if (bestParent.find(key) == bestParent.end() ||
                e.weight > bestParent[key].weight)
            {
                bestParent[key] = e;
            }
        }

        // =========================
        // STEP 4: GRAPH
        // =========================
        std::map<std::pair<int,bool>, std::vector<onboardDetector::Node>> childrenMap;
        std::set<std::pair<int,bool>> allNodes, childNodes;

        for (const auto& kv : bestParent)
        {
            const auto& e = kv.second;

            auto p = std::make_pair(e.parent.idx, e.parent.is_group1);
            auto c = std::make_pair(e.child.idx, e.child.is_group1);

            childrenMap[p].push_back(e.child);

            allNodes.insert(p);
            allNodes.insert(c);
            childNodes.insert(c);
        }
        // =========================
        // STEP X: MARK ALL GRAPH NODES AS USED
        // =========================
        for (const auto& k : allNodes)
        {
            if (k.second)  // is_group1 == true
                usedGroup1[k.first] = true;
            else           // group2
                usedGroup2[k.first] = true;
        }
        // =========================
        // 🔥 FIX CRITICO: SMALLER = LEAF ONLY
        // ==========================
        if (merging_style == "smaller")
        {
            for (const auto& k : allNodes)
            {
                // skip non-leaf
                if (childrenMap.find(k) != childrenMap.end()) continue;

                onboardDetector::Node leaf{k.first, k.second};

                // =========================
                // 🔥 TROVA IL PARENT (se esiste)
                // =========================
                bool hasParent = false;
                onboardDetector::Node parentNode;

                for (const auto& kv : bestParent)
                {
                    if (kv.first.first == leaf.idx &&
                        kv.first.second == leaf.is_group1)
                    {
                        parentNode = kv.second.parent;
                        hasParent = true;
                        break;
                    }
                }

                // =========================
                // 🔥 CASO CONSERVATIVO: 1 parent - 1 child
                // =========================
                if (hasParent)
                {
                    auto parentKey = std::make_pair(parentNode.idx, parentNode.is_group1);

                    // parent ha SOLO questo figlio?
                    if (childrenMap[parentKey].size() == 1)
                    {
                        // 👉 MERGE padre + figlio
                        onboardDetector::box3D b1, b2;
                        std::vector<Eigen::Vector3d> cluster;

                        // leaf
                        if (leaf.is_group1)
                        {
                            b1 = group1BBoxes_[leaf.idx];
                            cluster.insert(cluster.end(),
                                group1pcClusters_[leaf.idx].begin(),
                                group1pcClusters_[leaf.idx].end());
                        }
                        else
                        {
                            b1 = group2BBoxes_[leaf.idx];
                            cluster.insert(cluster.end(),
                                group2pcClusters_[leaf.idx].begin(),
                                group2pcClusters_[leaf.idx].end());
                        }

                        // parent
                        if (parentNode.is_group1)
                        {
                            b2 = group1BBoxes_[parentNode.idx];
                            cluster.insert(cluster.end(),
                                group1pcClusters_[parentNode.idx].begin(),
                                group1pcClusters_[parentNode.idx].end());
                        }
                        else
                        {
                            b2 = group2BBoxes_[parentNode.idx];
                            cluster.insert(cluster.end(),
                                group2pcClusters_[parentNode.idx].begin(),
                                group2pcClusters_[parentNode.idx].end());
                        }

                        double xmax = std::max(b1.x+b1.x_width/2, b2.x+b2.x_width/2);
                        double xmin = std::min(b1.x-b1.x_width/2, b2.x-b2.x_width/2);
                        double ymax = std::max(b1.y+b1.y_width/2, b2.y+b2.y_width/2);
                        double ymin = std::min(b1.y-b1.y_width/2, b2.y-b2.y_width/2);
                        double zmax = std::max(b1.z+b1.z_width/2, b2.z+b2.z_width/2);
                        double zmin = std::min(b1.z-b1.z_width/2, b2.z-b2.z_width/2);

                        onboardDetector::box3D bbox;
                        bbox.x = (xmin+xmax)/2;
                        bbox.y = (ymin+ymax)/2;
                        bbox.z = (zmin+zmax)/2;
                        bbox.x_width = xmax-xmin;
                        bbox.y_width = ymax-ymin;
                        bbox.z_width = zmax-zmin;
                        bbox.Vx = 0;
                        bbox.Vy = 0;

                        // center + std
                        Eigen::Vector3d center = Eigen::Vector3d::Zero();
                        for (auto& p : cluster) center += p;
                        if (!cluster.empty()) center /= cluster.size();

                        Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                        for (auto& p : cluster)
                            stddev += (p - center).cwiseAbs2();
                        if (!cluster.empty())
                            stddev = (stddev / cluster.size()).cwiseSqrt();

                        BBoxesTemp.push_back(bbox);
                        PcClustersTemp.push_back(cluster);
                        PcClusterCentersTemp.push_back(center);
                        PcClusterStdsTemp.push_back(stddev);

                        continue;
                    }
                }

                // =========================
                // DEFAULT: SOLO LEAF
                // =========================
                onboardDetector::box3D bbox;
                std::vector<Eigen::Vector3d> cluster;
                Eigen::Vector3d center, stddev;

                if (leaf.is_group1)
                {
                    bbox = group1BBoxes_[leaf.idx];
                    cluster = group1pcClusters_[leaf.idx];
                    center  = group1pcClusterCenters_[leaf.idx];
                    stddev  = group1pcClusterStds_[leaf.idx];
                }
                else
                {
                    bbox = group2BBoxes_[leaf.idx];
                    cluster = group2pcClusters_[leaf.idx];
                    center  = group2pcClusterCenters_[leaf.idx];
                    stddev  = group2pcClusterStds_[leaf.idx];
                }

                BBoxesTemp.push_back(bbox);
                PcClustersTemp.push_back(cluster);
                PcClusterCentersTemp.push_back(center);
                PcClusterStdsTemp.push_back(stddev);
            }
        }
        else if (merging_style == "bigger")
        {
            // DFS per visitare il sottoalbero
            std::function<void(const onboardDetector::Node&, std::vector<onboardDetector::Node>&)> dfs =
            [&](const onboardDetector::Node& n, std::vector<onboardDetector::Node>& out)
            {
                out.push_back(n);

                auto key = std::make_pair(n.idx, n.is_group1);
                if (childrenMap.find(key) == childrenMap.end()) return;

                for (const auto& c : childrenMap[key])
                    dfs(c, out);
            };

            // trova le root (nodi che non sono figli)
            std::set<std::pair<int,bool>> childNodes;
            for (const auto& kv : childrenMap)
            {
                for (const auto& c : kv.second)
                    childNodes.insert({c.idx, c.is_group1});
            }

            std::vector<onboardDetector::Node> roots;
            for (const auto& k : allNodes)
            {
                if (childNodes.find(k) == childNodes.end())
                    roots.push_back({k.first, k.second});
            }

            // MERGE PER OGNI ALBERO
            for (const auto& root : roots)
            {
                std::vector<onboardDetector::Node> subtree;
                dfs(root, subtree);

                double xmin =  std::numeric_limits<double>::max();
                double ymin =  std::numeric_limits<double>::max();
                double zmin =  std::numeric_limits<double>::max();

                double xmax = -std::numeric_limits<double>::max();
                double ymax = -std::numeric_limits<double>::max();
                double zmax = -std::numeric_limits<double>::max();

                std::vector<Eigen::Vector3d> mergedCluster;

                for (const auto& n : subtree)
                {
                    onboardDetector::box3D b;

                    // 👉 SCELTA CORRETTA DELLA SORGENTE (FIX PRINCIPALE)
                    if (n.is_group1)
                    {
                        b = group1BBoxes_[n.idx];
                        mergedCluster.insert(mergedCluster.end(),
                                            group1pcClusters_[n.idx].begin(),
                                            group1pcClusters_[n.idx].end());
                    }
                    else
                    {
                        b = group2BBoxes_[n.idx];
                        mergedCluster.insert(mergedCluster.end(),
                                            group2pcClusters_[n.idx].begin(),
                                            group2pcClusters_[n.idx].end());
                    }

                    xmax = std::max(xmax, b.x + b.x_width/2);
                    xmin = std::min(xmin, b.x - b.x_width/2);
                    ymax = std::max(ymax, b.y + b.y_width/2);
                    ymin = std::min(ymin, b.y - b.y_width/2);
                    zmax = std::max(zmax, b.z + b.z_width/2);
                    zmin = std::min(zmin, b.z - b.z_width/2);
                }

                // bounding box finale
                onboardDetector::box3D bbox;
                bbox.x = (xmin+xmax)/2;
                bbox.y = (ymin+ymax)/2;
                bbox.z = (zmin+zmax)/2;
                bbox.x_width = xmax-xmin;
                bbox.y_width = ymax-ymin;
                bbox.z_width = zmax-zmin;
                bbox.Vx = 0;
                bbox.Vy = 0;

                // calcolo feature cluster
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                for (const auto& p : mergedCluster) center += p;
                if (!mergedCluster.empty()) center /= mergedCluster.size();

                Eigen::Vector3d stddev = Eigen::Vector3d::Zero();
                for (const auto& p : mergedCluster)
                    stddev += (p - center).cwiseAbs2();
                if (!mergedCluster.empty())
                    stddev = (stddev / mergedCluster.size()).cwiseSqrt();

                BBoxesTemp.push_back(bbox);
                PcClustersTemp.push_back(mergedCluster);
                PcClusterCentersTemp.push_back(center);
                PcClusterStdsTemp.push_back(stddev);
            }
        }

        // =========================
        // ADD UNUSED
        // =========================
        if (flag_group1)
        {
            for (size_t i = 0; i < M; ++i)
            {
                if (!usedGroup1[i])
                {
                    BBoxesTemp.push_back(group1BBoxes_[i]);
                    PcClustersTemp.push_back(group1pcClusters_[i]);
                    PcClusterCentersTemp.push_back(group1pcClusterCenters_[i]);
                    PcClusterStdsTemp.push_back(group1pcClusterStds_[i]);
                }
            }
        }

        if (flag_group2)
        {
            for (size_t j = 0; j < N; ++j)
            {
                if (!usedGroup2[j])
                {
                    BBoxesTemp.push_back(group2BBoxes_[j]);
                    PcClustersTemp.push_back(group2pcClusters_[j]);
                    PcClusterCentersTemp.push_back(group2pcClusterCenters_[j]);
                    PcClusterStdsTemp.push_back(group2pcClusterStds_[j]);
                }
            }
        }
    }

    void dynamicDetector::mergeNestedGroup(
        const std::vector<onboardDetector::box3D>& inBoxes,
        const std::vector<std::vector<Eigen::Vector3d>>& inClusters,
        const std::vector<Eigen::Vector3d>& inCenters,
        const std::vector<Eigen::Vector3d>& inStds,

        std::vector<onboardDetector::box3D>& outBoxes,
        std::vector<std::vector<Eigen::Vector3d>>& outClusters,
        std::vector<Eigen::Vector3d>& outCenters,
        std::vector<Eigen::Vector3d>& outStds)
    {
        size_t N = inBoxes.size();
        if (N == 0) return;

        double iouThresh = this->samegroupIOUThresh_;
        double iovThresh = this->samegroupIOVThresh_;

        Eigen::MatrixXd IOU = Eigen::MatrixXd::Zero(N, N);

        // =========================
        // STEP 0: IOU MATRIX
        // =========================
        for (size_t i = 0; i < N; ++i)
        {
            for (size_t j = 0; j < N; ++j)
            {
                if (i == j) continue;
                IOU(i,j) = this->calBoxIOU(inBoxes[i], inBoxes[j]);
            }
        }

        std::vector<bool> used(N, false);

        // =========================
        // STEP 1: MUTUAL IOU MATCH
        // =========================
        for (size_t i = 0; i < N; ++i)
        {
            if (used[i]) continue;

            int best_j = -1;
            double best_iou = -1;

            for (size_t j = 0; j < N; ++j)
            {
                if (i == j || used[j]) continue;

                if (IOU(i,j) > best_iou)
                {
                    best_iou = IOU(i,j);
                    best_j = (int)j;
                }
            }

            if (best_j == -1) continue;

            int best_i_back = -1;
            double best_iou_back = -1;

            for (size_t k = 0; k < N; ++k)
            {
                if ((int)k == best_j || used[k]) continue;

                if (IOU(k, best_j) > best_iou_back)
                {
                    best_iou_back = IOU(k, best_j);
                    best_i_back = (int)k;
                }
            }

            if (best_i_back == (int)i && best_iou > iouThresh)
            {
                std::vector<int> comp = { (int)i, best_j };

                onboardDetector::box3D box;
                std::vector<Eigen::Vector3d> cluster;
                Eigen::Vector3d center, stddev;

                mergeBoxesSet(inBoxes, inClusters, comp,
                            box, cluster, center, stddev);

                outBoxes.push_back(box);
                outClusters.push_back(cluster);
                outCenters.push_back(center);
                outStds.push_back(stddev);

                used[i] = true;
                used[best_j] = true;
            }
        }

        // =========================
        // STEP 2: BUILD EDGES (NESTED via IOV)
        // =========================
        struct Edge { int parent; int child; double w; };
        std::vector<Edge> edges;

        for (size_t i = 0; i < N; ++i)
        {
            if (used[i]) continue;

            for (size_t j = 0; j < N; ++j)
            {
                if (i == j || used[j]) continue;

                double iov_ij = this->calBoxIOV(inBoxes[i], inBoxes[j]);
                double iov_ji = this->calBoxIOV(inBoxes[j], inBoxes[i]);

                // 🔥 nested detection (asimmetrico!)
                if (iov_ij > iovThresh || iov_ji > iovThresh)
                {
                    Edge e;

                    if (iov_ij > iov_ji)
                    {
                        // j contiene i
                        e.parent = (int)j;
                        e.child  = (int)i;
                        e.w = iov_ij;
                    }
                    else
                    {
                        // i contiene j
                        e.parent = (int)i;
                        e.child  = (int)j;
                        e.w = iov_ji;
                    }

                    edges.push_back(e);
                }
            }
        }

        // =========================
        // STEP 3: KEEP BEST PARENT
        // =========================
        std::map<int, Edge> bestParent;

        for (const auto& e : edges)
        {
            if (bestParent.find(e.child) == bestParent.end() ||
                e.w > bestParent[e.child].w)
            {
                bestParent[e.child] = e;
            }
        }

        // =========================
        // STEP 4: BUILD GRAPH
        // =========================
        std::map<int, std::vector<int>> children;
        std::set<int> allNodes, childNodes;

        for (const auto& kv : bestParent)
        {
            const auto& e = kv.second;

            children[e.parent].push_back(e.child);

            allNodes.insert(e.parent);
            allNodes.insert(e.child);
            childNodes.insert(e.child);
        }

        // =========================
        // STEP 5: MERGE TREES
        // =========================
        for (auto root : allNodes)
        {
            if (childNodes.count(root)) continue;

            std::vector<int> subtree;
            std::queue<int> q;
            q.push(root);

            while (!q.empty())
            {
                int u = q.front(); q.pop();
                subtree.push_back(u);

                for (auto c : children[u])
                    q.push(c);
            }

            onboardDetector::box3D box;
            std::vector<Eigen::Vector3d> cluster;
            Eigen::Vector3d center, stddev;

            mergeBoxesSet(inBoxes, inClusters, subtree,
                        box, cluster, center, stddev);

            outBoxes.push_back(box);
            outClusters.push_back(cluster);
            outCenters.push_back(center);
            outStds.push_back(stddev);

            for (auto idx : subtree)
                used[idx] = true;
        }

        // =========================
        // STEP 6: LEFTOVER
        // =========================
        for (size_t i = 0; i < N; ++i)
        {
            if (!used[i])
            {
                outBoxes.push_back(inBoxes[i]);
                outClusters.push_back(inClusters[i]);
                outCenters.push_back(inCenters[i]);
                outStds.push_back(inStds[i]);
            }
        }
    }

    void dynamicDetector::mergeBoxesSet(
        const std::vector<onboardDetector::box3D>& boxes,
        const std::vector<std::vector<Eigen::Vector3d>>& clusters,
        const std::vector<int>& indices,
        onboardDetector::box3D& outBox,
        std::vector<Eigen::Vector3d>& outCluster,
        Eigen::Vector3d& center,
        Eigen::Vector3d& stddev)
    {
        double xmin =  std::numeric_limits<double>::max();
        double ymin =  std::numeric_limits<double>::max();
        double zmin =  std::numeric_limits<double>::max();

        double xmax = -std::numeric_limits<double>::max();
        double ymax = -std::numeric_limits<double>::max();
        double zmax = -std::numeric_limits<double>::max();

        for (int idx : indices)
        {
            const auto& b = boxes[idx];

            xmax = std::max(xmax, b.x + b.x_width/2);
            xmin = std::min(xmin, b.x - b.x_width/2);
            ymax = std::max(ymax, b.y + b.y_width/2);
            ymin = std::min(ymin, b.y - b.y_width/2);
            zmax = std::max(zmax, b.z + b.z_width/2);
            zmin = std::min(zmin, b.z - b.z_width/2);

            outCluster.insert(outCluster.end(),
                            clusters[idx].begin(),
                            clusters[idx].end());
        }

        outBox.x = (xmin+xmax)/2;
        outBox.y = (ymin+ymax)/2;
        outBox.z = (zmin+zmax)/2;
        outBox.x_width = xmax-xmin;
        outBox.y_width = ymax-ymin;
        outBox.z_width = zmax-zmin;
        outBox.Vx = 0;
        outBox.Vy = 0;

        // center
        center = Eigen::Vector3d::Zero();
        for (auto& p : outCluster) center += p;
        if (!outCluster.empty()) center /= outCluster.size();

        // std
        stddev = Eigen::Vector3d::Zero();
        for (auto& p : outCluster)
            stddev += (p - center).cwiseAbs2();
        if (!outCluster.empty())
            stddev = (stddev / outCluster.size()).cwiseSqrt();
    }

    void dynamicDetector::filterLVBBoxes(){
        std::vector<onboardDetector::box3D> filteredBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClustersTemp;
        std::vector<Eigen::Vector3d> filteredPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> filteredPcClusterStdsTemp; 

        std::vector<onboardDetector::box3D> visualBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> visualPcClustersTemp;
        std::vector<Eigen::Vector3d> visualPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> visualPcClusterStdsTemp; // store visual output

        std::vector<onboardDetector::box3D> lidarBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersTemp;
        std::vector<Eigen::Vector3d> lidarPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> lidarPcClusterStdsTemp; // store lidar output

        // =========================
        // STEP 0: MERGE NESTED UV + DB (INTRA-GROUP)
        // =========================
        std::vector<onboardDetector::box3D> uvBBoxesFiltered;
        std::vector<std::vector<Eigen::Vector3d>> uvPcClustersFiltered;
        std::vector<Eigen::Vector3d> uvPcClusterCentersFiltered;
        std::vector<Eigen::Vector3d> uvPcClusterStdsFiltered;

        std::vector<onboardDetector::box3D> dbBBoxesFiltered;
        std::vector<std::vector<Eigen::Vector3d>> dbPcClustersFiltered;
        std::vector<Eigen::Vector3d> dbPcClusterCentersFiltered;
        std::vector<Eigen::Vector3d> dbPcClusterStdsFiltered;

        // UV
        this->mergeNestedGroup(
            this->uvBBoxes_,
            this->pcClustersVisual_,
            this->pcClusterCentersVisual_,
            this->pcClusterStdsVisual_,

            uvBBoxesFiltered,
            uvPcClustersFiltered,
            uvPcClusterCentersFiltered,
            uvPcClusterStdsFiltered
        );

        // DBSCAN
        this->mergeNestedGroup(
            this->dbBBoxes_,
            this->pcClustersVisual_,
            this->pcClusterCentersVisual_,
            this->pcClusterStdsVisual_,

            dbBBoxesFiltered,
            dbPcClustersFiltered,
            dbPcClusterCentersFiltered,
            dbPcClusterStdsFiltered
        );

        this->publish3dBox(uvBBoxesFiltered, this->uvBBoxesFilteredPub_, 0, 1, 0.5);
        this->publish3dBox(dbBBoxesFiltered, this->dbBBoxesFilteredPub_, 1, 0.5, 0);

        // STEP 1: Merge UV and DBSCAN bounding boxes to get visual bounding boxes (with clusters and features)
        this->BboxesMerger(
            uvBBoxesFiltered, 
            dbBBoxesFiltered,

            uvPcClustersFiltered,
            uvPcClusterCentersFiltered,
            uvPcClusterStdsFiltered,

            dbPcClustersFiltered,
            dbPcClusterCentersFiltered,
            dbPcClusterStdsFiltered,

            visualBBoxesTemp, 
            visualPcClustersTemp, 
            visualPcClusterCentersTemp, 
            visualPcClusterStdsTemp,

            this->visualmergingFlag_, 
            this->uvUnmergedFlag_, 
            this->dbUnmergedFlag_,

            this->visualboxIOUThresh_, 
            this->visualboxIOVThresh_);
        
        this->visualBBoxes_ = visualBBoxesTemp;

        // STEP 2: Get lidar bboxes and its corresponding clusters and features
        // lidar bbox filter
        for (size_t i = 0; i < this->lidarBBoxes_.size(); ++i) {
            onboardDetector::box3D lidarBBox = this->lidarBBoxes_[i];
            
            // get corresponding point cloud cluster
            onboardDetector::Cluster cluster = this->lidarClusters_[i];

            std::vector<Eigen::Vector3d> pcCluster;
            for (const pcl::PointXYZ& point : cluster.points->points) {
                pcCluster.emplace_back(point.x, point.y, point.z);
            }

            // extract the cluster center
            Eigen::Vector3d clusterCenter(cluster.centroid[0], cluster.centroid[1], cluster.centroid[2]);

            // compute std
            Eigen::Vector3d clusterStd = cluster.eigen_values.cwiseSqrt().cast<double>();

            lidarBBoxesTemp.push_back(lidarBBox);
            lidarPcClustersTemp.push_back(pcCluster);
            lidarPcClusterCentersTemp.push_back(clusterCenter);
            lidarPcClusterStdsTemp.push_back(clusterStd);
        }

        // =========================
        // STEP 2.5: MERGE NESTED LIDAR
        // =========================
        std::vector<onboardDetector::box3D> lidarBBoxesFiltered;
        std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersFiltered;
        std::vector<Eigen::Vector3d> lidarPcClusterCentersFiltered;
        std::vector<Eigen::Vector3d> lidarPcClusterStdsFiltered;

        this->mergeNestedGroup(
            lidarBBoxesTemp,
            lidarPcClustersTemp,
            lidarPcClusterCentersTemp,
            lidarPcClusterStdsTemp,

            lidarBBoxesFiltered,
            lidarPcClustersFiltered,
            lidarPcClusterCentersFiltered,
            lidarPcClusterStdsFiltered
        );

        this->publish3dBox(lidarBBoxesFiltered, this->lidarBBoxesFilteredPub_, 1, 0, 0.5);
        
        // STEP 3: Fuse LiDAR and visual bounding boxes
        // init processed flags

        this->BboxesMerger(
            visualBBoxesTemp,                     // group1 = visual
            lidarBBoxesFiltered,                  // group2 = lidar

            visualPcClustersTemp,                 // cluster visual
            visualPcClusterCentersTemp,
            visualPcClusterStdsTemp,

            lidarPcClustersFiltered,              // cluster lidar
            lidarPcClusterCentersFiltered,
            lidarPcClusterStdsFiltered,

            filteredBBoxesTemp,
            filteredPcClustersTemp,
            filteredPcClusterCentersTemp,
            filteredPcClusterStdsTemp,

            this->lidarVisualmergingFlag_,                            
            this->visualUnmergedFlag_,
            this->lidarUnmergedFlag_,                              

            this->lidarVisualboxIOUThresh_,
            this->lidarVisualboxIOVThresh_
        );

        this->filteredBBoxesBeforeYolo_ = filteredBBoxesTemp; // ready for visualization / next steps

        // =========================
        // ADD YOLO DETECTION RESULTS
        // =========================
        if (this->yoloDetectionResults_.detections.size() != 0){
            std::vector<int> best3DBBoxForYOLO(this->yoloDetectionResults_.detections.size(), -1);

            // Project 2D bbox in color image plane from 3D
            vision_msgs::msg::Detection2DArray filteredDetectionResults;
            for (int j=0; j<int(filteredBBoxesTemp.size()); ++j){
                onboardDetector::box3D bbox = filteredBBoxesTemp[j];

                // 1. transform the bounding boxes into the camera frame
                Eigen::Vector3d centerWorld (bbox.x, bbox.y, bbox.z);
                Eigen::Vector3d sizeWorld (bbox.x_width, bbox.y_width, bbox.z_width);
                Eigen::Vector3d centerCam, sizeCam;
                this->transformBBox(centerWorld, sizeWorld, -this->orientationColor_.inverse() * this->positionColor_, this->orientationColor_.inverse(), centerCam, sizeCam);

                // 2. find the top left and bottom right corner 3D position of the transformed bbox
                Eigen::Vector3d topleft (centerCam(0)-sizeCam(0)/2, centerCam(1)-sizeCam(1)/2, centerCam(2));
                Eigen::Vector3d bottomright (centerCam(0)+sizeCam(0)/2, centerCam(1)+sizeCam(1)/2, centerCam(2));

                // 3. project those two points into the camera image plane
                int tlX = (this->fxC_ * topleft(0) + this->cxC_ * topleft(2)) / topleft(2);
                int tlY = (this->fyC_ * topleft(1) + this->cyC_ * topleft(2)) / topleft(2);
                int brX = (this->fxC_ * bottomright(0) + this->cxC_ * bottomright(2)) / bottomright(2);
                int brY = (this->fyC_ * bottomright(1) + this->cyC_ * bottomright(2)) / bottomright(2);

                vision_msgs::msg::Detection2D result;
                // Store top-left corner in hypothesis pose for later extraction
                vision_msgs::msg::ObjectHypothesisWithPose hyp;
                hyp.pose.pose.position.x = tlX;
                hyp.pose.pose.position.y = tlY;
                result.results.push_back(hyp);
                result.bbox.size_x = brX - tlX;
                result.bbox.size_y = brY - tlY;
                filteredDetectionResults.detections.push_back(result);

                // cv::Rect bboxVis;
                // bboxVis.x = tlX;
                // bboxVis.y = tlY;
                // bboxVis.height = brY - tlY;
                // bboxVis.width = brX - tlX;
                // cv::rectangle(this->detectedColorImage_, bboxVis, cv::Scalar(0, 255, 0), 5, 8, 0);
            }

            for (int i=0; i<int(this->yoloDetectionResults_.detections.size()); ++i){
                // Extract YOLO bbox position from hypothesis pose
                int tlXTarget = 0, tlYTarget = 0;
                if (this->yoloDetectionResults_.detections[i].results.size() > 0) {
                    tlXTarget = int(this->yoloDetectionResults_.detections[i].results[0].pose.pose.position.x);
                    tlYTarget = int(this->yoloDetectionResults_.detections[i].results[0].pose.pose.position.y);
                }
                int brXTarget = tlXTarget + int(this->yoloDetectionResults_.detections[i].bbox.size_x);
                int brYTarget = tlYTarget + int(this->yoloDetectionResults_.detections[i].bbox.size_y);

                cv::Rect bboxVis;
                bboxVis.x = tlXTarget;
                bboxVis.y = tlYTarget;
                bboxVis.height = brYTarget - tlYTarget;
                bboxVis.width = brXTarget - tlXTarget;
                cv::rectangle(this->detectedColorImage_, bboxVis, cv::Scalar(255, 0, 0), 5, 8, 0);

                // Define the text to be added
                std::string text = "dynamic";

                // Define the position for the text (above the bounding box)
                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double fontScale = 1.0;
                int thickness = 2;
                int baseline;
                cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
                cv::Point textOrg(bboxVis.x, bboxVis.y - 10);  // 10 pixels above the bounding box

                // Add the text to the image
                cv::putText(this->detectedColorImage_, text, textOrg, fontFace, fontScale, cv::Scalar(255, 0, 0), thickness, 8);

                double bestIOU = 0.0;
                int bestIdx = -1;
                for (int j = 0; j < int(filteredBBoxesTemp.size()); ++j) {
                    // Extract 3D projected bbox position from hypothesis pose
                    int tlX = 0, tlY = 0;
                    if (filteredDetectionResults.detections[j].results.size() > 0) {
                        tlX = int(filteredDetectionResults.detections[j].results[0].pose.pose.position.x);
                        tlY = int(filteredDetectionResults.detections[j].results[0].pose.pose.position.y);
                    }
                    int brX = tlX + int(filteredDetectionResults.detections[j].bbox.size_x);
                    int brY = tlY + int(filteredDetectionResults.detections[j].bbox.size_y);

                    // check the IOU between yolo and projected bbox
                    double xOverlap = double(std::max(0, std::min(brX, brXTarget) - std::max(tlX, tlXTarget)));
                    double yOverlap = double(std::max(0, std::min(brY, brYTarget) - std::max(tlY, tlYTarget)));
                    double intersection = xOverlap * yOverlap;

                    // Calculate union area
                    double areaBox = double((brX - tlX) * (brY - tlY));
                    double areaBoxTarget = double((brXTarget - tlXTarget) * (brYTarget - tlYTarget));
                    double unionArea = areaBox + areaBoxTarget - intersection;

                    double IOU = (unionArea == 0) ? 0 : intersection / unionArea;
                    if (IOU > bestIOU){
                        bestIOU = IOU;
                        bestIdx = j;
                    }
                }

                if (bestIOU > 0.0){
                    best3DBBoxForYOLO[i] = bestIdx;
                }
            }

            std::map<int, std::vector<int>> box3DToYolo;
            for (int i = 0; i < int(best3DBBoxForYOLO.size()); ++i) {
                int idx3D = best3DBBoxForYOLO[i];
                if (idx3D >= 0 && idx3D < int(filteredBBoxesTemp.size())){
                    box3DToYolo[idx3D].push_back(i);
                }
            }

            std::vector<onboardDetector::box3D> newFilteredBBoxes;
            std::vector<std::vector<Eigen::Vector3d>> newFilteredPcClusters;
            std::vector<Eigen::Vector3d> newFilteredPcClusterCenters;
            std::vector<Eigen::Vector3d> newFilteredPcClusterStds;
            
            for (int idx3D = 0; idx3D < int(filteredBBoxesTemp.size()); ++idx3D) {
                auto it = box3DToYolo.find(idx3D);
                // *Case 1: No corresponding yolo box
                if (it == box3DToYolo.end()) {
                    newFilteredBBoxes.push_back(filteredBBoxesTemp[idx3D]);
                    newFilteredPcClusters.push_back(filteredPcClustersTemp[idx3D]);
                    newFilteredPcClusterCenters.push_back(filteredPcClusterCentersTemp[idx3D]);
                    newFilteredPcClusterStds.push_back(filteredPcClusterStdsTemp[idx3D]);
                    continue;
                }

                std::vector<int> yoloIndices = it->second;
                // *Case 2: one yolo box corresponds to one 3D box
                if (yoloIndices.size() == 1) {
                    filteredBBoxesTemp[idx3D].is_dynamic = true;
                    filteredBBoxesTemp[idx3D].is_human = true;
                    newFilteredBBoxes.push_back(filteredBBoxesTemp[idx3D]);
                    newFilteredPcClusters.push_back(filteredPcClustersTemp[idx3D]);
                    newFilteredPcClusterCenters.push_back(filteredPcClusterCentersTemp[idx3D]);
                    newFilteredPcClusterStds.push_back(filteredPcClusterStdsTemp[idx3D]);
                // *Case 3: multiple yolo boxes correspond to one 3D box
                } else {
                    std::vector<Eigen::Vector3d> cloudCluster = filteredPcClustersTemp[idx3D];

                    // iterate to assign all points
                    int allowMargin = 0; // pixel 
                    std::vector<int> assignment(cloudCluster.size(), -1);
                    for (size_t i = 0; i < cloudCluster.size(); ++i){
                        Eigen::Vector3d ptWorld = cloudCluster[i];
                        Eigen::Vector3d ptCam = this->orientationColor_.inverse() * (ptWorld - this->positionColor_);

                        int u = (this->fxC_ * ptCam(0) + this->cxC_ * ptCam(2)) / ptCam(2);
                        int v = (this->fyC_ * ptCam(1) + this->cyC_ * ptCam(2)) / ptCam(2);

                        int closestDist = std::numeric_limits<int>::max();
                        for (int yidx : yoloIndices){
                            int XTargetWid = int(this->yoloDetectionResults_.detections[yidx].bbox.size_x);
                            int YTargetWid = int(this->yoloDetectionResults_.detections[yidx].bbox.size_y);
                            // Extract YOLO bbox top-left corner from hypothesis pose
                            int XTarget = 0, YTarget = 0;
                            if (this->yoloDetectionResults_.detections[yidx].results.size() > 0) {
                                XTarget = int(this->yoloDetectionResults_.detections[yidx].results[0].pose.pose.position.x);
                                YTarget = int(this->yoloDetectionResults_.detections[yidx].results[0].pose.pose.position.y);
                            }
                            int xMin = XTarget;
                            int xMax = XTarget + XTargetWid;
                            int yMin = YTarget;
                            int yMax = YTarget + YTargetWid;

                            if (u >= xMin-allowMargin && u <= xMax+allowMargin && v >= yMin-allowMargin && v <= yMax+allowMargin) {
                                // Horizontal signed distance
                                int horizontalDistance = 0;
                                if (u < xMin) {
                                    horizontalDistance = xMin - u; // Outside on the left
                                } else if (u > xMax) {
                                    horizontalDistance = u - xMax; // Outside on the right
                                } else {
                                    horizontalDistance = std::max(xMin - u, u - xMax); // Inside horizontally
                                }

                                // Compute signed distance to the closest edge
                                int signedDistance;
                                if (u < xMin || u > xMax || v < yMin || v > yMax) {
                                    // Outside: Take the larger of horizontal or vertical distance
                                    signedDistance = horizontalDistance;
                                } else {
                                    // Inside: Take the negative of the minimum distance to any edge
                                    signedDistance = horizontalDistance;
                                }
          
                                int distance = signedDistance;
                                if (distance < closestDist){
                                    assignment[i] = yidx;
                                    closestDist = distance;
                                }
                            }
                        }
                    }

                    std::vector<bool> flag(cloudCluster.size(), false);
                    for (int yidx : yoloIndices){
                        std::vector<Eigen::Vector3d> subCloud;
                        for (size_t i = 0; i < cloudCluster.size(); ++i){
                            if (flag[i]){
                                continue;
                            }

                            if (assignment[i] == yidx){
                                subCloud.push_back(cloudCluster[i]);
                                flag[i] = true;
                            }
                        }
                        if (subCloud.size() != 0){
                            onboardDetector::box3D newBox;
                            Eigen::Vector3d center, stddev;
                            center = computeCenter(subCloud);

                            double xMin = std::numeric_limits<double>::max(), xMax = std::numeric_limits<double>::lowest();
                            double yMin = std::numeric_limits<double>::max(), yMax = std::numeric_limits<double>::lowest();
                            double zMin = std::numeric_limits<double>::max(), zMax = std::numeric_limits<double>::lowest();

                            for (const auto &pt : subCloud) {
                                xMin = std::min(xMin, pt.x());
                                xMax = std::max(xMax, pt.x());
                                yMin = std::min(yMin, pt.y());
                                yMax = std::max(yMax, pt.y());
                                zMin = std::min(zMin, pt.z());
                                zMax = std::max(zMax, pt.z());
                            }
                            // create a new bounding box
                            newBox.x = (xMin + xMax) / 2.;
                            newBox.y = (yMin + yMax) / 2.;
                            newBox.z = (zMin + zMax) / 2.;
                            newBox.x_width = xMax - xMin;
                            newBox.y_width = yMax - yMin;
                            newBox.z_width = zMax - zMin;
                            if (newBox.x_width <= 0 or newBox.y_width <= 0 or newBox.x_width <= 0){
                                continue;
                            }

                            newBox.is_dynamic = true;
                            newBox.is_human = true;

                            stddev = computeStd(subCloud, center);
                            newFilteredBBoxes.push_back(newBox);
                            newFilteredPcClusters.push_back(subCloud);
                            newFilteredPcClusterCenters.push_back(center);
                            newFilteredPcClusterStds.push_back(stddev);
                        }
                    }
                }
            }
            filteredBBoxesTemp = newFilteredBBoxes;
            filteredPcClustersTemp = newFilteredPcClusters;
            filteredPcClusterCentersTemp = newFilteredPcClusterCenters;
            filteredPcClusterStdsTemp = newFilteredPcClusterStds;
        }
        this->filteredBBoxes_ = filteredBBoxesTemp;
        this->filteredPcClusters_ = filteredPcClustersTemp;
        this->filteredPcClusterCenters_ = filteredPcClusterCentersTemp;
        this->filteredPcClusterStds_ = filteredPcClusterStdsTemp;
    }


    void dynamicDetector::transformUVBBoxes(std::vector<onboardDetector::box3D>& bboxes){
        bboxes.clear();
        for(size_t i = 0; i < this->uvDetector_->box3Ds.size(); ++i){
            onboardDetector::box3D bbox;
            double x = this->uvDetector_->box3Ds[i].x; 
            double y = this->uvDetector_->box3Ds[i].y;
            double z = this->uvDetector_->box3Ds[i].z;
            double xWidth = this->uvDetector_->box3Ds[i].x_width;
            double yWidth = this->uvDetector_->box3Ds[i].y_width;
            double zWidth = this->uvDetector_->box3Ds[i].z_width;

            Eigen::Vector3d center (x, y, z);
            Eigen::Vector3d size (xWidth, yWidth, zWidth);
            Eigen::Vector3d newCenter, newSize;

            this->transformBBox(center, size, this->positionDepth_, this->orientationDepth_, newCenter, newSize);

            // assign values to bounding boxes in the map frame
            bbox.x = newCenter(0);
            bbox.y = newCenter(1);
            bbox.z = newCenter(2);
            bbox.x_width = newSize(0);
            bbox.y_width = newSize(1);
            bbox.z_width = newSize(2);
            bboxes.push_back(bbox);            
        }        
    }

    void dynamicDetector::projectDepthImage(){
        this->projPointsNum_ = 0;

        int cols = this->depthImage_.cols;
        int rows = this->depthImage_.rows;
        uint16_t* rowPtr;

        Eigen::Vector3d currPointCam, currPointMap;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        // iterate through each pixel in the depth image
        for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
            rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
            for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
                depth = (*rowPtr) * inv_factor;
                
                if (*rowPtr == 0) {
                    depth = this->raycastMaxLength_ + 0.1;
                } else if (depth < this->depthMinValue_) {
                    continue;
                } else if (depth > this->depthMaxValue_) {
                    depth = this->raycastMaxLength_ + 0.1;
                }
                rowPtr =  rowPtr + this->skipPixel_;

                // get 3D point in camera frame
                currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                currPointCam(2) = depth;
                currPointMap = this->orientationDepth_ * currPointCam + this->positionDepth_; // transform to map coordinate

                this->projPoints_[this->projPointsNum_] = currPointMap;
                this->pointsDepth_[this->projPointsNum_] = depth;
                this->projPointsNum_ = this->projPointsNum_ + 1;
            }
        } 
    }

    void dynamicDetector::filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        // currently there is only one filtered (might include more in the future)
        std::vector<Eigen::Vector3d> voxelFilteredPoints;
        this->voxelFilter(points, voxelFilteredPoints);

        filteredPoints.clear();
        for (const auto& point : voxelFilteredPoints){
            if (point.z() <= this->roofHeight_ && point.z() >= this->groundHeight_){
                filteredPoints.push_back(point);
            }
        }
    }


    void dynamicDetector::clusterPointsAndBBoxes(
        const std::vector<Eigen::Vector3d>& points,
        std::vector<onboardDetector::box3D>& bboxes,
        std::vector<std::vector<Eigen::Vector3d>>& pcClusters,
        std::vector<Eigen::Vector3d>& pcClusterCenters,
        std::vector<Eigen::Vector3d>& pcClusterStds)
    {
        bboxes.clear();
        pcClusters.clear();
        pcClusterCenters.clear();
        pcClusterStds.clear();

        if (points.empty()){
            return;
        }

        std::vector<onboardDetector::Point> pointsDB;
        this->eigenToDBPointVec(points, pointsDB, points.size());

        this->dbCluster_.reset(new DBSCAN(
            this->dbMinPointsCluster_,
            this->dbEpsilon_,
            pointsDB));

        // Camera: refinement OFF
        this->dbCluster_->setRefinementParams(
            false,
            this->dbscanRefineMaxDiagonal_,
            this->dbscanRefineMinDensity_,
            this->dbscanRefineSplitMinPts_,
            this->dbscanRefineSplitEps_,
            this->dbscanRefineMinSubclusterPts_,
            this->dbscanRefineAxisSliceWidth_,
            this->dbscanRefineMaxDepth_,
            this->dbscanRefineRecursive_,
            this->dbscanRefineMinBoxVolume_);

        this->dbCluster_->run();

        int clusterNum = 0;
        for (size_t i = 0; i < this->dbCluster_->m_points.size(); ++i){
            onboardDetector::Point pDB = this->dbCluster_->m_points[i];
            if (pDB.clusterID > clusterNum){
                clusterNum = pDB.clusterID;
            }
        }

        std::vector<std::vector<Eigen::Vector3d>> pcClustersTemp;
        pcClustersTemp.resize(clusterNum);

        for (size_t i = 0; i < this->dbCluster_->m_points.size(); ++i){
            onboardDetector::Point pDB = this->dbCluster_->m_points[i];
            if (pDB.clusterID > 0){
                Eigen::Vector3d p = this->dbPointToEigen(pDB);
                pcClustersTemp[pDB.clusterID - 1].push_back(p);
            }
        }

        for (size_t i=0; i<pcClustersTemp.size(); ++i)
        { 
            onboardDetector::box3D box; 
            double xmin = pcClustersTemp[i][0](0); 
            double ymin = pcClustersTemp[i][0](1); 
            double zmin = pcClustersTemp[i][0](2); 
            double xmax = pcClustersTemp[i][0](0); 
            double ymax = pcClustersTemp[i][0](1); 
            double zmax = pcClustersTemp[i][0](2); 

            for (size_t j=0; j<pcClustersTemp[i].size(); ++j)
            { 
                xmin = (pcClustersTemp[i][j](0)<xmin)?pcClustersTemp[i][j](0):xmin; 
                ymin = (pcClustersTemp[i][j](1)<ymin)?pcClustersTemp[i][j](1):ymin; 
                zmin = (pcClustersTemp[i][j](2)<zmin)?pcClustersTemp[i][j](2):zmin; 
                xmax = (pcClustersTemp[i][j](0)>xmax)?pcClustersTemp[i][j](0):xmax; 
                ymax = (pcClustersTemp[i][j](1)>ymax)?pcClustersTemp[i][j](1):ymax; 
                zmax = (pcClustersTemp[i][j](2)>zmax)?pcClustersTemp[i][j](2):zmax; 
            } 

            box.id = static_cast<int>(i);
            box.x = (xmax + xmin) / 2.0;
            box.y = (ymax + ymin) / 2.0;
            box.z = (zmax + zmin) / 2.0;
            box.x_width = (xmax - xmin) > 0.1 ? (xmax - xmin) : 0.1;
            box.y_width = (ymax - ymin) > 0.1 ? (ymax - ymin) : 0.1;
            box.z_width = (zmax - zmin);
            box.Vx = 0.0;
            box.Vy = 0.0;

            if (box.x_width > this->maxObjectSize_(0) ||
                box.y_width > this->maxObjectSize_(1) ||
                box.z_width > this->maxObjectSize_(2)){
                continue;
            }

            bboxes.push_back(box);
            pcClusters.push_back(pcClustersTemp[i]);
        }

        for (size_t i = 0; i < pcClusters.size(); ++i){
            Eigen::Vector3d pcClusterCenter(0., 0., 0.);
            Eigen::Vector3d pcClusterStd(0., 0., 0.);
            this->calcPcFeat(pcClusters[i], pcClusterCenter, pcClusterStd);
            pcClusterCenters.push_back(pcClusterCenter);
            pcClusterStds.push_back(pcClusterStd);
        }
    }

    void dynamicDetector::voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        const double res = this->voxelSize_; // resolution of voxel (now tunable)
        int xVoxels = ceil(2*this->localSensorRange_(0)/res); int yVoxels = ceil(2*this->localSensorRange_(1)/res); int zVoxels = ceil(2*this->localSensorRange_(2)/res);
        int totalVoxels = xVoxels * yVoxels * zVoxels;
        // std::vector<bool> voxelOccupancyVec (totalVoxels, false);
        std::vector<int> voxelOccupancyVec (totalVoxels, 0);

        // Iterate through each points in the cloud
        filteredPoints.clear();
        
        for (int i=0; i<this->projPointsNum_; ++i){
            Eigen::Vector3d p = points[i];

            if (this->isInFilterRange(p) and p(2) >= this->groundHeight_ and this->pointsDepth_[i] <= this->raycastMaxLength_){
                // find the corresponding voxel id in the vector and check whether it is occupied
                int pID = this->posToAddress(p, res);

                // add one point
                voxelOccupancyVec[pID] +=1;

                // add only if thresh points are found
                if (voxelOccupancyVec[pID] == this->voxelOccThresh_){
                    filteredPoints.push_back(p);
                }
            }
        } 
    }

    void dynamicDetector::calcPcFeat(const std::vector<Eigen::Vector3d>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd){
        int numPoints = pcCluster.size();
        
        // center
        for (int i=0 ; i<numPoints ; i++){
            pcClusterCenter(0) += pcCluster[i](0)/numPoints;
            pcClusterCenter(1) += pcCluster[i](1)/numPoints;
            pcClusterCenter(2) += pcCluster[i](2)/numPoints;
        }

        // std
        for (int i=0 ; i<numPoints ; i++){
            pcClusterStd(0) += std::pow(pcCluster[i](0) - pcClusterCenter(0),2);
            pcClusterStd(1) += std::pow(pcCluster[i](1) - pcClusterCenter(1),2);
            pcClusterStd(2) += std::pow(pcCluster[i](2) - pcClusterCenter(2),2);
        }        

        // take square root
        pcClusterStd(0) = std::sqrt(pcClusterStd(0)/numPoints);
        pcClusterStd(1) = std::sqrt(pcClusterStd(1)/numPoints);
        pcClusterStd(2) = std::sqrt(pcClusterStd(2)/numPoints);
    }


    double dynamicDetector::calBoxIOU(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2, bool ignoreZmin){
        double box1Volume = box1.x_width * box1.y_width * box1.z_width;
        double box2Volume = box2.x_width * box2.y_width * box2.z_width;

        double l1Y = box1.y+box1.y_width/2.-(box2.y-box2.y_width/2.);
        double l2Y = box2.y+box2.y_width/2.-(box1.y-box1.y_width/2.);
        double l1X = box1.x+box1.x_width/2.-(box2.x-box2.x_width/2.);
        double l2X = box2.x+box2.x_width/2.-(box1.x-box1.x_width/2.);
        double l1Z = box1.z+box1.z_width/2.-(box2.z-box2.z_width/2.);
        double l2Z = box2.z+box2.z_width/2.-(box1.z-box1.z_width/2.);
        
        if (ignoreZmin){
            // modify box1 and box2 volumn based on the maximum lower z of two
            double zmin = std::max(box1.z - box1.z_width/2., box2.z - box2.z_width/2.);
            double zWidth1 = box1.z_width/2. + (box1.z - zmin);
            double zWidth2 = box2.z_width/2. + (box2.z - zmin);
            box1Volume = box1.x_width * box1.y_width * zWidth1;
            box2Volume = box2.x_width * box2.y_width * zWidth2;

            l1Z = box1.z+box1.z_width/2. - zmin;
            l2Z = box2.z+box2.z_width/2. - zmin;
        }
        
        double overlapX = std::min( l1X , l2X );
        double overlapY = std::min( l1Y , l2Y );
        double overlapZ = std::min( l1Z , l2Z );
       
        if (std::max(l1X, l2X)<=std::max(box1.x_width,box2.x_width)){ 
            overlapX = std::min(box1.x_width, box2.x_width);
        }
        if (std::max(l1Y, l2Y)<=std::max(box1.y_width,box2.y_width)){ 
            overlapY = std::min(box1.y_width, box2.y_width);
        }
        if (std::max(l1Z, l2Z)<=std::max(box1.z_width,box2.z_width)){ 
            overlapZ = std::min(box1.z_width, box2.z_width);
        }


        double overlapVolume = overlapX * overlapY *  overlapZ;
        double IOU = overlapVolume / (box1Volume+box2Volume-overlapVolume);
        
        // D-IOU
        if (overlapX<=0 || overlapY<=0 ||overlapZ<=0){
            IOU = 0;
        }
        return IOU;
    }

    double dynamicDetector::calBoxIOV(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2, bool ignoreZmin) {
        double box1Volume = box1.x_width * box1.y_width * box1.z_width;

        double l1Y = box1.y + box1.y_width / 2. - (box2.y - box2.y_width / 2.);
        double l2Y = box2.y + box2.y_width / 2. - (box1.y - box1.y_width / 2.);
        double l1X = box1.x + box1.x_width / 2. - (box2.x - box2.x_width / 2.);
        double l2X = box2.x + box2.x_width / 2. - (box1.x - box1.x_width / 2.);
        double l1Z = box1.z + box1.z_width / 2. - (box2.z - box2.z_width / 2.);
        double l2Z = box2.z + box2.z_width / 2. - (box1.z - box1.z_width / 2.);

        if (ignoreZmin) {
            double zmin = std::max(box1.z - box1.z_width / 2., box2.z - box2.z_width / 2.);
            double zWidth1 = box1.z_width / 2. + (box1.z - zmin);
            box1Volume = box1.x_width * box1.y_width * zWidth1;
            l1Z = box1.z + box1.z_width / 2. - zmin;
            l2Z = box2.z + box2.z_width / 2. - zmin;
        }

        double overlapX = std::min(l1X, l2X);
        double overlapY = std::min(l1Y, l2Y);
        double overlapZ = std::min(l1Z, l2Z);

        if (std::max(l1X, l2X) <= std::max(box1.x_width, box2.x_width)) {
            overlapX = std::min(box1.x_width, box2.x_width);
        }
        if (std::max(l1Y, l2Y) <= std::max(box1.y_width, box2.y_width)) {
            overlapY = std::min(box1.y_width, box2.y_width);
        }
        if (std::max(l1Z, l2Z) <= std::max(box1.z_width, box2.z_width)) {
            overlapZ = std::min(box1.z_width, box2.z_width);
        }

        double overlapVolume = overlapX * overlapY * overlapZ;
        double IOV = (box1Volume > 0) ? (overlapVolume / box1Volume) : 0.0;
        if (overlapX <= 0 || overlapY <= 0 || overlapZ <= 0) {
            IOV = 0;
        }
        return IOV;
    }

    double dynamicDetector::computeBoxIoU2D(const onboardDetector::box3D& boxA,
                                            const onboardDetector::box3D& boxB) const
    {
        double axMin = boxA.x - 0.5 * boxA.x_width;
        double axMax = boxA.x + 0.5 * boxA.x_width;
        double ayMin = boxA.y - 0.5 * boxA.y_width;
        double ayMax = boxA.y + 0.5 * boxA.y_width;

        double bxMin = boxB.x - 0.5 * boxB.x_width;
        double bxMax = boxB.x + 0.5 * boxB.x_width;
        double byMin = boxB.y - 0.5 * boxB.y_width;
        double byMax = boxB.y + 0.5 * boxB.y_width;

        double interX = std::max(0.0, std::min(axMax, bxMax) - std::max(axMin, bxMin));
        double interY = std::max(0.0, std::min(ayMax, byMax) - std::max(ayMin, byMin));
        double interArea = interX * interY;

        double areaA = std::max(0.0, boxA.x_width) * std::max(0.0, boxA.y_width);
        double areaB = std::max(0.0, boxB.x_width) * std::max(0.0, boxB.y_width);
        double unionArea = areaA + areaB - interArea;

        if (unionArea <= 1e-9){
            return 0.0;
        }

        return interArea / unionArea;
    } 

    void dynamicDetector::boxAssociation(std::vector<int>& bestMatch){
        int numObjs = int(this->filteredBBoxes_.size()); // current detected bboxes

        if (this->boxHist_.size() == 0){ // initialize new bounding box history if no history exists
            this->boxHist_.resize(numObjs);
            this->pcHist_.resize(numObjs);
            this->pcCenterHist_.resize(numObjs);
            this->missedFrames_.resize(numObjs, 0);
            bestMatch.resize(this->filteredBBoxes_.size(), -1); // first detection no match

            for (int i=0 ; i<numObjs ; ++i){
                // assign persistent track id
                this->filteredBBoxes_[i].id = this->nextTrackId_;
                this->nextTrackId_++;

                // initialize history for bbox, pc and KF
                this->boxHist_[i].push_back(this->filteredBBoxes_[i]);
                this->pcHist_[i].push_back(this->filteredPcClusters_[i]);
                this->pcCenterHist_[i].push_back(this->filteredPcClusterCenters_[i]);

                MatrixXd states, A, B, H, P, Q, R;
                this->kalmanFilterMatrixAcc(this->filteredBBoxes_[i], states, A, B, H, P, Q, R);

                onboardDetector::kalman_filter newFilter;
                newFilter.setup(states, A, B, H, P, Q, R);
                this->filters_.push_back(newFilter);
            }
        }
        else{
            if (this->newDetectFlag_){
                this->boxAssociationHelper(bestMatch);
            }
            else{
                bestMatch.clear();
            }
        }

        this->newDetectFlag_ = false; // the most recent detection has been associated
    }

    void dynamicDetector::boxAssociationHelper(std::vector<int>& bestMatch){
        int numObjs = int(this->filteredBBoxes_.size());

        std::vector<onboardDetector::box3D> prevBBoxes;
        std::vector<Eigen::Vector3d> prevPcCenters;
        std::vector<Eigen::VectorXd> prevBBoxesFeat;

        std::vector<onboardDetector::box3D> propedBBoxes;
        std::vector<Eigen::Vector3d> propedPcCenters;
        std::vector<Eigen::VectorXd> propedBBoxesFeat;

        std::vector<Eigen::VectorXd> currBBoxesFeat;
        currBBoxesFeat.resize(numObjs);
        bestMatch.resize(numObjs, -1);

        std::vector<onboardDetector::clusterGeometry> prevFrameClusterGeometries;
        std::vector<onboardDetector::clusterGeometry> currFrameClusterGeometries;

        this->genFeatHelper(this->filteredBBoxes_, this->filteredPcClusterCenters_, currBBoxesFeat);

        this->getPrevBBoxes(prevBBoxes, prevPcCenters);
        this->genFeatHelper(prevBBoxes, prevPcCenters, prevBBoxesFeat);

        this->getPredictedBBoxesFromFilters(propedBBoxes, propedPcCenters);
        this->genFeatHelper(propedBBoxes, propedPcCenters, propedBBoxesFeat);

        prevFrameClusterGeometries.resize(this->pcHist_.size());
        for (size_t j = 0; j < this->pcHist_.size(); ++j){
            if (!this->pcHist_[j].empty()){
                prevFrameClusterGeometries[j] = this->computeClusterGeometry(this->pcHist_[j][0]);
            }
            else{
                prevFrameClusterGeometries[j] = onboardDetector::clusterGeometry{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
        }

        currFrameClusterGeometries.resize(this->filteredPcClusters_.size());
        for (size_t i = 0; i < this->filteredPcClusters_.size(); ++i){
            currFrameClusterGeometries[i] = this->computeClusterGeometry(this->filteredPcClusters_[i]);
        }

        this->findBestMatch(prevBBoxesFeat,
                            propedBBoxes,
                            propedPcCenters,
                            propedBBoxesFeat,
                            currBBoxesFeat,
                            prevFrameClusterGeometries,
                            currFrameClusterGeometries,
                            bestMatch);
    }
        
    void dynamicDetector::genFeatHelper(const std::vector<onboardDetector::box3D>& boxes,
                                    const std::vector<Eigen::Vector3d>& pcCenters,
                                    std::vector<Eigen::VectorXd>& features){
        Eigen::VectorXd featureWeights = Eigen::VectorXd::Zero(9);
        featureWeights = this->featureWeights_;
        features.resize(boxes.size());

        for (size_t i = 0; i < boxes.size(); ++i) {
            Eigen::VectorXd feature = Eigen::VectorXd::Zero(9);

            feature(0) = (boxes[i].x - this->position_(0)) * featureWeights(0);
            feature(1) = (boxes[i].y - this->position_(1)) * featureWeights(1);
            feature(2) = (boxes[i].z - this->position_(2)) * featureWeights(2);
            feature(3) = boxes[i].x_width * featureWeights(3);
            feature(4) = boxes[i].y_width * featureWeights(4);
            feature(5) = boxes[i].z_width * featureWeights(5);
            feature(6) = pcCenters[i](0) * featureWeights(6);
            feature(7) = pcCenters[i](1) * featureWeights(7);
            feature(8) = pcCenters[i](2) * featureWeights(8);

            for (int j = 0; j < feature.size(); ++j) {
                if (std::isnan(feature(j)) || std::isinf(feature(j))) {
                    feature(j) = 0.0;
                }
            }

            features[i] = feature;
        }
    }

    void dynamicDetector::getPrevBBoxes(std::vector<onboardDetector::box3D>& prevBoxes, std::vector<Eigen::Vector3d>& prevPcCenters){
        onboardDetector::box3D prevBox;
        for (size_t i=0 ; i<this->boxHist_.size() ; i++){
            prevBox = this->boxHist_[i][0];
            prevBoxes.push_back(prevBox);

            Eigen::Vector3d prevPcCenter = this->pcCenterHist_[i][0];
            prevPcCenters.push_back(prevPcCenter);
        }
    }
      
    void dynamicDetector::findBestMatch(const std::vector<Eigen::VectorXd>& prevBBoxesFeat,
                                        const std::vector<onboardDetector::box3D>& propedBBoxes,
                                        const std::vector<Eigen::Vector3d>& propedPcCenters,
                                        const std::vector<Eigen::VectorXd>& propedBBoxesFeat,
                                        const std::vector<Eigen::VectorXd>& currBBoxesFeat,
                                        const std::vector<onboardDetector::clusterGeometry>& prevFrameClusterGeometries,
                                        const std::vector<onboardDetector::clusterGeometry>& currFrameClusterGeometries,
                                        std::vector<int>& bestMatch)
    {
        (void)prevBBoxesFeat;
        (void)propedPcCenters;
        (void)propedBBoxesFeat;
        (void)currBBoxesFeat;
        (void)prevFrameClusterGeometries;
        (void)currFrameClusterGeometries;

        int numObjs = static_cast<int>(this->filteredBBoxes_.size());
        bestMatch.assign(numObjs, -1);

        std::vector<onboardDetector::matchCandidate> candidates;
        candidates.reserve(numObjs * std::max<size_t>(1, propedBBoxes.size()));

        const double dt = this->clampPositive(this->dt_, 1e-3);

        for (int i = 0; i < numObjs; ++i){
            const onboardDetector::box3D& currBBox = this->filteredBBoxes_[i];

            for (size_t j = 0; j < propedBBoxes.size(); ++j){
                const onboardDetector::box3D& predBBox = propedBBoxes[j];

                double posDist = 0.0;
                double requiredSpeed = 0.0;
                double relSizeDiff = 0.0;
                double iou2d = 0.0;
                std::string rejectReason;

                double score = this->computeAssociationScore(predBBox,
                                                            currBBox,
                                                            dt,
                                                            posDist,
                                                            requiredSpeed,
                                                            relSizeDiff,
                                                            iou2d,
                                                            rejectReason);

                int trackId = -1;
                if (j < this->boxHist_.size() && !this->boxHist_[j].empty()){
                    trackId = this->boxHist_[j][0].id;
                }

                if (!rejectReason.empty()){
                    if (this->enableTrackingDebugLogs_){
                        RCLCPP_DEBUG(
                            this->nh_->get_logger(),
                            "[TRACK_%s] curr=%d prev=%d track_id=%d posDist=%.3f reqSpeed=%.3f relSizeDiff=%.3f iou2d=%.3f score=%.3f",
                            rejectReason.c_str(),
                            i,
                            static_cast<int>(j),
                            trackId,
                            posDist,
                            requiredSpeed,
                            relSizeDiff,
                            iou2d,
                            score);
                    }
                    continue;
                }

                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_CANDIDATE] curr=%d prev=%d track_id=%d posDist=%.3f reqSpeed=%.3f relSizeDiff=%.3f iou2d=%.3f score=%.3f",
                        i,
                        static_cast<int>(j),
                        trackId,
                        posDist,
                        requiredSpeed,
                        relSizeDiff,
                        iou2d,
                        score);
                }

                onboardDetector::matchCandidate candidate;
                candidate.currIdx = i;
                candidate.prevIdx = static_cast<int>(j);
                candidate.score = score;
                candidates.push_back(candidate);
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                [](const onboardDetector::matchCandidate& a, const onboardDetector::matchCandidate& b){
                    return a.score > b.score;
                });

        std::vector<bool> currAssigned(numObjs, false);
        std::vector<bool> prevAssigned(propedBBoxes.size(), false);

        for (size_t k = 0; k < candidates.size(); ++k){
            int currIdx = candidates[k].currIdx;
            int prevIdx = candidates[k].prevIdx;

            if (!currAssigned[currIdx] && !prevAssigned[prevIdx]){
                bestMatch[currIdx] = prevIdx;
                currAssigned[currIdx] = true;
                prevAssigned[prevIdx] = true;

                int trackId = -1;
                if (prevIdx < static_cast<int>(this->boxHist_.size()) && !this->boxHist_[prevIdx].empty()){
                    trackId = this->boxHist_[prevIdx][0].id;
                }

                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_ASSIGN] curr=%d prev=%d track_id=%d final_score=%.3f",
                        currIdx,
                        prevIdx,
                        trackId,
                        candidates[k].score);
                }
            }
        }
    }

    void dynamicDetector::kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch){
        std::vector<std::deque<onboardDetector::box3D>> boxHistTemp;
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHistTemp;
        std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
        std::vector<onboardDetector::kalman_filter> filtersTemp;
        std::vector<int> missedFramesTemp;
        std::vector<onboardDetector::box3D> trackedBBoxesTemp;

        std::deque<onboardDetector::box3D> newSingleBoxHist;
        std::deque<std::vector<Eigen::Vector3d>> newSinglePcHist;
        std::deque<Eigen::Vector3d> newSinglePcCenterHist;
        onboardDetector::kalman_filter newFilter;

        int numObjs = static_cast<int>(this->filteredBBoxes_.size());
        int prevTracks = static_cast<int>(this->boxHist_.size());

        std::vector<bool> prevMatched(prevTracks, false);

        for (int i = 0; i < numObjs; ++i){
            if (i < static_cast<int>(bestMatch.size()) && bestMatch[i] >= 0){
                int matchIdx = bestMatch[i];
                prevMatched[matchIdx] = true;

                boxHistTemp.push_back(this->boxHist_[matchIdx]);
                pcHistTemp.push_back(this->pcHist_[matchIdx]);
                pcCenterHistTemp.push_back(this->pcCenterHist_[matchIdx]);
                filtersTemp.push_back(this->filters_[matchIdx]);
                missedFramesTemp.push_back(0);

                onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];
                onboardDetector::box3D newEstimatedBBox;

                Eigen::MatrixXd Z;
                this->getKalmanObservationPos(currDetectedBBox, Z);
                filtersTemp.back().estimate(Z, MatrixXd::Zero(6,1));

                newEstimatedBBox.x = filtersTemp.back().output(0);
                newEstimatedBBox.y = filtersTemp.back().output(1);
                newEstimatedBBox.z = currDetectedBBox.z;
                newEstimatedBBox.Vx = filtersTemp.back().output(2);
                newEstimatedBBox.Vy = filtersTemp.back().output(3);
                newEstimatedBBox.Ax = filtersTemp.back().output(4);
                newEstimatedBBox.Ay = filtersTemp.back().output(5);

                newEstimatedBBox.x_width = currDetectedBBox.x_width;
                newEstimatedBBox.y_width = currDetectedBBox.y_width;
                newEstimatedBBox.z_width = currDetectedBBox.z_width;
                newEstimatedBBox.is_dynamic = currDetectedBBox.is_dynamic;
                newEstimatedBBox.is_human = currDetectedBBox.is_human;
                newEstimatedBBox.id = this->boxHist_[matchIdx][0].id;

                if (int(boxHistTemp.back().size()) == this->histSize_){
                    boxHistTemp.back().pop_back();
                    pcHistTemp.back().pop_back();
                    pcCenterHistTemp.back().pop_back();
                }

                boxHistTemp.back().push_front(newEstimatedBBox);
                pcHistTemp.back().push_front(this->filteredPcClusters_[i]);
                pcCenterHistTemp.back().push_front(this->filteredPcClusterCenters_[i]);

                trackedBBoxesTemp.push_back(newEstimatedBBox);
            }
        }

        for (int j = 0; j < prevTracks; ++j){
            if (prevMatched[j]){
                continue;
            }

            int newMissed = this->missedFrames_[j] + 1;
            if (newMissed > this->maxMissedFrames_){
                continue;
            }

            std::deque<onboardDetector::box3D> hist = this->boxHist_[j];
            std::deque<std::vector<Eigen::Vector3d>> pcHist = this->pcHist_[j];
            std::deque<Eigen::Vector3d> pcCenterHist = this->pcCenterHist_[j];
            onboardDetector::kalman_filter filter = this->filters_[j];

            filter.predict(MatrixXd::Zero(6,1));

            onboardDetector::box3D predictedBBox = hist.front();
            predictedBBox.x = filter.output(0);
            predictedBBox.y = filter.output(1);
            predictedBBox.Vx = filter.output(2);
            predictedBBox.Vy = filter.output(3);
            predictedBBox.Ax = filter.output(4);
            predictedBBox.Ay = filter.output(5);
            predictedBBox.id = hist.front().id;

            if (int(hist.size()) == this->histSize_){
                hist.pop_back();
                pcHist.pop_back();
                pcCenterHist.pop_back();
            }

            hist.push_front(predictedBBox);

            Eigen::Vector3d predictedPcCenter = pcCenterHist.front();
            predictedPcCenter(0) = predictedBBox.x;
            predictedPcCenter(1) = predictedBBox.y;
            pcCenterHist.push_front(predictedPcCenter);

            if (!pcHist.empty()){
                pcHist.push_front(pcHist.front());
            }

            boxHistTemp.push_back(hist);
            pcHistTemp.push_back(pcHist);
            pcCenterHistTemp.push_back(pcCenterHist);
            filtersTemp.push_back(filter);
            missedFramesTemp.push_back(newMissed);
            trackedBBoxesTemp.push_back(predictedBBox);
        }

        for (int i = 0; i < numObjs; ++i){
            if (i < static_cast<int>(bestMatch.size()) && bestMatch[i] >= 0){
                continue;
            }

            const onboardDetector::box3D& currDetectedBBox = this->filteredBBoxes_[i];
            const Eigen::Vector3d& currPcCenter = this->filteredPcClusterCenters_[i];

            if (this->isCloseToExistingTrack(currDetectedBBox, currPcCenter, prevMatched)){
                continue;
            }

            int duplicateTrackId = -1;
            if (this->isDuplicateOfTrackedThisFrame(currDetectedBBox, trackedBBoxesTemp, duplicateTrackId)){
                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_NEW_BLOCKED_BY_CURRENT_FRAME] det=%d duplicate_of_track_id=%d",
                        i,
                        duplicateTrackId);
                }
                continue;
            }

            boxHistTemp.push_back(newSingleBoxHist);
            pcHistTemp.push_back(newSinglePcHist);
            pcCenterHistTemp.push_back(newSinglePcCenterHist);

            onboardDetector::box3D newEstimatedBBox = currDetectedBBox;

            MatrixXd states, A, B, H, P, Q, R;
            this->kalmanFilterMatrixAcc(currDetectedBBox, states, A, B, H, P, Q, R);

            newFilter.setup(states, A, B, H, P, Q, R);
            filtersTemp.push_back(newFilter);
            missedFramesTemp.push_back(0);

            newEstimatedBBox.id = this->nextTrackId_;
            this->nextTrackId_++;

            boxHistTemp.back().push_front(newEstimatedBBox);
            pcHistTemp.back().push_front(this->filteredPcClusters_[i]);
            pcCenterHistTemp.back().push_front(this->filteredPcClusterCenters_[i]);

            trackedBBoxesTemp.push_back(newEstimatedBBox);

            if (this->enableTrackingDebugLogs_){
                RCLCPP_INFO(
                    this->nh_->get_logger(),
                    "[TRACK_NEW] det=%d new_track_id=%d x=%.3f y=%.3f z=%.3f",
                    i,
                    static_cast<int>(newEstimatedBBox.id),
                    newEstimatedBBox.x,
                    newEstimatedBBox.y,
                    newEstimatedBBox.z);
            }
        }

        if (boxHistTemp.size()){
            for (size_t i = 0; i < trackedBBoxesTemp.size(); ++i){
                if (int(boxHistTemp[i].size()) >= this->fixSizeHistThresh_){
                    if ((abs(trackedBBoxesTemp[i].x_width - boxHistTemp[i][1].x_width) / std::max(boxHistTemp[i][1].x_width, 1e-6)) <= this->fixSizeDimThresh_ &&
                        (abs(trackedBBoxesTemp[i].y_width - boxHistTemp[i][1].y_width) / std::max(boxHistTemp[i][1].y_width, 1e-6)) <= this->fixSizeDimThresh_ &&
                        (abs(trackedBBoxesTemp[i].z_width - boxHistTemp[i][1].z_width) / std::max(boxHistTemp[i][1].z_width, 1e-6)) <= this->fixSizeDimThresh_){
                        trackedBBoxesTemp[i].x_width = boxHistTemp[i][1].x_width;
                        trackedBBoxesTemp[i].y_width = boxHistTemp[i][1].y_width;
                        trackedBBoxesTemp[i].z_width = boxHistTemp[i][1].z_width;
                        boxHistTemp[i][0].x_width = trackedBBoxesTemp[i].x_width;
                        boxHistTemp[i][0].y_width = trackedBBoxesTemp[i].y_width;
                        boxHistTemp[i][0].z_width = trackedBBoxesTemp[i].z_width;
                    }
                }
            }
        }

        this->boxHist_ = boxHistTemp;
        this->pcHist_ = pcHistTemp;
        this->pcCenterHist_ = pcCenterHistTemp;
        this->filters_ = filtersTemp;
        this->missedFrames_ = missedFramesTemp;
        this->trackedBBoxes_ = trackedBBoxesTemp;
    }

    void dynamicDetector::kalmanFilterMatrixVel(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R){
        states.resize(4,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        // init vel and acc to zeros
        states(2) = 0.;
        states(3) = 0.;

        MatrixXd ATemp;
        ATemp.resize(4, 4);
        ATemp <<  0, 0, 1, 0,
                  0, 0, 0, 1,
                  0, 0, 0, 0,
                  0 ,0, 0, 0;
        A = MatrixXd::Identity(4,4) + this->dt_*ATemp;
        B = MatrixXd::Zero(4, 4);
        H = MatrixXd::Identity(4, 4);
        P = MatrixXd::Identity(4, 4) * this->eP_;
        Q = MatrixXd::Identity(4, 4);
        Q(0,0) *= this->eQPos_; Q(1,1) *= this->eQPos_; Q(2,2) *= this->eQVel_; Q(3,3) *= this->eQVel_; 
        R = MatrixXd::Identity(4, 4);
        R(0,0) *= this->eRPos_; R(1,1) *= this->eRPos_; R(2,2) *= this->eRVel_; R(3,3) *= this->eRVel_;

    }

    void dynamicDetector::kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox,
                                                MatrixXd& states,
                                                MatrixXd& A,
                                                MatrixXd& B,
                                                MatrixXd& H,
                                                MatrixXd& P,
                                                MatrixXd& Q,
                                                MatrixXd& R){
        states.resize(6,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        states(2) = 0.0;
        states(3) = 0.0;
        states(4) = 0.0;
        states(5) = 0.0;

        A.resize(6, 6);
        A << 1, 0, this->dt_, 0, 0.5 * this->dt_ * this->dt_, 0,
            0, 1, 0, this->dt_, 0, 0.5 * this->dt_ * this->dt_,
            0, 0, 1, 0, this->dt_, 0,
            0, 0, 0, 1, 0, this->dt_,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        B = MatrixXd::Zero(6, 6);

        H.resize(2, 6);
        H << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;

        P = MatrixXd::Identity(6, 6) * this->eP_;

        Q = MatrixXd::Identity(6, 6);
        Q(0,0) *= this->eQPos_;
        Q(1,1) *= this->eQPos_;
        Q(2,2) *= this->eQVel_;
        Q(3,3) *= this->eQVel_;
        Q(4,4) *= this->eQAcc_;
        Q(5,5) *= this->eQAcc_;

        R = MatrixXd::Identity(2, 2);
        R(0,0) *= this->eRPos_;
        R(1,1) *= this->eRPos_;
    }

    void dynamicDetector::getKalmanObservationPos(const onboardDetector::box3D& currDetectedBBox, MatrixXd& Z){
        Z.resize(2, 1);
        Z(0) = currDetectedBBox.x;
        Z(1) = currDetectedBBox.y;
    }

    void dynamicDetector::getKalmanObservationVel(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z){
        Z.resize(4,1);
        Z(0) = currDetectedBBox.x; 
        Z(1) = currDetectedBBox.y;

        // use previous k frame for velocity estimation
        int k = this->kfAvgFrames_;
        int historySize = this->boxHist_[bestMatchIdx].size();
        if (historySize < k){
            k = historySize;
        }
        onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];

        Z(2) = (currDetectedBBox.x-prevMatchBBox.x)/(this->dt_*k);
        Z(3) = (currDetectedBBox.y-prevMatchBBox.y)/(this->dt_*k);
    }

    void dynamicDetector::getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z){
        Z.resize(6, 1);
        Z(0) = currDetectedBBox.x;
        Z(1) = currDetectedBBox.y;

        // use previous k frame for velocity estimation
        int k = this->kfAvgFrames_;
        int historySize = this->boxHist_[bestMatchIdx].size();
        if (historySize < k){
            k = historySize;
        }
        onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];

        Z(2) = (currDetectedBBox.x - prevMatchBBox.x)/(this->dt_*k);
        Z(3) = (currDetectedBBox.y - prevMatchBBox.y)/(this->dt_*k);
        Z(4) = (Z(2) - prevMatchBBox.Vx)/(this->dt_*k);
        Z(5) = (Z(3) - prevMatchBBox.Vy)/(this->dt_*k);
    }

    void dynamicDetector::getPredictedBBoxesFromFilters(std::vector<onboardDetector::box3D>& propedBBoxes,
                                                        std::vector<Eigen::Vector3d>& propedPcCenters){
        propedBBoxes.clear();
        propedPcCenters.clear();

        for (size_t i = 0; i < this->boxHist_.size(); ++i){
            if (this->boxHist_[i].empty()){
                continue;
            }

            onboardDetector::kalman_filter predictedFilter = this->filters_[i];
            predictedFilter.predict(MatrixXd::Zero(6,1));

            onboardDetector::box3D predBBox = this->boxHist_[i][0];
            predBBox.x = predictedFilter.output(0);
            predBBox.y = predictedFilter.output(1);
            predBBox.Vx = predictedFilter.output(2);
            predBBox.Vy = predictedFilter.output(3);
            predBBox.Ax = predictedFilter.output(4);
            predBBox.Ay = predictedFilter.output(5);

            Eigen::Vector3d predPcCenter = this->pcCenterHist_[i][0];
            predPcCenter(0) = predBBox.x;
            predPcCenter(1) = predBBox.y;

            propedBBoxes.push_back(predBBox);
            propedPcCenters.push_back(predPcCenter);
        }
    }
    
    bool dynamicDetector::isCloseToExistingTrack(const onboardDetector::box3D& currDetectedBBox,
                                                const Eigen::Vector3d& currPcCenter,
                                                const std::vector<bool>& prevMatched)
    {
        (void)currPcCenter;
        (void)prevMatched;

        for (size_t j = 0; j < this->boxHist_.size(); ++j){
            if (this->boxHist_[j].empty()){
                continue;
            }

            onboardDetector::kalman_filter predictedFilter = this->filters_[j];
            predictedFilter.predict(MatrixXd::Zero(6,1));

            onboardDetector::box3D predictedBox = this->boxHist_[j][0];
            predictedBox.x = predictedFilter.output(0);
            predictedBox.y = predictedFilter.output(1);
            predictedBox.Vx = predictedFilter.output(2);
            predictedBox.Vy = predictedFilter.output(3);
            predictedBox.Ax = predictedFilter.output(4);
            predictedBox.Ay = predictedFilter.output(5);

            double centerDist = 0.0;
            double iou2d = 0.0;
            double relSizeDiff = 0.0;

            const bool isDuplicate = this->isDetectionDuplicateOfPrediction(currDetectedBBox,
                                                                            predictedBox,
                                                                            centerDist,
                                                                            iou2d,
                                                                            relSizeDiff);

            if (this->enableTrackingDebugLogs_){
                RCLCPP_DEBUG(
                    this->nh_->get_logger(),
                    "[TRACK_NEW_CHECK] candidate_vs_track_id=%d centerDist=%.3f iou2d=%.3f relSizeDiff=%.3f duplicate=%d",
                    static_cast<int>(this->boxHist_[j][0].id),
                    centerDist,
                    iou2d,
                    relSizeDiff,
                    isDuplicate ? 1 : 0);
            }

            if (isDuplicate){
                if (this->enableTrackingDebugLogs_){
                    RCLCPP_DEBUG(
                        this->nh_->get_logger(),
                        "[TRACK_NEW_BLOCKED_BY_EXISTING] candidate_bbox duplicate_of_track_id=%d centerDist=%.3f iou2d=%.3f relSizeDiff=%.3f",
                        static_cast<int>(this->boxHist_[j][0].id),
                        centerDist,
                        iou2d,
                        relSizeDiff);
                }
                return true;
            }
        }

        return false;
    }

    onboardDetector::clusterGeometry dynamicDetector::computeClusterGeometry(const std::vector<Eigen::Vector3d>& cluster){
        onboardDetector::clusterGeometry geom;
        geom.numPoints = static_cast<double>(cluster.size());
        geom.sizeX = 0.0;
        geom.sizeY = 0.0;
        geom.sizeZ = 0.0;
        geom.varX = 0.0;
        geom.varY = 0.0;
        geom.varZ = 0.0;

        if (cluster.empty()){
            return geom;
        }

        double minX = cluster[0](0);
        double maxX = cluster[0](0);
        double minY = cluster[0](1);
        double maxY = cluster[0](1);
        double minZ = cluster[0](2);
        double maxZ = cluster[0](2);

        double meanX = 0.0;
        double meanY = 0.0;
        double meanZ = 0.0;

        for (size_t i = 0; i < cluster.size(); ++i){
            double x = cluster[i](0);
            double y = cluster[i](1);
            double z = cluster[i](2);

            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);

            meanX += x;
            meanY += y;
            meanZ += z;
        }

        meanX /= static_cast<double>(cluster.size());
        meanY /= static_cast<double>(cluster.size());
        meanZ /= static_cast<double>(cluster.size());

        double varX = 0.0;
        double varY = 0.0;
        double varZ = 0.0;

        for (size_t i = 0; i < cluster.size(); ++i){
            double dx = cluster[i](0) - meanX;
            double dy = cluster[i](1) - meanY;
            double dz = cluster[i](2) - meanZ;

            varX += dx * dx;
            varY += dy * dy;
            varZ += dz * dz;
        }

        varX /= static_cast<double>(cluster.size());
        varY /= static_cast<double>(cluster.size());
        varZ /= static_cast<double>(cluster.size());

        geom.sizeX = maxX - minX;
        geom.sizeY = maxY - minY;
        geom.sizeZ = maxZ - minZ;
        geom.varX = varX;
        geom.varY = varY;
        geom.varZ = varZ;

        return geom;
    }

    double dynamicDetector::computeClusterGeometrySimilarity(const onboardDetector::clusterGeometry& geomA,
                                                            const onboardDetector::clusterGeometry& geomB)
    {
        double eps = 1e-6;

        double numPointsDiff = std::abs(geomA.numPoints - geomB.numPoints) / std::max(std::max(geomA.numPoints, geomB.numPoints), 1.0);
        double sizeXDiff = std::abs(geomA.sizeX - geomB.sizeX) / std::max(std::max(geomA.sizeX, geomB.sizeX), eps);
        double sizeYDiff = std::abs(geomA.sizeY - geomB.sizeY) / std::max(std::max(geomA.sizeY, geomB.sizeY), eps);
        double sizeZDiff = std::abs(geomA.sizeZ - geomB.sizeZ) / std::max(std::max(geomA.sizeZ, geomB.sizeZ), eps);

        double varXDiff = std::abs(geomA.varX - geomB.varX) / std::max(std::max(geomA.varX, geomB.varX), eps);
        double varYDiff = std::abs(geomA.varY - geomB.varY) / std::max(std::max(geomA.varY, geomB.varY), eps);
        double varZDiff = std::abs(geomA.varZ - geomB.varZ) / std::max(std::max(geomA.varZ, geomB.varZ), eps);

        double avgDiff =
            (1.0 * numPointsDiff +
            1.0 * sizeXDiff +
            1.0 * sizeYDiff +
            1.0 * sizeZDiff +
            0.5 * varXDiff +
            0.5 * varYDiff +
            0.5 * varZDiff) / 5.5;

        double similarity = 1.0 - avgDiff;

        if (similarity < 0.0){
            similarity = 0.0;
        }
        if (similarity > 1.0){
            similarity = 1.0;
        }

        return similarity;
    }

    double dynamicDetector::computeRelativeSizeDiff(const onboardDetector::box3D& a,
                                                    const onboardDetector::box3D& b) const
    {
        const double eps = 1e-6;

        double dx = std::abs(a.x_width - b.x_width) / std::max(std::max(a.x_width, b.x_width), eps);
        double dy = std::abs(a.y_width - b.y_width) / std::max(std::max(a.y_width, b.y_width), eps);
        double dz = std::abs(a.z_width - b.z_width) / std::max(std::max(a.z_width, b.z_width), eps);

        return (dx + dy + dz) / 3.0;
    }

    bool dynamicDetector::areBoxesDuplicate2D(const onboardDetector::box3D& a,
                                            const onboardDetector::box3D& b) const
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        double iou2d = this->computeBoxIoU2D(a, b);
        double relSizeDiff = this->computeRelativeSizeDiff(a, b);

        if (dist < this->duplicateTrackDistThresh_ &&
            iou2d > this->duplicateTrackIou2DThresh_ &&
            relSizeDiff < this->duplicateSizeRelThresh_)
        {
            return true;
        }

        return false;
    }

    bool dynamicDetector::isDuplicateOfTrackedThisFrame(
        const onboardDetector::box3D& currDetectedBBox,
        const std::vector<onboardDetector::box3D>& trackedBBoxesTemp,
        int& duplicateTrackId) const
    {
        duplicateTrackId = -1;

        for (const auto& trackedBox : trackedBBoxesTemp){
            if (this->areBoxesDuplicate2D(currDetectedBBox, trackedBox)){
                duplicateTrackId = trackedBox.id;
                return true;
            }
        }

        return false;
    }

    void dynamicDetector::logMatchCandidate(int currIdx,
                                            int prevIdx,
                                            int trackId,
                                            double posDist,
                                            double pcDist,
                                            double velDiff,
                                            double sizeDiff,
                                            double featScore,
                                            double clusterGeomSim,
                                            double score,
                                            const std::string& status) const
    {
        if (!this->enableTrackingDebugLogs_){
            return;
        }

        RCLCPP_DEBUG(
            this->nh_->get_logger(),
            "[TRACK_%s] curr=%d prev=%d track_id=%d posDist=%.3f pcDist=%.3f velDiff=%.3f sizeDiff=%.3f featScore=%.3f shapeSim=%.3f score=%.3f",
            status.c_str(),
            currIdx,
            prevIdx,
            trackId,
            posDist,
            pcDist,
            velDiff,
            sizeDiff,
            featScore,
            clusterGeomSim,
            score);
    }

    double dynamicDetector::clampPositive(double value, double minValue) const
    {
        return (value < minValue) ? minValue : value;
    }

    double dynamicDetector::computeAssociationScore(const onboardDetector::box3D& predictedBox,
                                                    const onboardDetector::box3D& currentBox,
                                                    double dt,
                                                    double& posDist,
                                                    double& requiredSpeed,
                                                    double& relSizeDiff,
                                                    double& iou2d,
                                                    std::string& rejectReason) const
    {
        rejectReason.clear();

        const double dx = predictedBox.x - currentBox.x;
        const double dy = predictedBox.y - currentBox.y;
        posDist = std::sqrt(dx * dx + dy * dy);

        if (posDist >= this->maxMatchRange_){
            rejectReason = "REJECT_POS";
            return -1e9;
        }

        dt = this->clampPositive(dt, 1e-3);
        requiredSpeed = posDist / dt;

        if (requiredSpeed >= this->maxMatchSpeed_){
            rejectReason = "REJECT_SPEED";
            return -1e9;
        }

        relSizeDiff = this->computeRelativeSizeDiff(predictedBox, currentBox);
        if (relSizeDiff >= this->maxRelativeSizeDiffMatch_){
            rejectReason = "REJECT_SIZE";
            return -1e9;
        }

        iou2d = this->computeBoxIoU2D(predictedBox, currentBox);

        const double normPosDist = posDist / this->clampPositive(this->maxMatchRange_, 1e-6);

        double score =
            - this->matchPosScoreWeight_ * normPosDist
            - this->matchSizeScoreWeight_ * relSizeDiff
            + this->matchIou2DScoreWeight_ * iou2d;

        if (score < this->minMatchScore_){
            rejectReason = "REJECT_SCORE";
            return score;
        }

        return score;
    }

    bool dynamicDetector::isDetectionDuplicateOfPrediction(const onboardDetector::box3D& currDetectedBBox,
                                                        const onboardDetector::box3D& predictedBox,
                                                        double& centerDist,
                                                        double& iou2d,
                                                        double& relSizeDiff) const
    {
        const double dx = predictedBox.x - currDetectedBBox.x;
        const double dy = predictedBox.y - currDetectedBBox.y;
        centerDist = std::sqrt(dx * dx + dy * dy);

        iou2d = this->computeBoxIoU2D(currDetectedBBox, predictedBox);
        relSizeDiff = this->computeRelativeSizeDiff(currDetectedBBox, predictedBox);

        const bool veryCloseCenter = (centerDist < this->newTrackMinDist_);
        const bool duplicateByOverlap =
            (centerDist < this->duplicateTrackDistThresh_) &&
            (iou2d > this->duplicateTrackIou2DThresh_) &&
            (relSizeDiff < this->duplicateSizeRelThresh_);

        return (veryCloseCenter || duplicateByOverlap);
    }

    void dynamicDetector::getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc){
        Eigen::Vector3d curPoint;
        for (size_t i=0; i<this->filteredPcClusters_.size(); ++i){
            for (size_t j=0; j<this->filteredPcClusters_[i].size(); ++j){
                curPoint = this->filteredPcClusters_[i][j];
                for (size_t k=0; k<this->dynamicBBoxes_.size(); ++k){
                    if (abs(curPoint(0)-this->dynamicBBoxes_[k].x)<=this->dynamicBBoxes_[k].x_width/2 and 
                        abs(curPoint(1)-this->dynamicBBoxes_[k].y)<=this->dynamicBBoxes_[k].y_width/2 and 
                        abs(curPoint(2)-this->dynamicBBoxes_[k].z)<=this->dynamicBBoxes_[k].z_width/2) {
                        dynamicPc.push_back(curPoint);
                        break;
                    }
                }
            }
        }
    } 
    
    void dynamicDetector::publishUVImages(){
        if (this->uvDetector_ != NULL){
            std_msgs::msg::Header depthHeader;
            depthHeader.stamp = this->nh_->now();
            depthHeader.frame_id = this->tfDepthFrame_;

            auto depthBoxMsg = cv_bridge::CvImage(depthHeader, "bgr8", this->uvDetector_->depth_show).toImageMsg();
            auto UmapBoxMsg = cv_bridge::CvImage(depthHeader, "bgr8", this->uvDetector_->U_map_show).toImageMsg();
            auto birdBoxMsg = cv_bridge::CvImage(depthHeader, "bgr8", this->uvDetector_->bird_view).toImageMsg();  
            this->uvDepthMapPub_->publish(*depthBoxMsg);
            this->uDepthMapPub_->publish(*UmapBoxMsg); 
            this->uvBirdViewPub_->publish(*birdBoxMsg);
        }     
    }


    void dynamicDetector::publishColorImages(){
        std_msgs::msg::Header colorHeader;
        colorHeader.stamp = this->nh_->now();
        colorHeader.frame_id = this->tfColorFrame_;
        auto detectedColorImgMsg = cv_bridge::CvImage(colorHeader, "rgb8", this->detectedColorImage_).toImageMsg();
        this->detectedColorImgPub_->publish(*detectedColorImgMsg);
    }

    void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d>& points, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher){
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;        
        for (size_t i=0; i<points.size(); ++i){
            pt.x = points[i](0);
            pt.y = points[i](1);
            pt.z = points[i](2);
            cloud.push_back(pt);
        }    
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";

        sensor_msgs::msg::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud, cloudMsg);
        publisher->publish(cloudMsg);
    }


    void dynamicDetector::publish3dBox(const std::vector<box3D>& boxes,
                                const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
                                double r, double g, double b){
        visualization_msgs::msg::MarkerArray markers;

        for (size_t i = 0; i < boxes.size(); i++)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.ns = "box3D";
            line.id = i;
            line.type = visualization_msgs::msg::Marker::LINE_LIST;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.06;
            line.color.r = r;
            line.color.g = g;
            line.color.b = b;
            line.color.a = 1.0;
            line.lifetime = rclcpp::Duration::from_seconds(0.05);
            line.pose.orientation.x = 0.0;
            line.pose.orientation.y = 0.0;
            line.pose.orientation.z = 0.0;
            line.pose.orientation.w = 1.0;

            line.pose.position.x = boxes[i].x;
            line.pose.position.y = boxes[i].y;
            line.pose.position.z = boxes[i].z;

            double half_x = boxes[i].x_width / 2.0;
            double half_y = boxes[i].y_width / 2.0;
            double half_z = boxes[i].z_width / 2.0;

            geometry_msgs::msg::Point corner[8];
            corner[0].x = -half_x; corner[0].y = -half_y; corner[0].z = -half_z;
            corner[1].x = -half_x; corner[1].y =  half_y; corner[1].z = -half_z;
            corner[2].x =  half_x; corner[2].y =  half_y; corner[2].z = -half_z;
            corner[3].x =  half_x; corner[3].y = -half_y; corner[3].z = -half_z;
            corner[4].x = -half_x; corner[4].y = -half_y; corner[4].z =  half_z;
            corner[5].x = -half_x; corner[5].y =  half_y; corner[5].z =  half_z;
            corner[6].x =  half_x; corner[6].y =  half_y; corner[6].z =  half_z;
            corner[7].x =  half_x; corner[7].y = -half_y; corner[7].z =  half_z;

            int edgeIdx[12][2] = {
                {0,1}, {1,2}, {2,3}, {3,0},
                {4,5}, {5,6}, {6,7}, {7,4},
                {0,4}, {1,5}, {2,6}, {3,7}
            };

            for (int e = 0; e < 12; e++)
            {
                line.points.push_back(corner[edgeIdx[e][0]]);
                line.points.push_back(corner[edgeIdx[e][1]]);
            }

            markers.markers.push_back(line);
        }

        publisher->publish(markers);
    }


    void dynamicDetector::publishHistoryTraj(){
        visualization_msgs::msg::MarkerArray trajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->boxHist_.size(); ++i){
            if (this->boxHist_[i].size() > 1){
                visualization_msgs::msg::Marker traj;
                traj.header.frame_id = "map";
                traj.header.stamp = this->nh_->now();
                traj.ns = "dynamic_detector";
                traj.id = countMarker;
                traj.type = visualization_msgs::msg::Marker::LINE_LIST;
                traj.scale.x = 0.03;
                traj.scale.y = 0.03;
                traj.scale.z = 0.03;
                traj.color.a = 1.0; // Don't forget to set the alpha!
                traj.color.r = 0.0;
                traj.color.g = 1.0;
                traj.color.b = 0.0;
                traj.pose.orientation.w = 1.0;
                traj.pose.orientation.x = 0.0;
                traj.pose.orientation.y = 0.0;
                traj.pose.orientation.z = 0.0;
                for (size_t j=0; j<this->boxHist_[i].size()-1; ++j){
                    geometry_msgs::msg::Point p1, p2;
                    onboardDetector::box3D box1 = this->boxHist_[i][j];
                    onboardDetector::box3D box2 = this->boxHist_[i][j+1];
                    p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
                    p2.x = box2.x; p2.y = box2.y; p2.z = box2.z;
                    traj.points.push_back(p1);
                    traj.points.push_back(p2);
                }

                ++countMarker;
                trajMsg.markers.push_back(traj);
            }
        }
        this->historyTrajPub_->publish(trajMsg);
    }

    void dynamicDetector::publishVelVis(){ // publish velocities for all tracked objects
        visualization_msgs::msg::MarkerArray velVisMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->trackedBBoxes_.size(); ++i){
            visualization_msgs::msg::Marker velMarker;
            velMarker.header.frame_id = "map";
            velMarker.header.stamp = this->nh_->now();
            velMarker.ns = "dynamic_detector";
            velMarker.id =  countMarker;
            velMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            velMarker.pose.position.x = this->trackedBBoxes_[i].x;
            velMarker.pose.position.y = this->trackedBBoxes_[i].y;
            velMarker.pose.position.z = this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width/2. + 0.3;
            velMarker.scale.x = 0.15;
            velMarker.scale.y = 0.15;
            velMarker.scale.z = 0.15;
            velMarker.color.a = 1.0;
            velMarker.color.r = 1.0;
            velMarker.color.g = 0.0;
            velMarker.color.b = 0.0;
            velMarker.lifetime = rclcpp::Duration::from_seconds(0.1);
            double vx = this->trackedBBoxes_[i].Vx;
            double vy = this->trackedBBoxes_[i].Vy;
            double vNorm = sqrt(vx*vx+vy*vy);
            std::string velText = "Vx=" + std::to_string(vx) + ", Vy=" + std::to_string(vy) + ", |V|=" + std::to_string(vNorm);
            velMarker.text = velText;
            velVisMsg.markers.push_back(velMarker);
            ++countMarker;
        }
        this->velVisPub_->publish(velVisMsg);
    }

    void dynamicDetector::publishLidarClusters(){
        sensor_msgs::msg::PointCloud2 lidarClustersMsg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t i=0; i<this->lidarClusters_.size(); ++i){
            onboardDetector::Cluster & cluster = this->lidarClusters_[i];

            std_msgs::msg::ColorRGBA color;
            srand(cluster.cluster_id);
            color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            // color.r = 0.5;
            // color.g = 0.5;
            // color.b = 0.5;
            // color.a = 1.0;

            for (size_t j=0; j<cluster.points->size(); ++j){
                pcl::PointXYZRGB point;
                const pcl::PointXYZ & pt = cluster.points->at(j);
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                point.r = color.r * 255;
                point.g = color.g * 255;
                point.b = color.b * 255;
                colored_cloud->push_back(point);
            }
        }
        pcl::toROSMsg(*colored_cloud, lidarClustersMsg);
        lidarClustersMsg.header.frame_id = "map";
        lidarClustersMsg.header.stamp = this->nh_->now();
        this->lidarClustersPub_->publish(lidarClustersMsg);
    }

    void dynamicDetector::publishFilteredPoints(){
        sensor_msgs::msg::PointCloud2 filteredPointsMsg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t i=0; i<this->filteredPcClusters_.size(); ++i){
            std_msgs::msg::ColorRGBA color;
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
            color.a = 1.0;

            for (size_t j=0; j<this->filteredPcClusters_[i].size(); ++j){
                pcl::PointXYZRGB point;
                point.x = this->filteredPcClusters_[i][j](0);
                point.y = this->filteredPcClusters_[i][j](1);
                point.z = this->filteredPcClusters_[i][j](2);
                point.r = color.r * 255;
                point.g = color.g * 255;
                point.b = color.b * 255;
                colored_cloud->push_back(point);
            }
        }
        pcl::toROSMsg(*colored_cloud, filteredPointsMsg);
        filteredPointsMsg.header.frame_id = "map";
        filteredPointsMsg.header.stamp = this->nh_->now();
        this->filteredPointsPub_->publish(filteredPointsMsg);
    }

    void dynamicDetector::publishRawDynamicPoints(){
        if (not this->latestCloud_){
            return;
        }
        try {
            pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (this->hasSensorPose_) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(*this->latestCloud_, *tempCloud);
                
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.linear() = this->orientationLidar_;
                transform.translation() = this->positionLidar_;
                
                pcl::transformPointCloud(*tempCloud, *globalCloud, transform);
                sensor_msgs::msg::PointCloud2 cloudMsg;
                pcl::toROSMsg(*globalCloud, cloudMsg);
                cloudMsg.header.frame_id = "map";
                cloudMsg.header.stamp = this->nh_->now();
                this->rawLidarPointsPub_->publish(cloudMsg);
            }
            else {
                pcl::fromROSMsg(*this->latestCloud_, *globalCloud);
            }
            
            std::vector<Eigen::Vector3d> dynamicEigenPoints;
            
            for (const auto& box : this->dynamicBBoxes_) {
                if (!box.is_dynamic)
                    continue;
                
                double xmin = box.x - box.x_width / 2.0;
                double xmax = box.x + box.x_width / 2.0;
                double ymin = box.y - box.y_width / 2.0;
                double ymax = box.y + box.y_width / 2.0;
                double zmin = box.z - box.z_width / 2.0;
                double zmax = box.z + box.z_width / 2.0;
                
                for (const auto& point : globalCloud->points) {
                    if (point.x >= xmin && point.x <= xmax &&
                        point.y >= ymin && point.y <= ymax &&
                        point.z >= zmin && point.z <= zmax)
                    {
                        dynamicEigenPoints.push_back(Eigen::Vector3d(point.x, point.y, point.z));
                    }
                }
            }
            
            if (dynamicEigenPoints.empty()) {
                return;
            }
            
            this->publishPoints(dynamicEigenPoints, this->rawDynamicPointsPub_);
        }
        catch (const pcl::PCLException& e) {
            RCLCPP_ERROR(this->nh_->get_logger(), "PCL Exception during dynamic point extraction: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->nh_->get_logger(), "Standard Exception during dynamic point extraction: %s", e.what());
        }
        catch (...) {
            RCLCPP_ERROR(this->nh_->get_logger(), "Unknown error during dynamic point extraction.");
        }
    }

    void dynamicDetector::transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                                               Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize){
        double x = center(0); 
        double y = center(1);
        double z = center(2);
        double xWidth = size(0);
        double yWidth = size(1);
        double zWidth = size(2);

        // get 8 bouding boxes coordinates in the camera frame
        Eigen::Vector3d p1 (x+xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p2 (x+xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p3 (x+xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p4 (x+xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p5 (x-xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p6 (x-xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p7 (x-xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p8 (x-xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);

        // transform 8 points to the map coordinate frame
        Eigen::Vector3d p1m = orientation * p1 + position;
        Eigen::Vector3d p2m = orientation * p2 + position;
        Eigen::Vector3d p3m = orientation * p3 + position;
        Eigen::Vector3d p4m = orientation * p4 + position;
        Eigen::Vector3d p5m = orientation * p5 + position;
        Eigen::Vector3d p6m = orientation * p6 + position;
        Eigen::Vector3d p7m = orientation * p7 + position;
        Eigen::Vector3d p8m = orientation * p8 + position;
        std::vector<Eigen::Vector3d> pointsMap {p1m, p2m, p3m, p4m, p5m, p6m, p7m, p8m};

        // find max min in x, y, z directions
        double xmin=p1m(0); double xmax=p1m(0); 
        double ymin=p1m(1); double ymax=p1m(1);
        double zmin=p1m(2); double zmax=p1m(2);
        for (Eigen::Vector3d pm : pointsMap){
            if (pm(0) < xmin){xmin = pm(0);}
            if (pm(0) > xmax){xmax = pm(0);}
            if (pm(1) < ymin){ymin = pm(1);}
            if (pm(1) > ymax){ymax = pm(1);}
            if (pm(2) < zmin){zmin = pm(2);}
            if (pm(2) > zmax){zmax = pm(2);}
        }
        newCenter(0) = (xmin + xmax)/2.0;
        newCenter(1) = (ymin + ymax)/2.0;
        newCenter(2) = (zmin + zmax)/2.0;
        newSize(0) = xmax - xmin;
        newSize(1) = ymax - ymin;
        newSize(2) = zmax - zmin;
    }

    // user functions
    void dynamicDetector::getDynamicObstacles(std::vector<onboardDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize){
        incomeDynamicBBoxes.clear();
        for (int i=0; i<int(this->dynamicBBoxes_.size()); i++){
            onboardDetector::box3D box = this->dynamicBBoxes_[i];
            box.x_width += robotSize(0);
            box.y_width += robotSize(1);
            box.z_width += robotSize(2);
            incomeDynamicBBoxes.push_back(box);
        }
    }

    void dynamicDetector::getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, std::vector<std::vector<Eigen::Vector3d>>& velHist, std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize){
		posHist.clear();
        velHist.clear();
        sizeHist.clear();

        if (this->boxHist_.size()){
            for (size_t i=0 ; i<this->boxHist_.size() ; ++i){
                if (this->boxHist_[i][0].is_dynamic or this->boxHist_[i][0].is_human){   
                    bool findMatch = false;     
                    if (this->constrainSize_){
                        for (Eigen::Vector3d targetSize : this->targetObjectSize_){
                            double xdiff = std::abs(this->boxHist_[i][0].x_width - targetSize(0));
                            double ydiff = std::abs(this->boxHist_[i][0].y_width - targetSize(1));
                            double zdiff = std::abs(this->boxHist_[i][0].z_width - targetSize(2)); 
                            if (xdiff < 0.8 and ydiff < 0.8 and zdiff < 1.0){
                                findMatch = true;
                            }
                        }
                    }
                    else{
                        findMatch = true;
                    }
                    if (findMatch){
                        std::vector<Eigen::Vector3d> obPosHist, obVelHist, obSizeHist;
                        for (size_t j=0; j<this->boxHist_[i].size() ; ++j){
                            Eigen::Vector3d pos(this->boxHist_[i][j].x, this->boxHist_[i][j].y, this->boxHist_[i][j].z);
                            Eigen::Vector3d vel(this->boxHist_[i][j].Vx, this->boxHist_[i][j].Vy, 0);
                            Eigen::Vector3d size(this->boxHist_[i][j].x_width, this->boxHist_[i][j].y_width, this->boxHist_[i][j].z_width);
                            size += robotSize;
                            obPosHist.push_back(pos);
                            obVelHist.push_back(vel);
                            obSizeHist.push_back(size);
                        }
                        posHist.push_back(obPosHist);
                        velHist.push_back(obVelHist);
                        sizeHist.push_back(obSizeHist);
                    }
                }
            }
        }
	}
}