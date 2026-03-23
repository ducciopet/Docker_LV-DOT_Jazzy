/*
	FILE: detector_node.cpp
	--------------------------
	Run detector node
*/
#include <rclcpp/rclcpp.hpp>

#include <obstacle_avoidance/obstacleAvoider.h>

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);
	auto nh = std::make_shared<rclcpp::Node>("obstacle_avoidance_node", options);

	obstacle_avoidance::ObstacleAvoider avoider(nh);

	rclcpp::spin(nh);
	rclcpp::shutdown();
	return 0;
}