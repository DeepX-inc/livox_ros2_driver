#include <string>
#include <memory>
#include "include/livox_ros2_driver.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_name = std::string("livox_driver_node");

    rclcpp::Rate loop_rate(10);
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    // options.allow_undeclared_parameters(true);
    // options.automatically_declare_parameters_from_overrides(true);

    auto livox_driver = std::make_shared<livox_ros::LivoxDriver>(
        node_name, options);
    livox_driver->init();
    while (rclcpp::ok()) {
        RCLCPP_INFO(livox_driver->get_logger(), "Spinning1");
        livox_driver->lddc_ptr_->DistributeLidarData();
        RCLCPP_INFO(livox_driver->get_logger(), "Spinning2");
        rclcpp::spin_some(livox_driver);
        RCLCPP_INFO(livox_driver->get_logger(), "Spinning3");
        loop_rate.sleep();
    }
    // rclcpp::spin(livox_driver);
    rclcpp::shutdown();
    return 0;
}