#include <string>
#include <memory>
#include "include/livox_ros2_driver.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_name = std::string("livox_driver_node");
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.start_parameter_services(false);

    auto livox_driver = std::make_shared<livox_ros::LivoxDriver>(
        node_name, options);
    livox_driver->init();
    rclcpp::spin(livox_driver);
    rclcpp::shutdown();
    return 0;
}