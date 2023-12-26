#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("imu_subscriber"), "Received IMU Data");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_sub_node");

    auto subscription = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, imuCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
