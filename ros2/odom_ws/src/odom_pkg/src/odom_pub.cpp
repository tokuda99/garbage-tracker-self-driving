#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TwistToOdometryNode : public rclcpp::Node {
public:
  TwistToOdometryNode() : Node("twist_to_odometry") {
    twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "twist_topic", 10, std::bind(&TwistToOdometryNode::twistCallback, this, std::placeholders::_1));
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  }

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) {
    // Twistメッセージを受け取ったときの処理
    // double linear_x = twist_msg->linear.x;
    // double linear_y = twist_msg->linear.y;
    // double angular_z = twist_msg->angular.z;

    odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_msg->header.stamp = this->now();
    odometry_msg->header.frame_id = "odom";
    odometry_msg->child_frame_id = "base_link";
    odometry_msg->pose.pose.position.x = 1.0;
    odometry_msg->pose.pose.position.y = 2.0;
    odometry_msg->pose.pose.position.z = 0.0;
    odometry_msg->twist.twist = *twist_msg;
    odometry_publisher_->publish(*odometry_msg);

  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  nav_msgs::msg::Odometry::SharedPtr odometry_msg;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}