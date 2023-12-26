#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToTwistNode : public rclcpp::Node {
public:
  JoyToTwistNode(const std::string name, const rclcpp::NodeOptions & options) : Node(name, options) {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToTwistNode::joyCallback, this, std::placeholders::_1));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    // ジョイスティックのボタンや軸の入力を取得し、それに基づいてTwistメッセージを生成
    // twist_msg->linear.x = joy_msg->axes[1]; // 前後方向の速度
    // twist_msg->angular.z = joy_msg->axes[3]; // 回転速度
   geometry_msgs::msg::Twist twist_msg;
   twist_msg.linear.x = joy_msg->axes[1];
   twist_msg.angular.z = joy_msg->axes[2] * 1.5707;
   twist_publisher_->publish(twist_msg);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<JoyToTwistNode>("joy2twist", options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
