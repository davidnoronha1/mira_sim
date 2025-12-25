// #include <custom_msgs/msg/commands.h>
#include <mira2_rov/keyboard_utils.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
