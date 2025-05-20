#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sim_param_server");

  node->declare_parameter<std::string>("my_param", "hello_world");
  node->declare_parameter<int>("other_param", 42);

  node->declare_parameter<double>("ground_truth_pub_rate", 0.01);
  node->declare_parameter<double>("ground_truth_pos_noise", 0.0);
  node->declare_parameter<double>("ground_truth_vel_noise", 0.0);
  node->declare_parameter<double>("ground_truth_rot_noise", 0.0);
  node->declare_parameter<double>("ground_truth_angular_noise", 0.0);
  node->declare_parameter<double>("ground_truth_rot_drift", 0.0);
  node->declare_parameter<double>("ground_truth_vel_drift", 0.0);
  node->declare_parameter<double>("ground_truth_angular_drift", 0.0);
  node->declare_parameter<double>("ground_truth_rot_drift_frequency", 0.0);
  node->declare_parameter<double>("ground_truth_vel_drift_frequency", 0.0);
  node->declare_parameter<double>("ground_truth_angular_drift_frequency", 0.0);
  node->declare_parameter<double>("mocap_pub_rate", 0.01);
  node->declare_parameter<double>("mocap_pos_noise", 0.001);
  node->declare_parameter<double>("mocap_rot_noise", 0.001);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
