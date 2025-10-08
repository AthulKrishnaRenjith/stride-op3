#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_demo/soccer_demo.h"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include <std_msgs/msg/string.hpp>

const int SPIN_RATE = 30;

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("soccer_demo_continuous");

  // Create SoccerDemo instance
  auto soccer_demo = std::make_shared<robotis_op::SoccerDemo>();
  soccer_demo->setNode(node);

  // Enable Soccer Demo immediately
  soccer_demo->setDemoEnable();
  RCLCPP_INFO(node->get_logger(), "Soccer Demo started and running continuously");

  // Wait for op3_manager
  std::string manager_name = "/op3_manager";
  while (rclcpp::ok())
  {
    auto node_list = node->get_node_graph_interface()->get_node_names();
    bool found = false;
    for (const auto& n : node_list)
    {
      if (n == manager_name)
      {
        found = true;
        break;
      }
    }
    if (found)
    {
      RCLCPP_INFO(node->get_logger(), "Connected to op3_manager");
      break;
    }
    RCLCPP_WARN(node->get_logger(), "Waiting for op3_manager...");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // Main loop (continuous processing)
  rclcpp::Rate loop_rate(SPIN_RATE);
  while (rclcpp::ok())
  {
    soccer_demo->process();    // Keep running soccer demo
    rclcpp::spin_some(node);   // Process callbacks
    loop_rate.sleep();         // Maintain SPIN_RATE
  }

  rclcpp::shutdown();
  return 0;
}
