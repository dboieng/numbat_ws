// Node file to create object and initialize the ROS 2 node

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "spot_micro_motion_cmd.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SpotMicroMotionCmd>();

  rclcpp::Rate rate(1.0 / node->getNodeConfig().dt);

  // Only proceed if servo configuration publishing succeeds.
  if (node->publishServoConfiguration()) {
    const bool debug_mode = node->getNodeConfig().debug_mode;

    while (rclcpp::ok()) {
      rclcpp::Time begin;

      if (debug_mode) {
        begin = node->now();
      }

      node->runOnce();

      rclcpp::spin_some(node);
      rate.sleep();

      if (debug_mode) {
        const rclcpp::Duration elapsed = node->now() - begin;
        std::cout << elapsed.seconds() << " sec" << std::endl;
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}