/*
 * scout_base_node.cpp
 *
 * Created on: Oct 15, 2021 16:20
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "scout_base/scout_base_ros.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void RequestStop(int) { stop_requested = 1; }
}

int main(int argc, char **argv) {
  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);
  int result = EXIT_FAILURE;
  try {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    // Destroy ROS entities and join CAN workers before tearing down the RMW
    // context, including when initialization or Zenoh entity creation throws.
    {
      auto robot = std::make_shared<westonrobot::ScoutBaseRos>("scout", [] { return stop_requested != 0; });
      if (robot->Initialize()) result = robot->Run() ? EXIT_SUCCESS : EXIT_FAILURE;
      else if (stop_requested) result = EXIT_SUCCESS;
      else std::cerr << "scout_base initialization failed" << std::endl;
      robot->Stop();
    }
  } catch (const std::exception &e) {
    std::cerr << "scout_base failed (CAN/ROS middleware): " << e.what() << std::endl;
    if (stop_requested) result = EXIT_SUCCESS;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
