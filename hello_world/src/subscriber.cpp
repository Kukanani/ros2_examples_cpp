// A very simple ROS2 subscriber. 15 lines of code.
// Author: Adam Allevato

#include <iostream>
#include <memory>

// Include the ROS2 Client Library for C++ (rclcpp)
#include "rclcpp/rclcpp.hpp"

// In ROS2, messages are usually defined in separate packages from nodes.
// Here, we include the String message type so we can publish it.
#include "std_msgs/msg/string.hpp"

int main(int argc, char* argv[]) {
  // Set up ROS2
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new ROS2 node named "subscriber"
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("subscriber");

  // Define a callback that prints the data contained within a ROS String message.
  // The callback is defined using a C++ lambda, which is a compact way to define
  // a function.
  auto callback = [](const std_msgs::msg::String::SharedPtr msg) {
    std::cout << "Received: " << msg->data << std::endl;
  };

  // Set up a subscription to the "hello" topic. Messages that arrive on the topic
  // will be passed to the callback. "1" is the queue size - how many messages can
  // be received and queued for callback processing at any given time.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription =
      node->create_subscription<std_msgs::msg::String>("hello", 1, callback);

  // "Spinning" handles ROS communication for a node.
  // In this case, it will receive messages and trigger callbacks.
  // spin() will run forever until the node is sent a shutdown signal
  // (such as Ctrl-C from the terminal).
  rclcpp::spin(node);

  // Clean up
  rclcpp::shutdown();
  return 0;
}
