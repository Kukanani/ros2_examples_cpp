// A very simple ROS2 publisher. 18 lines of code.
// Author: Adam Allevato

#include <chrono>
#include <memory>

// Include the ROS2 Client Library for C++ (rclcpp)
#include "rclcpp/rclcpp.hpp"

// In ROS2, messages are usually defined in separate packages from nodes.
// Here, we include the String message type so we can publish it.
#include "std_msgs/msg/string.hpp"

// This line allows us to specify time in milliseconds (ms) later in the code.
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  // Set up ROS2
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new ROS2 node named "publisher"
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("publisher");

  // Set up a publisher on the "hello" topic. "1" is the queue size - how many
  // messages can be queued for publishing at any given time.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher =
      node->create_publisher<std_msgs::msg::String>("hello", 1);

  // Since ROS2 publisher/subscriber connections happen between different
  // processes, we cannot guarantee that a message will be delivered if it
  // is published immediately.
  // Therefore, we sleep to give some time for connections to establish in the background.
  // 100ms should be plenty of time for two nodes running on the same machine.
  std::this_thread::sleep_for(100ms);

  // Create a new String message and fill it with the data we want to send.
  std_msgs::msg::String message;
  message.data = "Hello, world!";

  // Put the new message on the publisher's queue
  publisher->publish(message);

  // "Spinning" handles ROS communication for a node.
  // In this case, it will process the publisher's message queue and send messages.
  // spin_once() will run just long enough to process all work, which in this case
  // means sending the message we just placed on the queue.
  rclcpp::spin_some(node);

  std::cout << "Published: " << message.data << std::endl;

  // Clean up
  rclcpp::shutdown();
  return 0;
}