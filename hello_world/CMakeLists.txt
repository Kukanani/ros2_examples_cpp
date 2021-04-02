##################################
# Header
##################################
cmake_minimum_required(VERSION 3.5)
project(ros2_testing)

##################################
# Dependencies
##################################
# First we need to make sure that our package's dependencies
# are installed and that we can find them.

# ament_cmake defines CMake macros that are needed for creating
# and depending on ament packages (such as ament_package(),
# which is used at the end of this file).
find_package(ament_cmake REQUIRED)

# rclcpp = ROS2 Client Library for C++.
# In other words, it's ROS2.
find_package(rclcpp REQUIRED)

# std_msgs defines some simple ROS2 messages that are useful
# for a wide variety of purposes.
find_package(std_msgs REQUIRED)

##################################
# Publisher node
##################################
# Create an executable from publisher.cpp
add_executable(publisher src/publisher.cpp)
# The executable depends on ROS2 and message definitions.
# These come from other packages and so they have to be
# added as depdendencies.
ament_target_dependencies(publisher rclcpp std_msgs)
# This line ensures that the publisher executable gets
# copied into the "install" directory of the ros2 workspace.
install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})

##################################
# Subscriber node
##################################
# Same as above, but for the subscriber.
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)
install(TARGETS
  subscriber
  DESTINATION lib/${PROJECT_NAME})

##################################
# ament package directive
##################################
# This line should go at the end. It tells ament to create
# a complete ament package using all the information we've
# already provided earlier in the file.
ament_package()
