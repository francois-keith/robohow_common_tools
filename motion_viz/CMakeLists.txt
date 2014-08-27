cmake_minimum_required(VERSION 2.8.3)
project(motion_viz)

find_package(catkin REQUIRED COMPONENTS # std_msgs
)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS #message_runtime std_msgs
)

