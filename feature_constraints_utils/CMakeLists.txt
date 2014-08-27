cmake_minimum_required(VERSION 2.8.3)
project(feature_constraints_utils)

find_package(catkin REQUIRED COMPONENTS robohow_common_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS robohow_common_msgs 
)

