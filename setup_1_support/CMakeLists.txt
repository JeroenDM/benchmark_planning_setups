cmake_minimum_required(VERSION 2.8.3)
project(setup_1_support)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)