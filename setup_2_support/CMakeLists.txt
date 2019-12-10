cmake_minimum_required(VERSION 2.8.3)
project(setup_2_support)

find_package(catkin REQUIRED COMPONENTS
  xacro
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)