cmake_minimum_required(VERSION 3.0.2)
project(launch_files)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)