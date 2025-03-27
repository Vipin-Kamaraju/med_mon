cmake_minimum_required(VERSION 3.0.2)
project(gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  medical_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)