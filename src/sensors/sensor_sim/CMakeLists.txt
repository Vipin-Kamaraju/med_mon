cmake_minimum_required(VERSION 3.0.2)
project(sensor_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  medical_sensor.py
  ekg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)