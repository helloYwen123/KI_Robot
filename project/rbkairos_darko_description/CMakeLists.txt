cmake_minimum_required(VERSION 3.0.2)
project(rbkairos_darko_description)

find_package(catkin REQUIRED urdf xacro robotnik_sensors)

catkin_package(
  CATKIN_DEPENDS urdf xacro robotnik_sensors franka_description
)

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)