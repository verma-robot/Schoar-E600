cmake_minimum_required(VERSION 3.0.2)
project(scholar_slam_gazebo)

find_package(catkin REQUIRED)

catkin_package( )

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

