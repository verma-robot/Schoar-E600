cmake_minimum_required(VERSION 2.8.3)
project(scholar_base_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslaunch
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs  

)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(scholar_bringup
               src/scholar_bringup.cpp
               src/scholar.cpp
 )
add_dependencies(scholar_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(scholar_bringup ${catkin_LIBRARIES} ${Boost_LIBRARIES})
