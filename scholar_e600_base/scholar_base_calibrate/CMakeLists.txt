cmake_minimum_required(VERSION 3.0.2)
project(scholar_base_calibrate)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CalibrateLinear.cfg  cfg/CalibrateAngular.cfg)

catkin_package( CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs tf  sensor_msgs )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/calibrate_linear.py
  scripts/angular_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
