cmake_minimum_required(VERSION 3.0.2)
project(ret_error_logger)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_logger
#  CATKIN_DEPENDS rosconsole rospy std_msgs
#  DEPENDS system_lib
)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(segfault_node src/ros_segfault.cpp)

target_link_libraries(segfault_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/ros_segfault
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
