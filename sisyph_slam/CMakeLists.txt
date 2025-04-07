cmake_minimum_required(VERSION 3.0.2)
project(sisyph_slam)


add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
)


# catkin_python_setup()




catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES sisyph_slam
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tfs_pub src/robot_tfs_pub.cpp)
target_link_libraries(tfs_pub ${catkin_LIBRARIES})

