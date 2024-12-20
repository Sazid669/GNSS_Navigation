cmake_minimum_required(VERSION 3.0.2)
project(scout_mini_project)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  # pcl_ros
  # geometry_msgs
  # pcl_conversions
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

