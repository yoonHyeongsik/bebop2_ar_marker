cmake_minimum_required(VERSION 2.8.3)
project(bebop_ar_marker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 #  ar_sys
 ar_track_alvar
 #  goal_point
 #  ar_track_alvar_msg
 geometry_msgs
 message_generation
 nav_msgs
 roscpp
 rospy
 tf
)
#add_executable(
#  goal_point msg/points.msg
#)


catkin_package(
 LIBRARIES ar_recog
 CATKIN_DEPENDS std_msgs rospy goal_point
)

#add_dependencies(ar_recog ${catkin_EXPORTED_TARGETS})

include_directories(
 ${catkin_INCLUDE_DIRS}

)
