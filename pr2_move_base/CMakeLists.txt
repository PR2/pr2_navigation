# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_move_base)

find_package(catkin REQUIRED COMPONENTS move_base_msgs pr2_msgs actionlib actionlib_msgs rospy dynamic_reconfigure pr2_controllers_msgs pr2_common_action_msgs)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS move_base_msgs pr2_msgs actionlib actionlib_msgs rospy dynamic_reconfigure pr2_controllers_msgs pr2_common_action_msgs
    INCLUDE_DIRS
    LIBRARIES
)

install(PROGRAMS  scripts/pr2_move_base.py
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
