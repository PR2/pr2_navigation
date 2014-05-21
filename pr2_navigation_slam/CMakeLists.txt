# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_slam)

find_package(catkin REQUIRED COMPONENTS gmapping pr2_machine move_base pr2_navigation_config pr2_move_base topic_tools joint_trajectory_generator pr2_tuck_arms_action)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS gmapping pr2_machine move_base pr2_navigation_config pr2_move_base topic_tools joint_trajectory_generator pr2_tuck_arms_action
    INCLUDE_DIRS
    LIBRARIES
)

install(FILES  move_base.xml  slam_gmapping.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rviz config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
