# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_global)

find_package(catkin REQUIRED COMPONENTS move_base amcl pr2_machine pr2_navigation_config pr2_move_base topic_tools joint_trajectory_generator pr2_tuck_arms_action)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS move_base amcl pr2_machine pr2_navigation_config pr2_move_base topic_tools joint_trajectory_generator pr2_tuck_arms_action
    INCLUDE_DIRS
    LIBRARIES
)

install(FILES amcl_node.xml move_base.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config rviz
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
