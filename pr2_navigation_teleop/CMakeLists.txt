# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_teleop)

find_package(catkin REQUIRED COMPONENTS pr2_teleop pr2_machine topic_tools)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS pr2_teleop pr2_machine topic_tools
    INCLUDE_DIRS
    LIBRARIES
)

install(FILES teleop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})