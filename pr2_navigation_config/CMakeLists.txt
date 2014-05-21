# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_config)

find_package(catkin REQUIRED COMPONENTS move_base)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS move_base
    INCLUDE_DIRS
    LIBRARIES
)

install(DIRECTORY move_base
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
