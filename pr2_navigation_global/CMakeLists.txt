# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_global)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

install(FILES amcl_node.xml move_base.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config rviz
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
