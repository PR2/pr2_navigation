# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(laser_tilt_controller_filter)

find_package(catkin REQUIRED COMPONENTS filters sensor_msgs roscpp pluginlib pr2_msgs)



catkin_package(
    DEPENDS 
    CATKIN-DEPENDS filters sensor_msgs roscpp pluginlib pr2_msgs
    INCLUDE_DIRS include
    LIBRARIES laser_tilt_controller_filter
)


add_library(laser_tilt_controller_filter src/laser_tilt_controller_filter.cpp)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(laser_tilt_controller_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS laser_tilt_controller_filter
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES  laser_tilt_controller_filter_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
