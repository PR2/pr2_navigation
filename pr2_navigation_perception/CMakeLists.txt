# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_perception)

find_package(catkin REQUIRED COMPONENTS semantic_point_annotator laser_filters pr2_machine topic_tools pr2_navigation_self_filter geometry_msgs dynamic_reconfigure sensor_msgs roscpp tf filters message_filters laser_geometry laser_tilt_controller_filter pcl_ros)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS semantic_point_annotator laser_filters pr2_machine topic_tools pr2_navigation_self_filter geometry_msgs dynamic_reconfigure sensor_msgs roscpp tf filters message_filters laser_geometry laser_tilt_controller_filter pcl_ros
    INCLUDE_DIRS config/pr2_laser_filters/include
    LIBRARIES pr2_point_cloud_filters
)

include_directories(${PROJECT_SOURCE_DIR}/config/pr2_laser_filters/include ${catkin_INCLUDE_DIRS})
add_library(pr2_point_cloud_filters config/pr2_laser_filters/src/pr2_point_cloud_filters.cpp)
target_link_libraries(pr2_point_cloud_filters ${catkin_LIBRARIES})
add_dependencies(pr2_point_cloud_filters ${catkin_EXPORTED_TARGETS})

install(TARGETS pr2_point_cloud_filters
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES  ground_plane.xml lasers_and_filters.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
