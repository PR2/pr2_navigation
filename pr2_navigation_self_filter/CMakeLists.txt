# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_self_filter)

find_package(PkgConfig REQUIRED)

find_package(Boost REQUIRED COMPONENTS signals)

set(bullet_FOUND 0)
pkg_check_modules(bullet REQUIRED bullet)

find_package( PCL REQUIRED )
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS roscpp tf filters sensor_msgs urdf resource_retriever visualization_msgs pcl_ros)

catkin_package(
    DEPENDS bullet
    CATKIN-DEPENDS roscpp tf filters sensor_msgs urdf resource_retriever visualization_msgs pcl_ros
    INCLUDE_DIRS include
    LIBRARIES pr2_navigation_geometric_shapes ${PROJECT_NAME}
)

#common commands for building c++ executables and libraries
add_library(pr2_navigation_geometric_shapes src/load_mesh.cpp src/shapes.cpp src/bodies.cpp)

add_library(${PROJECT_NAME} src/self_mask.cpp)
target_link_libraries(${PROJECT_NAME} pr2_navigation_geometric_shapes)


add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME})
target_link_libraries(test_filter pr2_navigation_geometric_shapes)

add_executable(self_filter src/self_filter.cpp)
target_link_libraries(self_filter ${PROJECT_NAME})
target_link_libraries(self_filter pr2_navigation_geometric_shapes)

include_directories(include ${bullet_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(pr2_navigation_geometric_shapes ${bullet_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} test_filter self_filter pr2_navigation_geometric_shapes
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
