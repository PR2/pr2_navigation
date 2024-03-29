# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_navigation_self_filter)

find_package(catkin REQUIRED COMPONENTS roscpp tf filters sensor_msgs urdf roscpp resource_retriever visualization_msgs pcl_ros)

find_package(PkgConfig REQUIRED)

find_package(Boost REQUIRED)

set(bullet_FOUND 0)
pkg_check_modules(bullet REQUIRED bullet)

find_package( PCL REQUIRED )
add_definitions(${PCL_DEFINITIONS})

if("$ENV{ROS_DISTRO}" STRLESS "melodic")
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150" )
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()


catkin_package(
    DEPENDS bullet
    CATKIN_DEPENDS roscpp tf filters sensor_msgs urdf resource_retriever visualization_msgs pcl_ros
    INCLUDE_DIRS include
    LIBRARIES pr2_navigation_geometric_shapes ${PROJECT_NAME}
)

#common commands for building c++ executables and libraries
add_library(pr2_navigation_geometric_shapes src/load_mesh.cpp src/shapes.cpp src/bodies.cpp)

add_library(${PROJECT_NAME} src/self_mask.cpp)
target_link_libraries(${PROJECT_NAME} pr2_navigation_geometric_shapes assimp)


add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME})
target_link_libraries(test_filter pr2_navigation_geometric_shapes  assimp)

add_executable(self_filter src/self_filter.cpp)
target_link_libraries(self_filter ${PROJECT_NAME})
target_link_libraries(self_filter pr2_navigation_geometric_shapes  assimp)

include_directories(include ${bullet_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(pr2_navigation_geometric_shapes ${bullet_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES}  assimp)


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} test_filter self_filter pr2_navigation_geometric_shapes
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
