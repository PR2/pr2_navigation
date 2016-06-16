# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(semantic_point_annotator)

find_package(catkin REQUIRED COMPONENTS roscpp tf pcl_ros)

find_package(PkgConfig)

find_package(Boost REQUIRED COMPONENTS signals)

add_definitions(${PC_EIGEN_CFLAGS_OTHER})

find_package( PCL REQUIRED )
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    DEPENDS  eigen
    CATKIN-DEPENDS roscpp tf pcl_ros
    INCLUDE_DIRS include
    LIBRARIES
)


add_executable(sac_inc_ground_removal_node src/sac_inc_ground_removal_standalone.cpp src/sac/sac.cpp src/sac/ransac.cpp src/sac/sac_model.cpp src/sac/sac_model_line.cpp)
include_directories(include/${PROJECT_NAME} ${Boost_INCLUDE_DIRS} ${PC_EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}  ${caktin_INCLUDE_DIRS})
target_link_libraries(sac_inc_ground_removal_node ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS sac_inc_ground_removal_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
