cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(pr2_navigation_geometric_shapes src/load_mesh.cpp src/shapes.cpp src/bodies.cpp)

rosbuild_add_library(${PROJECT_NAME} src/self_mask.cpp)
target_link_libraries(${PROJECT_NAME} pr2_navigation_geometric_shapes)


rosbuild_add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME})
target_link_libraries(test_filter pr2_navigation_geometric_shapes)

rosbuild_add_executable(self_filter src/self_filter.cpp)
target_link_libraries(self_filter ${PROJECT_NAME})
target_link_libraries(self_filter pr2_navigation_geometric_shapes)
