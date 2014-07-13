cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)
rosbuild_init()

find_package(PkgConfig)
pkg_check_modules(PC_EIGEN REQUIRED eigen3)
include_directories(${PC_EIGEN_INCLUDE_DIRS})
add_definitions(${PC_EIGEN_CFLAGS_OTHER})

rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#
rosbuild_add_executable(sac_inc_ground_removal_node src/sac_inc_ground_removal_standalone.cpp src/sac/sac.cpp src/sac/ransac.cpp src/sac/sac_model.cpp src/sac/sac_model_line.cpp)
rosbuild_link_boost(sac_inc_ground_removal_node signals)
