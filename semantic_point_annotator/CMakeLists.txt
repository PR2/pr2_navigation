cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#
#rosbuild_add_executable(semantic_point_annotator_node src/semantic_point_annotator_omp.cpp)
#rosbuild_add_executable(sac_ground_removal_node src/sac_ground_removal.cpp)
#rosbuild_add_executable(sac_inc_ground_removal_node src/sac_inc_ground_removal.cpp)
rosbuild_add_executable(sac_inc_ground_removal_node src/sac_inc_ground_removal_standalone.cpp src/sac/sac.cpp src/sac/ransac.cpp src/sac/sac_model.cpp src/sac/sac_model_line.cpp)

# check for OpenMP
include(CheckIncludeFile)
include(CheckCXXCompilerFlag)
include(CheckLibraryExists)

#rosbuild_add_openmp_flags(semantic_point_annotator_node)
#rosbuild_add_openmp_flags(sac_ground_removal_node)
