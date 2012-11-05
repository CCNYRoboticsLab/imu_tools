cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate dynamic parameters
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# create imu_filter library
rosbuild_add_library (imu_filter src/imu_filter.cpp)
rosbuild_link_boost(imu_filter signals)

# create imu_filter_nodelet library
rosbuild_add_library (imu_filter_nodelet src/imu_filter_nodelet.cpp)
target_link_libraries (imu_filter_nodelet imu_filter)

# create imu_filter_node executable
rosbuild_add_executable(imu_filter_node src/imu_filter_node.cpp)
target_link_libraries (imu_filter_node imu_filter)
