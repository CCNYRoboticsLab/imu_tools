cmake_minimum_required(VERSION 2.8.3)
project(imu_complementary_filter)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(Eigen REQUIRED COMPONENTS)

catkin_package(
  DEPENDS Eigen
  INCLUDE_DIRS include
  LIBRARIES complementary_filter complementary_filter_node
  CATKIN_DEPENDS message_filters roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(complementary_filter
    src/complementary_filter.cpp
    src/complementary_filter_ros.cpp
    src/complementary_filter_node.cpp
    include/imu_complementary_filter/complementary_filter.h
    include/imu_complementary_filter/complementary_filter_ros.h
 )


# create complementary_filter_node executable
add_executable(complementary_filter_node 
                        src/complementary_filter_node.cpp)

target_link_libraries(complementary_filter_node complementary_filter ${catkin_LIBRARIES} ${Eigen_LIBRARIES})



## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


