cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(imu_complementary_filter)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES complementary_filter
  CATKIN_DEPENDS message_filters roscpp sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(complementary_filter
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
  include/imu_complementary_filter/complementary_filter.h
  include/imu_complementary_filter/complementary_filter_ros.h
)
target_link_libraries(complementary_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# create complementary_filter_node executable
add_executable(complementary_filter_node
  src/complementary_filter_node.cpp)
target_link_libraries(complementary_filter_node complementary_filter ${catkin_LIBRARIES})

install(TARGETS complementary_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS complementary_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
