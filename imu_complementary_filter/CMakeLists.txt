cmake_minimum_required(VERSION 3.8)
project(imu_complementary_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
)

## Declare a cpp library
add_library(complementary_filter
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
  include/imu_complementary_filter/complementary_filter.h
  include/imu_complementary_filter/complementary_filter_ros.h
)
#target_link_libraries(complementary_filter rclcpp sensor_msgs)


# create complementary_filter_node executable
add_executable(complementary_filter_node
  src/complementary_filter_node.cpp)
target_link_libraries(complementary_filter_node complementary_filter)
ament_target_dependencies(complementary_filter geometry_msgs rclcpp message_filters sensor_msgs std_msgs tf2 tf2_ros)
#ament_target_dependencies(complementary_filter_node)

install(TARGETS
        complementary_filter
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
        complementary_filter_node
        DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs message_filters sensor_msgs std_msgs tf2 tf2_ros)
ament_package()
