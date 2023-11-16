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

add_library(complementary_filter SHARED
  src/complementary_filter.cpp
  src/complementary_filter_ros.cpp
  include/imu_complementary_filter/complementary_filter.h
  include/imu_complementary_filter/complementary_filter_ros.h
)
target_compile_features(complementary_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(complementary_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(complementary_filter
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(complementary_filter PRIVATE "IMU_COMPLEMENTARY_FILTER_BUILDING_LIBRARY")

# create complementary_filter_node executable
add_executable(complementary_filter_node
  src/complementary_filter_node.cpp)
target_include_directories(complementary_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(complementary_filter_node complementary_filter)

install(
  TARGETS complementary_filter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
        complementary_filter_node
        DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(complementary_filter)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
