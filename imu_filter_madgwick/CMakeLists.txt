cmake_minimum_required(VERSION 3.8)
project(imu_filter_madgwick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(imu_filter_madgwick SHARED
            src/imu_filter.cpp
            src/imu_filter_ros.cpp
            src/stateless_orientation.cpp)
target_compile_features(imu_filter_madgwick PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(imu_filter_madgwick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(imu_filter_madgwick
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(imu_filter_madgwick "ImuFilterMadgwickRos")
target_compile_definitions(imu_filter_madgwick
  PRIVATE "IMU_FILTER_MADGWICK_CPP_BUILDING_DLL")

add_executable(imu_filter_madgwick_node src/imu_filter_node.cpp)
target_include_directories(imu_filter_madgwick_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(imu_filter_madgwick_node imu_filter_madgwick)

install(
  TARGETS imu_filter_madgwick
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  imu_filter_madgwick_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(imu_filter_madgwick)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
