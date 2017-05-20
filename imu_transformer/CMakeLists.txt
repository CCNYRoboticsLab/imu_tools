cmake_minimum_required(VERSION 2.8.3)
project(imu_transformer)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_transformer_nodelet
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(imu_transformer_nodelet src/imu_transformer_nodelet.cpp)
target_link_libraries(imu_transformer_nodelet ${catkin_LIBRARIES})

add_executable(imu_transformer_node src/imu_transformer_node.cpp)
target_link_libraries(imu_transformer_node ${catkin_LIBRARIES})

install(TARGETS imu_transformer_node imu_transformer_nodelet
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})