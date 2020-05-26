cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(imu_filter_madgwick)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros nodelet pluginlib message_filters dynamic_reconfigure)

find_package(Boost REQUIRED COMPONENTS system thread)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(cfg/ImuFilterMadgwick.cfg)


catkin_package(
  DEPENDS Boost
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs nodelet pluginlib message_filters dynamic_reconfigure
  INCLUDE_DIRS
  LIBRARIES imu_filter imu_filter_nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


# create imu_filter library
add_library (imu_filter src/imu_filter.cpp  src/imu_filter_ros.cpp src/stateless_orientation.cpp)
add_dependencies(imu_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_nodelet library
add_library (imu_filter_nodelet src/imu_filter_nodelet.cpp)
add_dependencies(imu_filter_nodelet ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_nodelet imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# create imu_filter_node executable
add_executable(imu_filter_node src/imu_filter_node.cpp)
add_dependencies(imu_filter_node ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter_node imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS imu_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS imu_filter imu_filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES imu_filter_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-madgwick_test
    test/stateless_orientation_test.cpp
    test/madgwick_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}-madgwick_test
    imu_filter
    ${catkin_LIBRARIES}
  )
endif()
