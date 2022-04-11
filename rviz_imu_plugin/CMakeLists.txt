cmake_minimum_required(VERSION 3.8)
project(rviz_imu_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED COMPONENTS rviz)
find_package(message_filters REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)


include_directories(include)

set(dependencies
    message_filters
    Qt5
    rviz_common
    rviz_ogre_vendor
    rviz_rendering
    sensor_msgs
    tf2
    tf2_ros
)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES  src/imu_display.cpp
               src/imu_orientation_visual.cpp
               src/imu_axes_visual.cpp
               src/imu_acc_visual.cpp
               src/mag_display.cpp
               src/mag_visual.cpp)

## Build libraries
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

#set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})

target_link_libraries(${PROJECT_NAME} PUBLIC rviz_ogre_vendor::OgreMain rviz_ogre_vendor::OgreOverlay Qt5::Widgets)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# must place the description in rviz_common package in order to have plugin recognized.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
    Qt5
    rviz_common
    sensor_msgs
    tf2
    tf2_ros
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
