cmake_minimum_required(VERSION 2.8.3)
project(rviz_imu_plugin)

find_package(catkin REQUIRED COMPONENTS rviz)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/rviz_imu_plugin/imu_display.h
)


catkin_package(
  DEPENDS 
  CATKIN_DEPENDS rviz
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Here we specify the list of source files
set(SOURCE_FILES  src/imu_display.cpp
                  src/imu_orientation_visual.cpp
                  src/imu_axes_visual.cpp
                  src/imu_acc_visual.cpp
                  ${MOC_FILES})

## Build libraries
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
