cmake_minimum_required(VERSION 2.8.3)

project(kinova_vision)

# System Dependencies
find_package(PkgConfig)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# Use Catkin
find_package(catkin REQUIRED
  COMPONENTS roscpp image_transport sensor_msgs nodelet
  camera_calibration_parsers camera_info_manager tf
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES KinovaVision
  CATKIN_DEPENDS roscpp nodelet image_transport sensor_msgs
 camera_calibration_parsers camera_info_manager
  DEPENDS GSTREAMER GST_APP
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS})

add_library(KinovaVision src/vision.cpp)
target_link_libraries(KinovaVision
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES})

add_executable(KinovaVision_node src/vision_node.cpp)
target_link_libraries(KinovaVision_node KinovaVision
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES})
set_target_properties(KinovaVision_node PROPERTIES OUTPUT_NAME kinova_vision)

# Install directives
install(TARGETS KinovaVision KinovaVision_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.ini"
  )

set(CMAKE_CXX_FLAGS "-fpermissive")