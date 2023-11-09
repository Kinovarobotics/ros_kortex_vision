cmake_minimum_required(VERSION 3.14)

project(kinova_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Ament System Dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  camera_calibration_parsers
  camera_info_manager
  cv_bridge
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
)

# Other Dependencies
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

# Define targets
include_directories(include)
include_directories(${GSTREAMER_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/vision.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
  "${GSTREAMER_LIBRARIES}")

add_executable(${PROJECT_NAME}_node src/vision_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Install directives
install(TARGETS ${PROJECT_NAME} 
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Include additional settings if needed
set(CMAKE_CXX_FLAGS "-fpermissive")

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
)
ament_package()
