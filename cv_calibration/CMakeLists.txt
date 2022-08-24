cmake_minimum_required(VERSION 3.0.2)
project(cv_calibration)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)
# find_package(OpenCV REQUIRED)
# find_package(video_stream_opencv REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)


###########
## Build ##
###########


# include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(include ${PROJECT_SOURCE_DIR}/src)
# include_directories(include ${PROJECT_SOURCE_DIR}/include)
# include_directories(include ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
# include_directories(include ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/src/ros_kortex/kortex_api/include/client)
# include_directories(include ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/src/ros_kortex/kortex_api/include/client_stubs)
# include_directories(include ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/src/ros_kortex/kortex_api/include/messages)
# include_directories(include ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/src/ros_kortex/kortex_api/include/common)
# include_directories(include ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/src/ros_kortex/my_pkg/include)
# include_directories(include ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/devel/include)


# catkin_install_python(PROGRAMS 
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# add_executable(calibration_cpp src/calibration.cpp)
# add_dependencies(calibration_cpp ${catkin_EXPORTED_TARGETS})
# add_library(acquisition src/acquisition.cpp)
# add_library(SimpleAcquisition src/SimpleAcquisition.cpp)
# add_library(processing src/processing.cpp)
# add_library(DisplayProcess src/DisplayProcess.cpp)
# add_library(ProcessingChain src/ProcessingChain.cpp)
# add_library(ColorDepthSingleFrameCapure src/ColorDepthSingleFrameCapure.cpp)
# add_library(EdgeDetectProcess src/EdgeDetectProcess.cpp)
# add_library(RGBEdgeDrawerProcess src/RGBEdgeDrawerProcess.cpp)
# add_library(RobotControl ${PROJECT_SOURCE_DIR}/../../../catkin_workspace/src/ros_kortex/my_pkg/src/RobotControl.cpp)
# target_link_libraries(RobotControl ${catkin_LIBRARIES})
# target_link_libraries(calibration_cpp ${catkin_LIBRARIES})
# target_link_libraries(calibration_cpp ${OpenCV_LIBRARIES})
# target_link_libraries(calibration_cpp RobotControl)
# target_link_libraries(calibration_cpp acquisition)
# target_link_libraries(calibration_cpp SimpleAcquisition)
# target_link_libraries(calibration_cpp processing)
# target_link_libraries(calibration_cpp DisplayProcess)
# target_link_libraries(calibration_cpp ProcessingChain)
# target_link_libraries(calibration_cpp ColorDepthSingleFrameCapure)
# target_link_libraries(calibration_cpp EdgeDetectProcess)
# target_link_libraries(calibration_cpp RGBEdgeDrawerProcess)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
  src/rviz_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(
#   DIRECTORY include/${PROJECT_NAME}/ 
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/
# )
# install(TARGETS
#   calibration_cpp
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############
