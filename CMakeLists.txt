cmake_minimum_required(VERSION 3.0.2)
project(xloc_ros_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Diagnostics.msg
)

## Generate service files
add_service_files(
  FILES
  ActivateMap.srv
  StartMapping.srv
  StopMapping.srv
  StartLocalization.srv
  StopLocalization.srv
  SetInitialPose.srv
  ResetSlamError.srv
  GetCurrentPose.srv
  SwitchMap.srv
  ChangeMapOrigin.srv
  StartUpdateMap.srv
  StopUpdateMap.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_cpp/include
  # installed headers from xloc (if you ran make install)
  ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_install/include
  /usr/local/include
)

# Eigen include (for xloc headers)
include_directories(${EIGEN3_INCLUDE_DIRS})

## Create executable
add_executable(xloc_ros_node src/xloc_ros_node.cpp)

add_dependencies(xloc_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Try to find libxloc from common locations (optional)
find_library(XLOC_LIB xloc HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_install/lib /usr/local/lib /usr/lib)
if(XLOC_LIB)
  message(STATUS "Found libxloc: ${XLOC_LIB}")
  target_link_libraries(xloc_ros_node ${catkin_LIBRARIES} ${XLOC_LIB})
else()
  message(WARNING "libxloc not found; linking only against catkin libraries. If you have libxloc, set up link directories or install it to a standard location.")
  target_link_libraries(xloc_ros_node ${catkin_LIBRARIES})
endif()

install(TARGETS xloc_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
