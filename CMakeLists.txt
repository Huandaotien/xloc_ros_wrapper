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
  # installed headers from xloc (if you ran make install)
  ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_install/include
  /usr/local/include
  /usr/local/include/xloc
  /usr/include
  /usr/include/xloc
)

# Eigen include (for xloc headers)
include_directories(${EIGEN3_INCLUDE_DIRS})

## Create executable
add_executable(xloc_ros_node src/xloc_ros_node.cpp)

## C API based node
add_executable(xloc_ros_node_capi src/xloc_ros_node_c_api.cpp)

add_dependencies(xloc_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(xloc_ros_node_capi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Try to find libxloc from common locations (optional)
find_library(XLOC_LIB xloc HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_install/lib /usr/local/lib /usr/lib)
if(XLOC_LIB)
  message(STATUS "Found libxloc: ${XLOC_LIB}")
  target_link_libraries(xloc_ros_node ${catkin_LIBRARIES} ${XLOC_LIB})
  target_link_libraries(xloc_ros_node_capi ${catkin_LIBRARIES} ${XLOC_LIB})
else()
  message(WARNING "libxloc not found; linking only against catkin libraries. If you have libxloc, set up link directories or install it to a standard location.")
  target_link_libraries(xloc_ros_node ${catkin_LIBRARIES})
  target_link_libraries(xloc_ros_node_capi ${catkin_LIBRARIES})
endif()

## Prefer installed xloc headers/libs from /usr/local when available
find_path(XLOC_INCLUDE_DIR xloc_c_api.h HINTS /usr/local/include /usr/include ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_install/include)
if(XLOC_INCLUDE_DIR)
  message(STATUS "Found xloc headers: ${XLOC_INCLUDE_DIR}")
  include_directories(${XLOC_INCLUDE_DIR})
else()
  message(WARNING "xloc headers not found in /usr/local/include or install dir; falling back to source-tree include if present.")
endif()

link_directories(/usr/local/lib ${CMAKE_CURRENT_SOURCE_DIR}/../xloc_install/lib)

install(TARGETS xloc_ros_node xloc_ros_node_capi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
