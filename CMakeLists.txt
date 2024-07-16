cmake_minimum_required(VERSION 3.1.2)
project(eagle_track)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3)
add_compile_options(-Wall)
add_compile_options(-Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  dynamic_reconfigure
  std_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  mrs_lib
  vofod
  lidar_tracker
  eagle_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  config/TrackParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Tracker
  CATKIN_DEPENDS roscpp mrs_lib image_transport std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(Tracker src/tracker.cpp)
add_dependencies(Tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
