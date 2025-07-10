cmake_minimum_required(VERSION 3.0.2)
project(octomap_related_nodes)



## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  computation
  message_filters
)

## Find Octomap library
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_example
   CATKIN_DEPENDS octomap_msgs octomap_ros roscpp sensor_msgs computation pcl_ros pcl_conversions message_filters
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(predefined_octomap src/octomap_example_node.cpp)
add_executable(octomap_from_cloudpoint src/octomap_from_pointcloud.cpp)
add_executable(filter_pointcloud src/filter_point_cloud.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(predefined_octomap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(octomap_from_cloudpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(filter_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(predefined_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(octomap_from_cloudpoint
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(filter_pointcloud 
  ${catkin_LIBRARIES})