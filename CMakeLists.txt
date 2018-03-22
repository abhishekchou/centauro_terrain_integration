cmake_minimum_required(VERSION 2.8.3)
project(centauro_terrain_integration)

set(PCL_DIR /usr/share/pcl-1.7)
set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  OpenCV
  PCL REQUIRED
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(time_sync src/time_sync.cpp)
#add_executable(validate_steps src/validate_steps.cpp)
target_link_libraries(time_sync  ${catkin_LIBRARIES})
add_dependencies(time_sync time_sync)
