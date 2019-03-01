cmake_minimum_required(VERSION 2.8.3)
project(stat_color)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  stat_color.srv
)


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(stat_color src/stat_color.cpp)
add_executable(record_hist src/record_hist.cpp)
target_link_libraries(stat_color ${catkin_LIBRARIES} )
target_link_libraries(record_hist ${catkin_LIBRARIES} )
add_dependencies(stat_color ${catkin_EXPORTED_TARGETS})
add_dependencies(record_hist ${catkin_EXPORTED_TARGETS})
