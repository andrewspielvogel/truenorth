cmake_minimum_required(VERSION 2.8.3)
project(truenorth)
set (CMAKE_CXX_STANDARD 11)

#set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  rosbag
  helper_funcs
  dscl_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES truenorth
  CATKIN_DEPENDS message_runtime roscpp std_msgs rosbag helper_funcs dscl_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


## For Eigen library support
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories( include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )


## Declare a cpp library
#add_library(truenorth src/serial_io.cpp src/att_est.cpp src/fog_bias.cpp)
#target_link_libraries(truenorth ${catkin_LIBRARIES})



add_executable(gyro_publisher src/gyro_publisher.cpp src/serial_io.cpp src/att_est.cpp src/thread.cpp src/so3_att.cpp src/fog_bias.cpp)
target_link_libraries(gyro_publisher ${catkin_LIBRARIES})
add_dependencies(gyro_publisher dscl_msgs_generate_messages_cpp helper_funcs geometry_msgs_generate_messages_cpp)
set_target_properties(gyro_publisher PROPERTIES COMPILE_FLAGS "-O3")

add_executable(post_process src/post_process.cpp src/att_est.cpp src/so3_att.cpp src/fog_bias.cpp)
target_link_libraries(post_process ${catkin_LIBRARIES})
add_dependencies(post_process helper_funcs)
set_target_properties(post_process PROPERTIES COMPILE_FLAGS "-O3")
