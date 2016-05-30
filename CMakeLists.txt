cmake_minimum_required(VERSION 2.8.3)
project(truenorth)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  rosbag
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   gyro_sensor_data.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES truenorth
  CATKIN_DEPENDS message_runtime roscpp std_msgs rosbag
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## For Eigen library support
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories( include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )


## Declare a cpp library
add_library(truenorth src/serial_io.cpp src/gyro_data.cpp src/so3_adap_id.cpp)
target_link_libraries(truenorth ${catkin_LIBRARIES})

add_executable(gyro_publisher src/gyro_publisher.cpp src/serial_io.cpp src/gyro_data.cpp src/so3_adap_id.cpp)
target_link_libraries(gyro_publisher ${catkin_LIBRARIES})
add_dependencies(gyro_publisher truenorth_generate_messages_cpp)

add_executable(data_2_csv src/data_2_csv.cpp)
target_link_libraries(data_2_csv ${catkin_LIBRARIES})
add_dependencies(data_2_csv truenorth_generate_messages_cpp)

