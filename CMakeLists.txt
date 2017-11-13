cmake_minimum_required(VERSION 2.8.3)
project(truenorth)

#set (CMAKE_CXX_STANDARD 11)

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
   imu.msg
   imu_bias.msg
   kvh_imu.msg
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


## For Eigen library support
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include_directories( include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )


## Declare a cpp library
add_library(truenorth src/serial_io.cpp src/gyro_data.cpp src/att_est.cpp src/bias_est.cpp)
target_link_libraries(truenorth ${catkin_LIBRARIES})



add_executable(gyro_publisher src/gyro_publisher.cpp src/serial_io.cpp src/gyro_data.cpp src/att_est.cpp src/bias_est.cpp src/thread.cpp)
target_link_libraries(gyro_publisher ${catkin_LIBRARIES})
add_dependencies(gyro_publisher truenorth_generate_messages_cpp)
set_target_properties(gyro_publisher PROPERTIES COMPILE_FLAGS "-O3")

#add_executable(data_2_csv src/data_2_csv.cpp)
#target_link_libraries(data_2_csv ${catkin_LIBRARIES})
#add_dependencies(data_2_csv truenorth_generate_messages_cpp)

add_executable(post_process src/post_process.cpp src/att_est.cpp src/bias_est.cpp src/gyro_data.cpp)
target_link_libraries(post_process ${catkin_LIBRARIES})
set_target_properties(post_process PROPERTIES COMPILE_FLAGS "-O3")


add_executable(bias_process src/bias_process.cpp src/bias_est.cpp src/gyro_data.cpp)
target_link_libraries(bias_process ${catkin_LIBRARIES})
set_target_properties(bias_process PROPERTIES COMPILE_FLAGS "-O3")

add_executable(ekf_process src/ekf_process.cpp src/EKF.cpp src/gyro_data.cpp)
target_link_libraries(ekf_process ${catkin_LIBRARIES})
#set_target_properties(ekf_process PROPERTIES COMPILE_FLAGS "-O3")
