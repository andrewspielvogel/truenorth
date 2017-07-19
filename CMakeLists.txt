cmake_minimum_required(VERSION 2.8.3)
project(truenorth)

set (CMAKE_CXX_STANDARD 11)



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
add_library(truenorth  src/gyro_data.cpp src/att_est.cpp )
target_link_libraries(truenorth ${catkin_LIBRARIES})




add_executable(post_process src/post_process.cpp src/att_est.cpp src/bias_est.cpp src/gyro_data.cpp)
target_link_libraries(post_process ${catkin_LIBRARIES})
set_target_properties(post_process PROPERTIES COMPILE_FLAGS "-O3")
