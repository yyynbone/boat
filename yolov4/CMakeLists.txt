cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(yolov4)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
include_directories(${PROJECT_SOURCE_DIR}/src)

FILE( GLOB source_src
  src/main.cc
  src/darknet.cc
  src/config.cc
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
#   geometry_msgs
#   message_generation
)

find_package( OpenCV REQUIRED )

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   yolov4.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
#   message_runtime
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  )

set(Torch_DIR /home/hzh/Desktop/catk/src/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

add_executable(main ${source_src})
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${TORCH_LIBRARIES} )
add_executable(pub src/publish.cpp)
target_link_libraries(pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# set_property(TARGET yolov4 PROPERTY CXX_STANDARD 14)
