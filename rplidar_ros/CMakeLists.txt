cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

add_compile_options(-std=c++11)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosconsole
  sensor_msgs
  std_msgs
  geometry_msgs
  boat_msgs
  tf
  visualization_msgs
)

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)
if (OPENCV_FOUND)
    message(STATUS "找到OpenCV:\"${OpenCV_INCLUDE_DIRS}\",ENABLE_OPENCV宏已打开")
    message(STATUS "找到OpenCV:\"${OpenCV_LIBS}\"")
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_definitions(-DENABLE_OPENCV)
    list(APPEND LINK_LIB_LIST ${OpenCV_LIBS})
endif (OPENCV_FOUND)


include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

add_executable(rectangular src/maker.cpp)
target_link_libraries(rectangular ${catkin_LIBRARIES})

#add_executable(test_node src/laser_avoid.cpp)
#target_link_libraries(test_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(cluster src/cluster.cpp)
target_link_libraries(cluster ${catkin_LIBRARIES})

install(TARGETS rplidarNode rplidarNodeClient rectangular
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
