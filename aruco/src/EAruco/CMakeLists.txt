cmake_minimum_required(VERSION 3.0.2)
project(EAruco)

#add_definitions(-std=c++11)
### Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
#add_definitions("-DENABLE_SSE")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  mavros_msgs
  geometry_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

find_package (OpenCV 4 REQUIRED)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS
roscpp std_msgs message_generation sensor_msgs)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(EAruco src/EAruco.cpp)
target_link_libraries(EAruco
${OpenCV_LIBRARIES}
${catkin_LIBRARIES}
)
