cmake_minimum_required(VERSION 2.8.3)
project(rags_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  actionlib_msgs 
  actionlib
  dji_sdk
)
find_package(OpenCV 3.1.0 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
)

include_directories(
#  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(dji_rags src/dji_rags.cpp)
target_link_libraries(dji_rags ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(dji_rags dji_sdk_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(dji_drone_client src/dji_drone_client.cpp)
target_link_libraries(dji_drone_client ${catkin_LIBRARIES})
add_dependencies(dji_drone_client dji_sdk_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
