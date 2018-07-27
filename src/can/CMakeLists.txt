
cmake_minimum_required(VERSION 2.8.3)
project(can)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(CATKIN_DEPENDS std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vehicle src/vehicle.cpp)
target_link_libraries(vehicle ${catkin_LIBRARIES} 
  /home/jizhi/projects/can_candy/src/can/lib/libcontrolcan.so)