cmake_minimum_required(VERSION 2.8.3)
project(relocalization_server)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


catkin_package()


add_executable(relocalization_server_node 
	src/relocalize_publisher.cpp)

target_link_libraries(relocalization_server_node ${catkin_LIBRARIES} ) 
