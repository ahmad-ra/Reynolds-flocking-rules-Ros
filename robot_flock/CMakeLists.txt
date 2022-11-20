cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)

project(robot_flock)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_flock_node src/robot_flock_node.cpp  src/robot.cpp src/robotRosInterface.cpp )

target_link_libraries(robot_flock_node ${catkin_LIBRARIES}   )



