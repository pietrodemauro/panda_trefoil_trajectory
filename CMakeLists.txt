cmake_minimum_required(VERSION 3.0.2)
project(panda_trefoil_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_move_group
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trefoil_trajectory src/trefoil_trajectory.cpp)

target_link_libraries(trefoil_trajectory
  ${catkin_LIBRARIES}
)

install(TARGETS trefoil_trajectory
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

