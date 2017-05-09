cmake_minimum_required(VERSION 2.8.3)
project(ece595project)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  gazebo_msgs
)

add_message_files(
	FILES
	movement_status.msg
	uav_status.msg
	swarm_start.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  init_sine_path.srv
  next_position.srv
  set_position.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(MasterHand src/MasterHand.cpp)
target_link_libraries(MasterHand ${catkin_LIBRARIES})
add_dependencies(MasterHand ece595project_generate_messages_cpp)

add_executable(movement_manager src/MovementManager.cpp)
target_link_libraries(movement_manager ${catkin_LIBRARIES})
add_dependencies(movement_manager ece595project_generate_message_cpp)

add_executable(quad_brain src/QuadBrain.cpp)
target_link_libraries(quad_brain ${catkin_LIBRARIES})
add_dependencies(quad_brain ece595project_generate_message_cpp)

add_executable(path_planner src/PathPlanner.cpp)
target_link_libraries(path_planner ${catkin_LIBRARIES})
add_dependencies(path_planner ece595project_generate_message_cpp)

add_executable(swarm_start_command src/SwarmStartCommand.cpp)
target_link_libraries(swarm_start_command ${catkin_LIBRARIES})
add_dependencies(swarm_start_command ece595project_generate_message_cpp)

