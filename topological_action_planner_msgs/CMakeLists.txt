cmake_minimum_required(VERSION 3.5)
project(topological_action_planner_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation
)

add_message_files(
  FILES
  Edge.msg
  Node.msg
)

add_service_files(
  FILES
  Plan.srv
  UpdateEdge.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS message_runtime
  DEPENDS
)
