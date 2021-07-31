cmake_minimum_required(VERSION 3.0.2)
project(topological_action_planner)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        std_msgs
        message_generation
)

# catkin_python_setup()

add_message_files(
  FILES
  Edge.msg
  Node.msg
)

add_service_files(
  FILES
  Plan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_topological_action_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
