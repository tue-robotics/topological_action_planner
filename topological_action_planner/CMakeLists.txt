cmake_minimum_required(VERSION 3.0.2)
project(topological_action_planner)

find_package(catkin REQUIRED
  COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

catkin_install_python(PROGRAMS
  scripts/tap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
