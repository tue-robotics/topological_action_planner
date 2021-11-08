# topological_action_planner

[![CI](https://github.com/tue-robotics/topological_action_planner/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/topological_action_planner/actions/workflows/main.yml) [![Lint](https://github.com/tue-robotics/topological_action_planner/actions/workflows/black.yml/badge.svg)](https://github.com/tue-robotics/topological_action_planner/actions/workflows/black.yml)

ROS package for planning over a topological navigation graph. 
Nodes are areas of interest in an ED world model and edges between these areas are labeled with the action to take to traverse the edge,
like opening a door, plain driving or pushing an object aside.
